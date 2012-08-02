#include "LabelsVolumeGenerator.h"
#include <cassert>
#include <cstring>

#include <itkImage.h>
#include <itkCommand.h>
#include <itkGrayscaleDilateImageFilter.h>
#include <itkBinaryBallStructuringElement.h>
#include <itkBinaryThresholdImageFilter.h>
#include <itkDiscreteGaussianImageFilter.h>
#include <itkSmoothingRecursiveGaussianImageFilter.h>

using namespace std;
using namespace cml;
using namespace itk;

LabelsVolumeGenerator::LabelsVolumeGenerator(Volume     *targetVolume,
                                             Volume     *targetMask,
                                             Labelling  *targetLabelling)
{
    m_targetVolume      = targetVolume;
    m_targetMask        = targetMask;
    m_targetLabelling   = targetLabelling;
    m_progressReporter  = 0;

    // the target volume should be allocated as a UInt8 date type
    assert(m_targetVolume->format == Volume::pfUInt8);

    // allocate a local buffer to accumulate labels in, just in case the target
    // volume is also on the list of volumes to combine
    const vector3i &d = m_targetVolume->dimensions;
    m_bufferSize = d[0]*d[1]*d[2];
    m_buffer = new unsigned char [ m_bufferSize ];
    memset(m_buffer, 0, m_bufferSize);

    // initialize a 0 offset
    m_offset = 0;
}

LabelsVolumeGenerator::~LabelsVolumeGenerator()
{
    if (m_buffer) delete [] m_buffer;
}

bool LabelsVolumeGenerator::addLabelVolume(Volume *v, Labelling *lab)
{
    // check that the dimensions match
    if (v->dimensions != m_targetVolume->dimensions) return false;

    // check that the volume has a small, finite band of intensities
    if (!lab->threshold && v->histogram.maxValue > k_bandLimit) return false;

    // combine this volume into the buffer
    int t = lab->threshold ? lab->thresholdValue : k_noThreshold;
    switch (v->format) {
    case Volume::pfUInt8:   bufferCombine<unsigned char>(v, t);     break;
    case Volume::pfInt16:   bufferCombine<signed short>(v, t);      break;
    case Volume::pfUInt16:  bufferCombine<unsigned short>(v, t);    break;
    default: /* error! */   return false;
    }

    // increment offset and append new entries to colour LUT
    if (lab->threshold) {
        ++m_offset;
        m_table.push_back(lab->colourLUT.front());
    }
    else {
        m_offset += lab->entries();
        m_table.insert(m_table.end(), lab->colourLUT.begin(), lab->colourLUT.end());
    }

    return true;
}

void LabelsVolumeGenerator::commitToTarget()
{
    // copy the buffer data into the target volume
    memcpy(m_targetVolume->data, m_buffer, m_bufferSize);

    // copy the new colour lookup table
    m_targetLabelling->colourLUT = m_table;

    // perform post-processing on the volume so that it is ready for volume rendering
    postProcessTarget();
}

void LabelsVolumeGenerator::postProcessTarget()
{
    // ITK version is much faster than doing it manually!
    postProcessTargetITK();
/*
    // copy the data to the buffer and perform dilation operation going back
    memcpy(m_buffer, m_targetVolume->data, m_bufferSize);
    unsigned char *p = reinterpret_cast<unsigned char *>(m_targetVolume->data);
    for (int z = 0; z < m_targetVolume->dimensions[2]; ++z)
        for (int y = 0; y < m_targetVolume->dimensions[1]; ++y)
            for (int x = 0; x < m_targetVolume->dimensions[0]; ++x)
                *p++ = bufferNeighbourhoodMax(vector3i(x,y,z));
*/
    // generate an alpha mask by slightly blurring a binary version of the
    // label volume

    m_targetLabelling->processed = true;
}

unsigned char LabelsVolumeGenerator::bufferNeighbourhoodMax(const cml::vector3i &x)
{
    // delta offsets from the center voxel
    static cml::vector3i delta[27];
    static bool initialized = false;

    // first-time initialization
    if (!initialized) {
        for (int i = 0, z = -1; z <= 1; ++z)
            for (int y = -1; y <= 1; ++y)
                for (int x = -1; x <= 1; ++x, ++i)
                    delta[i].set(x, y, z);
        initialized = true;
    }

    // find the highest of the 27 voxels in the region
    unsigned char hi = 0;
    vector3i lower(0,0,0);
    vector3i upper = m_targetVolume->dimensions - vector3i(1,1,1);
    for (int i = 0; i < 27; ++i) {
        vector3i p = x + delta[i];
        p.maximize(lower);
        p.minimize(upper);
        hi = max(hi, bufferVoxel(p));
    }

    return hi;
}

// --------------------------------------------------------------------------
// ITK image processing methods below here...
// --------------------------------------------------------------------------

static void ProgressCallback(Object *object, const EventObject &event, void *clientdata)
{
    ProcessObject *process = dynamic_cast<ProcessObject *>(object);
    Reporter *reporter = reinterpret_cast<Reporter *>(clientdata);
    if (process && reporter) {
        float progress = process->GetProgress();
        reporter->report(int(progress * 100.f));
    }
}

// --------------------------------------------------------------------------

void LabelsVolumeGenerator::postProcessTargetITK()
{
    typedef unsigned char       PixelType;
    typedef Image<PixelType, 3> ImageType;

    // create a new image from the target data

    ImageType::Pointer input = ImageType::New();

    ImageType::IndexType start;
    start.Fill(0);

    ImageType::SizeType size;
    for (int i = 0; i < 3; ++i) size[i] = m_targetVolume->dimensions[i];

    ImageType::RegionType region;
    region.SetSize(size);
    region.SetIndex(start);

    input->SetRegions(region);
    input->SetSpacing(m_targetVolume->spacing.data());
    input->Allocate();

    memcpy(input->GetBufferPointer(), m_targetVolume->data, m_bufferSize);

    // create a grayscale dilation filter and a structuring element

    typedef BinaryBallStructuringElement<PixelType, 3> StructureType;
    typedef GrayscaleDilateImageFilter<ImageType, ImageType, StructureType> DilateFilterType;

    DilateFilterType::Pointer filter = DilateFilterType::New();

    StructureType structure;
    structure.SetRadius(1);

    filter->SetKernel(structure);

    // set up progress reporting
    if (m_progressReporter)
    {
        CStyleCommand::Pointer command = CStyleCommand::New();
        command->SetClientData(m_progressReporter);
        command->SetCallback(ProgressCallback);
        filter->AddObserver(ProgressEvent(), command);
        m_progressReporter->start("Post-Processing Label volume",
                                  "Dilating label field...");
    }

    // hook up the filters and run

    filter->SetInput(input);
    filter->Update();

    if (m_progressReporter)
        m_progressReporter->finish();

    // copy back into the target volume data
    memcpy(m_targetVolume->data, filter->GetOutput()->GetBufferPointer(), m_bufferSize);

    // threshold and gaussian blur to put a smooth version into the alpha channel

    typedef BinaryThresholdImageFilter<ImageType, ImageType> ThresholderType;
//    typedef DiscreteGaussianImageFilter<ImageType, ImageType> SmoothFilterType;
    typedef SmoothingRecursiveGaussianImageFilter<ImageType, ImageType> SmoothFilterType;

    ThresholderType::Pointer thresholder = ThresholderType::New();
    thresholder->SetLowerThreshold(1);
    thresholder->SetUpperThreshold(255);
    thresholder->SetInsideValue(255);
    thresholder->SetOutsideValue(0);

    SmoothFilterType::Pointer smoother = SmoothFilterType::New();
//    smoother->SetVariance(0.05); // in physical units
//    smoother->SetMaximumKernelWidth(5);
    smoother->SetSigma(0.2);

    // set up progress reporting (again)
    if (m_progressReporter)
    {
        CStyleCommand::Pointer command = CStyleCommand::New();
        command->SetClientData(m_progressReporter);
        command->SetCallback(ProgressCallback);
        smoother->AddObserver(ProgressEvent(), command);
        m_progressReporter->start("Post-Processing Label volume",
                                  "Smoothing alpha mask...");
    }

    // hook up the filters and run

    thresholder->SetInput(input);
    smoother->SetInput(thresholder->GetOutput());
    smoother->Update();

    // copy back into the target volume data
    memcpy(m_targetMask->data, smoother->GetOutput()->GetBufferPointer(), m_bufferSize);

    if (m_progressReporter)
        m_progressReporter->finish();
}
