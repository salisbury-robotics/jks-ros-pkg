#include "DicomReader.h"

#include "itkOrientedImage.h"
#include "itkImageSeriesReader.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"

#include <sstream>
#include <algorithm>
#include <cml/cml.h>

using namespace std;
using namespace itk;


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

ostream &operator<<(ostream &stream, const DicomInfo &info)
{
    stream << info.uid << ':' << endl;
    stream << '\t' << info.dimensions[0] << 'x' << info.dimensions[1] << 'x' << info.dimensions[2]
           << '\t' << info.modality << " (" << info.bitsAllocated << (info.pixelRepresentation ? 's' : 'u') << ')'
           << '\t' << info.seriesDescription << ',' << info.studyDescription << endl;
    stream << '\t' << "( position=" << info.imagePositionPatient
                   << ", orientation=" << info.imageOrientationPatient << ')' << endl;
    return stream;
}

// --------------------------------------------------------------------------

DicomReader::DicomReader()
{
    Logger::setClassName("DicomReader");
}

// --------------------------------------------------------------------------

vector<string> DicomReader::getAvailableSeries(const string &path)
{
    // remember the requested directory for the next few queries
    m_directory = path;

    // create a series filenames object to look for available series
    GDCMSeriesFileNames::Pointer scavenger = GDCMSeriesFileNames::New();
    scavenger->RecursiveOn();
    scavenger->SetInputDirectory(m_directory);
    return scavenger->GetSeriesUIDs();
}

// --------------------------------------------------------------------------

DicomInfo DicomReader::getSeriesInformation(const string &uid)
{
    // work with the previously queried directory
    GDCMSeriesFileNames::Pointer scavenger = GDCMSeriesFileNames::New();
    scavenger->RecursiveOn();
    scavenger->SetInputDirectory(m_directory);
    vector<string> filenames = scavenger->GetFileNames(uid);

    DicomInfo info;
    if (!filenames.empty())
    {
        GDCMImageIO::Pointer io = GDCMImageIO::New();
        io->SetFileName(filenames[0]);
        io->ReadImageInformation();

        info.uid = uid;
        io->GetValueFromTag("0008|0060", info.modality);
        io->GetValueFromTag("0008|1030", info.studyDescription);
        io->GetValueFromTag("0008|103e", info.seriesDescription);

        io->GetValueFromTag("0020|0032", info.imagePositionPatient);
        io->GetValueFromTag("0020|0037", info.imageOrientationPatient);

        string number;
        io->GetValueFromTag("0028|0100", number);
        istringstream(number) >> info.bitsAllocated;
        io->GetValueFromTag("0028|0101", number);
        istringstream(number) >> info.bitsStored;
        io->GetValueFromTag("0028|0103", number);
        istringstream(number) >> info.pixelRepresentation;

        info.dimensions[0] = io->GetDimensions(0);
        info.dimensions[1] = io->GetDimensions(1);
        if (io->GetDimensions(2) != 1)
            info.dimensions[2] = io->GetDimensions(2);
        else
            info.dimensions[2] = filenames.size();
    }

    return info;
}

// --------------------------------------------------------------------------

VolumeData *DicomReader::readDirectory(const string &path)
{
    m_directory = path;

    // get available series UIDs
    GDCMSeriesFileNames::Pointer scavenger = GDCMSeriesFileNames::New();
    scavenger->RecursiveOn();
    scavenger->SetInputDirectory(m_directory);
    vector<string> uids = scavenger->GetSeriesUIDs();

    // bail if nothing is available
    if (uids.empty()) {
        log() << "No series found at path " << path << endl;
        return 0;
    }

    // check if selected UID is in those found, and if not, select first series
    if (find(uids.begin(), uids.end(), m_selectedUID) == uids.end())
        m_selectedUID = uids[0];

    // get series info and filenames
    DicomInfo info = getSeriesInformation(m_selectedUID);
    vector<string> filenames = scavenger->GetFileNames(m_selectedUID);

    // output a log message
    log() << "Attempting to read series" << endl
          << info << endl;

    // dispatch the image reading function depending on the type of data
    VolumeData *vd = 0;
    if (info.bitsAllocated == 16)
        vd = readVolume<signed short>(filenames);
    else if (info.bitsAllocated == 8)
        vd = readVolume<unsigned char>(filenames);
    else {
        log() << "[ERROR] Unsupported image type!" << endl
              << info << endl;
    }

    // ascertain the root directory path of this particular series
    size_t p = filenames[0].rfind('/');
    string seriesPath = filenames[0].substr(0, p);

    // set additional fields in the volume based on DICOM header info
    vd->getVolume()->name = info.seriesDescription;
    vd->getVolume()->path = seriesPath;
    setVolumePose(vd->getVolume(), info);

    return vd;
}

// --------------------------------------------------------------------------

template <class PixelType>
VolumeData *DicomReader::readVolume(const vector<string> &filenames)
{
    // create an image series reader of the desired type
    typedef OrientedImage<PixelType, 3> ImageType;
    typedef ImageSeriesReader<ImageType> ReaderType;
    typename ReaderType::Pointer reader = ReaderType::New();

    // set up progress reporting
    if (m_reporter)
    {
        CStyleCommand::Pointer command = CStyleCommand::New();
        command->SetClientData(m_reporter);
        command->SetCallback(ProgressCallback);
        reader->AddObserver(ProgressEvent(), command);
        m_reporter->start("Load Volume", "Reading DICOM series...");
    }

    // read the image series
    reader->SetImageIO(GDCMImageIO::New());
    reader->SetFileNames(filenames);
    reader->Update();

    if (m_reporter) m_reporter->finish();

    // create a new VolumeData object and return
    return new VolumeITK<PixelType>(reader->GetOutput());
}

// --------------------------------------------------------------------------

void DicomReader::setVolumePose(Volume *v, const DicomInfo &info)
{
    // extract the position vector from the DICOM field
    istringstream iss(info.imagePositionPatient);
    cml::vector3f position;
    char backslash;
    iss >> position[0] >> backslash;
    iss >> position[1] >> backslash;
    iss >> position[2];

    // translate to physical origin (of the patient?)
    cml::matrix44f_c translate;
    cml::matrix_translation(translate, position);

    // TODO: do something with the orientation matrix?

    // scale to physical dimensions from 0-1 cube
    cml::vector3f size(v->dimensions[0] * v->spacing[0],
                       v->dimensions[1] * v->spacing[1],
                       v->dimensions[2] * v->spacing[2]);
    int i = cml::index_of_max(size[0], size[1], size[2]);

    cml::matrix44f_c scale;
    cml::matrix_uniform_scale(scale, size[i]);

    // set the volume's transform as the composition of the matrices
    v->transform = translate * scale;
}

// --------------------------------------------------------------------------
