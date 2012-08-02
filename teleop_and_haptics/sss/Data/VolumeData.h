#ifndef VOLUMEDATA_H
#define VOLUMEDATA_H

#include "Common/Volume.h"
#include "Common/Labelling.h"
#include "Common/Reporter.h"

#include "itkOrientedImage.h"

// --------------------------------------------------------------------------
// The VolumeData class mainly acts as an abstract class so that a container
// can hold a heterogeneous set of volumes.  The class, or some structure
// within, is what actually owns the volume's memory in the application.

class VolumeData
{
protected:
    Volume          m_volume;
    Volume          m_mask;
    unsigned char  *m_maskData;
    Labelling       m_labelling;

    virtual void    createMask();
    virtual void    createLabelling() {
        m_labelling.initializeLUT(m_volume.histogram.maxValue);
    }

public:
    VolumeData() : m_maskData(0)    { }
    virtual ~VolumeData()           { if (m_maskData) delete [] m_maskData; }
    virtual Volume *getVolume()     { return &m_volume; }
    virtual Volume *getMask() {
        if (m_maskData == 0) createMask();
        return &m_mask;
    }
    virtual Labelling *getLabelling() {
        if (m_labelling.entries() == 0) createLabelling();
        return &m_labelling;
    }
};

// --------------------------------------------------------------------------

// TODO: maybe this should be an instance of VolumeITK in the future...
class VolumeTestData : public VolumeData
{
    typedef unsigned short PixelType;

    int m_size;
    PixelType *m_data;

public:
    VolumeTestData(int size = 128);
    ~VolumeTestData() { if (m_data) delete [] m_data; }
};

// --------------------------------------------------------------------------

class VolumePointData : public VolumeData
{
    typedef unsigned short PixelType;

    int m_size;
    PixelType *m_data;

public:
    VolumePointData(int size = 128);
    ~VolumePointData() { if (m_data) delete [] m_data; }
};

// --------------------------------------------------------------------------

template <class PixelType>
class VolumeCustomData : public VolumeData
{
    PixelType *m_data;

public:
    VolumeCustomData(const Volume &header, const void *data)
    {
        m_volume = header;
        if (data)
        {
            // if the header has allocated (non-zero) pointer, then just take
            // ownership rather than copy the data
            if (header.data == data) {
                m_data = reinterpret_cast<PixelType *>(header.data);
            }
            else {
                int size = header.dimensions[0]
                         * header.dimensions[1]
                         * header.dimensions[2];
                m_data = new PixelType[size];
                memcpy(m_data, data, size * sizeof(PixelType));
            }
        }
        else m_data = 0;
        m_volume.data = m_data;
        m_volume.computeHistogram();
    }

    ~VolumeCustomData() { if (m_data) delete [] m_data; }
};

// --------------------------------------------------------------------------

template <class PixelType>
class VolumeITK : public VolumeData
{
    typedef itk::OrientedImage<PixelType, 3> ImageType;
    typename ImageType::Pointer m_image;

public:
    VolumeITK(typename ImageType::Pointer image, const std::string &name = "ITK Image");
};

// --------------------------------------------------------------------------
#endif // VOLUMEDATA_H
