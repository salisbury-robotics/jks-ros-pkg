#ifndef DICOMREADER_H
#define DICOMREADER_H

#include "Data/VolumeReader.h"

#include <vector>
#include <ostream>

// --------------------------------------------------------------------------

struct DicomInfo
{
    std::string     uid;
    std::string     modality;               // 0008,0060
    std::string     studyDescription;       // 0008,1030
    std::string     seriesDescription;      // 0008,103E
    std::string     imagePositionPatient;   // 0020,0032
    std::string     imageOrientationPatient;// 0020,0037
    int             bitsAllocated;          // 0028,0100
    int             bitsStored;             // 0028,0101
    bool            pixelRepresentation;    // 0028,0103  (0=unsigned, 1=signed)
    int             dimensions[3];
};

std::ostream &operator<<(std::ostream &stream, const DicomInfo &info);

// --------------------------------------------------------------------------

class DicomReader : public VolumeReader
{
    std::string     m_directory;
    std::string     m_selectedUID;

    template <class PixelType>
    VolumeData *readVolume(const std::vector<std::string> &filenames);

    // this function's job is to ready position/orientation from the DICOM
    // info structure and set the vector/matrix in the volume
    void setVolumePose(Volume *v, const DicomInfo &info);

public:
    DicomReader();

    // returns a list of UIDs for the series available in the given path
    std::vector<std::string> getAvailableSeries(const std::string &path);

    // attempts to retrieve series information for a given UID
    DicomInfo getSeriesInformation(const std::string &uid);

    // select the series to load if there are more than one in a directory
    void selectSeries(const std::string &uid) { m_selectedUID = uid; }

    // implmentations of the virtual read functions
    virtual VolumeData *readDirectory(const std::string &path);
};

// --------------------------------------------------------------------------
#endif // DICOMREADER_H
