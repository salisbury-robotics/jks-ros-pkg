#ifndef VOLUMEREPOSITORY_H
#define VOLUMEREPOSITORY_H

#include "Common/Logger.h"
#include "Common/Reporter.h"
#include "Common/Singleton.h"
#include "VolumeData.h"
#include "DicomReader.h"

#include <string>
#include <vector>

class VolumeRepository : public Singleton<VolumeRepository>, public Logger
{
    // vector of all volumes currently loaded
    std::vector<VolumeData*> m_volumes;

    // optional callback that allows the user to select a single DICOM series
    // within a directory (possibly recursive) that contains several
    std::string (*m_dicomSeriesSelector)(const std::vector<DicomInfo> &);

    // a progress reporting class
    Reporter *m_progressReporter;

public:
    VolumeRepository();

    // enumeration of different file formats the repository can load
    enum Format { FormatUnknown, FormatDICOM, FormatSentinel };

    bool loadFromDirectory(const std::string &path, Format format = FormatUnknown);
    bool addCustomVolume(const Volume &header, const void *data);
    bool addTestVolume(int size = 64);

    // methods for accessing volumes stored in the repository
    int getNumberVolumes() const        { return m_volumes.size(); }
    Volume *getVolume(int index) const  { return m_volumes[index]->getVolume(); }
    Volume *getMask(int index) const    { return m_volumes[index]->getMask(); }
    Labelling *getLabelling(int index) const
                                        { return m_volumes[index]->getLabelling(); }

    // removes a number of volumes, beginning at index
    bool removeVolume(int index, int n = 1);

    // sets the dicom series selector callback
    void setDicomSeriesSelector(std::string (*callback)(const std::vector<DicomInfo> &))
                                        { m_dicomSeriesSelector = callback; }

    void setProgressReporter(Reporter *reporter) { m_progressReporter = reporter; }
};

#endif // VOLUMEREPOSITORY_H
