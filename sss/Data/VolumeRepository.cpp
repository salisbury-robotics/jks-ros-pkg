#include "VolumeRepository.h"
#include <algorithm>

using namespace std;

// --------------------------------------------------------------------------
// singleton instance

template < >
VolumeRepository *Singleton<VolumeRepository>::m_instance = 0;

// --------------------------------------------------------------------------

VolumeRepository::VolumeRepository()
{
    Logger::setClassName("VolumeRepository");

    m_dicomSeriesSelector = 0;
    m_progressReporter = 0;

    // the first volume is a dummy, empty (null) volume
    Volume header;
    header.name = "(none)";
    header.dimensions = cml::vector3i(0, 0, 0);
    header.spacing = cml::vector3f(0.f, 0.f, 0.f);
    header.format = Volume::pfUInt8;
    m_volumes.push_back(new VolumeCustomData<unsigned char>(header, 0));
}

// --------------------------------------------------------------------------

bool VolumeRepository::loadFromDirectory(const string &path, Format format)
{
    log() << "Attempting to load data from path " << path << endl;

    switch (format)
    {
    case FormatUnknown:
    case FormatDICOM:
        {
            DicomReader *reader = new DicomReader();
            reader->setReporter(m_progressReporter);

            // determine what series are available in the given directory
            vector<string> uids = reader->getAvailableSeries(path);

            // if there are more than one series, prompt user for selection
            if (uids.size() > 1 && m_dicomSeriesSelector)
            {
                log() << "Found " << uids.size() << " series in path." << endl;
                vector<DicomInfo> series;
                for (vector<string>::iterator it = uids.begin(); it != uids.end(); ++it)
                    series.push_back(reader->getSeriesInformation(*it));
                string uid = m_dicomSeriesSelector(series);
                if (!uid.empty())   reader->selectSeries(uid);
                else                return false;
            }

            VolumeData *volume = reader->readDirectory(path);
            if (volume) m_volumes.push_back(volume);

            // get the output log from the dicom reader
            Logger::coalesce(reader);
        }
        break;
    default:
        log() << "[ERROR] Unknown file format!" << endl;
    }

    return true;
}

// --------------------------------------------------------------------------

bool VolumeRepository::addCustomVolume(const Volume &header, const void *data)
{
    VolumeData *volume;
    if (header.format == Volume::pfInt16)
        volume = new VolumeCustomData<signed short>(header, data);
    else if (header.format == Volume::pfUInt8)
        volume = new VolumeCustomData<unsigned char>(header, data);
    else if (header.format == Volume::pfUInt16)
        volume = new VolumeCustomData<unsigned short>(header, data);
    else {
        log() << "[ERROR] Unsupported custom volume type" << endl;
        return false;
    }
    m_volumes.push_back(volume);
    return true;
}

// --------------------------------------------------------------------------

bool VolumeRepository::addTestVolume(int size)
{
    m_volumes.push_back(new VolumeTestData(size));
    m_volumes.push_back(new VolumePointData(size));
    return true;
}

// --------------------------------------------------------------------------

bool VolumeRepository::removeVolume(int index, int n)
{
    // don't allow the null volume (index 0) to be removed
    if (index < 1 || index+n > m_volumes.size())
        return false;

    // deallocate volume at the specified index, then remove it from the array
    for (int i = 0; i < n; ++i) delete m_volumes[index+i];
    copy(m_volumes.begin()+index+n, m_volumes.end(), m_volumes.begin()+index);
    for (int i = 0; i < n; ++i) m_volumes.pop_back();

    return true;
}

// --------------------------------------------------------------------------
