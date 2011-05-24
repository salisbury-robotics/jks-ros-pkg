#ifndef VOLUMEREADER_H
#define VOLUMEREADER_H

// VolumeReader is intended to be an abstract class that can read a volume
// from a file or directory into a VolumeData object.

#include "Common/Logger.h"
#include "Data/VolumeData.h"

class VolumeReader : public Logger
{
protected:
    Reporter   *m_reporter;

public:
            VolumeReader()  { m_reporter = 0; }
    virtual ~VolumeReader() { }
    virtual VolumeData *readFile(const std::string &filename)   { return 0; }
    virtual VolumeData *readDirectory(const std::string &path)  { return 0; }
    void setReporter(Reporter *reporter) { m_reporter = reporter; }

};

#endif // VOLUMEREADER_H
