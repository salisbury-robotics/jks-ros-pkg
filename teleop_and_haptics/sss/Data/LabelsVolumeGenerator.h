#ifndef LABELSVOLUMEGENERATOR_H
#define LABELSVOLUMEGENERATOR_H

#include "Common/Volume.h"
#include "Common/Labelling.h"
#include "Common/Reporter.h"

class LabelsVolumeGenerator
{
    Volume          *m_targetVolume;
    Volume          *m_targetMask;
    Labelling       *m_targetLabelling;

    unsigned char   *m_buffer;
    int              m_bufferSize;
    int              m_offset;
    std::vector<cml::vector4f> m_table;

    static const int k_bandLimit = 24;
    static const int k_noThreshold = -0x10000;

    // for progress reporting
    Reporter        *m_progressReporter;

    // combines a label volume's data into the buffer
    template <class T>
    void bufferCombine(Volume *v, int threshold = k_noThreshold) {
        unsigned char *p = m_buffer;
        const T *q = reinterpret_cast<const T *>(v->data);
        if (threshold == k_noThreshold) {
            for (int i = 0; i < m_bufferSize; ++i, ++p, ++q)
                if (*q > 0) *p = *q + m_offset;
        }
        else {
            for (int i = 0; i < m_bufferSize; ++i, ++p, ++q)
                if (*q >= threshold) *p = m_offset + 1;
        }
    }

    // helper function to read a voxel from the buffer
    unsigned char bufferVoxel(const cml::vector3i &x) {
        const cml::vector3i &d = m_targetVolume->dimensions;
        unsigned char *p = m_buffer + (x[2] * d[1] + x[1]) * d[0] + x[0];
        return *p;
    }

    // returns the maximum value of the buffer within a 3x3x3 neighbourhood
    unsigned char bufferNeighbourhoodMax(const cml::vector3i &x);

    // ITK version of post-processing... is it any faster? (yes!)
    void postProcessTargetITK();

public:
    LabelsVolumeGenerator(Volume    *targetVolume,
                          Volume    *targetMask,
                          Labelling *targetLabelling);
    virtual ~LabelsVolumeGenerator();

    // adds all labels to the local target/buffer according to labelling info
    bool addLabelVolume(Volume *v, Labelling *lab);

    // commits the local buffer to the target volume
    void commitToTarget();

    // performs post-processing on a label volume so that it can be used for
    // volume ray-casting (includes dilation and alpha mask generation)
    void postProcessTarget();

    // optionally set a progress reporter to indicate progress
    void setProgressReporter(Reporter *reporter) { m_progressReporter = reporter; }
};

#endif // LABELSVOLUMEGENERATOR_H
