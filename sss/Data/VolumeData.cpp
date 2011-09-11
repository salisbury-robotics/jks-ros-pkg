#include "VolumeData.h"
#include <limits>

using namespace std;

// --------------------------------------------------------------------------
// VolumeData class

void VolumeData::createMask()
{
    // copy header from volume
    m_mask = m_volume;
    m_mask.name += " (mask)";
    m_mask.path = "";
    m_mask.format = Volume::pfUInt8;
    m_mask.modified = false;

    // allocate and fill with full intensity
    size_t bytes = m_volume.dimensions[0]
                 * m_volume.dimensions[1]
                 * m_volume.dimensions[2];
    m_maskData = new unsigned char [bytes];
    memset(m_maskData, 0xff, bytes);
    m_mask.data = m_maskData;
}

// --------------------------------------------------------------------------
// VolumeTestData class

VolumeTestData::VolumeTestData(int size)
    : m_size(size)
{
    m_data = new PixelType [size*size*size];

    // create an interesting test volume here...
    PixelType *p = m_data;
    float r = 5.f;
    float d = 2.f * r / size;
    float s =  -r + 0.5*d;

    for (float z = s; z < r; z += d)
        for (float y = s; y < r; y += d)
            for (float x = s; x < r; x += d)
            {
                float t = 1.61803f;
                float f = cos(x + t*y) + cos(x - t*y) +
                          cos(y + t*z) + cos(y - t*z) +
                          cos(z + t*x) + cos(z - t*x);
                float g = min( max( 0.f, r - sqrt(x*x + y*y + z*z) ), 1.f );
                float h = g * (f+6.f) / 12.f;
                *p++ = static_cast<PixelType>( h * numeric_limits<PixelType>::max() );
            }

//  // this creates a simple cube data set
//    p = m_data;
//      for (float z = s; z < r; z += d)
//          for (float y = s; y < r; y += d)
//              for (float x = s; x < r; x += d)
//              {
//                  float h = 1.f - max( max( fabsf(x), fabsf(y) ), fabsf(z) ) / r;
//                  *p++ = static_cast<PixelType>( h * numeric_limits<PixelType>::max() );
//              }

    // populate the volume "header" object
    Volume &v = m_volume;
    v.name          = "Test Volume";
    v.dimensions    = cml::vector3i(m_size, m_size, m_size);
    v.spacing       = cml::vector3f(1.f, 1.f, 1.f);
    v.data          = m_data;

    // determine appropriate pixel format
    if (typeid(PixelType) == typeid(signed short))
        v.format = Volume::pfInt16;
    else if (typeid(PixelType) == typeid(unsigned char))
        v.format = Volume::pfUInt8;
    else if (typeid(PixelType) == typeid(unsigned short))
        v.format = Volume::pfUInt16;

    v.computeHistogram();
}

// --------------------------------------------------------------------------
// VolumeTestData class

VolumePointData::VolumePointData(int size)
    : m_size(size)
{
    m_data = new PixelType [size*size*size];

    // create an interesting test volume here...
    PixelType *p = m_data;
    float r = 5.f;
    float d = 2.f * r / size;
    float s =  -r + 0.5*d;

    std::vector< cml::vector3f > cloud;
    for(int i = -4; i <= 4; i++)
    {
      for(int j = -4; j <= 4; j++)
      {
        cloud.push_back(cml::vector3f(i, j, 0 ));
      }

    }

      p = m_data;
        for (float z = s; z < r; z += d)
            for (float y = s; y < r; y += d)
                for (float x = s; x < r; x += d)
                {
                    float R = 1.4;
                    float h = 0;
                    for( int i = 0; i < cloud.size(); i++)
                    {
                      cml::vector3f pt2pos = cml::vector3f(x,y,z) - cloud[i];
                      float distance = pt2pos.length();
                      if (distance > R)
                        continue;
                      float r_scaled = distance/R;
                      h += pow((1-r_scaled),4)*(4*r_scaled + 1);
                    }

                    //float h = 1.f - max( max( fabsf(x), fabsf(y) ), fabsf(z) ) / r;
                    *p++ = static_cast<PixelType>( h * numeric_limits<PixelType>::max() / 2 );
                }

    // populate the volume "header" object
    Volume &v = m_volume;
    v.name          = "Test Volume";
    v.dimensions    = cml::vector3i(m_size, m_size, m_size);
    v.spacing       = cml::vector3f(1.f, 1.f, 1.f);
    v.data          = m_data;

    // determine appropriate pixel format
    if (typeid(PixelType) == typeid(signed short))
        v.format = Volume::pfInt16;
    else if (typeid(PixelType) == typeid(unsigned char))
        v.format = Volume::pfUInt8;
    else if (typeid(PixelType) == typeid(unsigned short))
        v.format = Volume::pfUInt16;

    v.computeHistogram();
}

// --------------------------------------------------------------------------
// VolumeITK class

template <class PixelType>
VolumeITK<PixelType>::VolumeITK(typename ImageType::Pointer image, const string &name)
    : m_image(image)
{
    Volume &v = m_volume;
    v.name = name;

    // get dimensions of volume
    typename ImageType::SizeType size = m_image->GetLargestPossibleRegion().GetSize();
    for (int i = 0; i < 3; ++i) v.dimensions[i] = size[i];

    // get voxel spacing
    typename ImageType::SpacingType spacing = m_image->GetSpacing();
    for (int i = 0; i < 3; ++i) v.spacing[i] = spacing[i];

    // determine appropriate pixel format
    if (typeid(PixelType) == typeid(signed short))
        v.format = Volume::pfInt16;
    else if (typeid(PixelType) == typeid(unsigned char))
        v.format = Volume::pfUInt8;
    else if (typeid(PixelType) == typeid(unsigned short))
        v.format = Volume::pfUInt16;

    // set the volume's data pointer
    v.data = m_image->GetBufferPointer();

    v.computeHistogram();
}

// some explicit instantiations
template class VolumeITK<signed short>;
template class VolumeITK<unsigned char>;
template class VolumeITK<unsigned short>;

// --------------------------------------------------------------------------
