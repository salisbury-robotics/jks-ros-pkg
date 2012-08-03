#include "ros/console.h"
#include "ros/assert.h"
#include "hydra.h"
#include <usb.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <cstring>
//#include <LinearMath/tf::Matrix3x3.h>
#include <tf/tf.h>
#include <errno.h>
using namespace hydra;

// loosely adapted from the following 
// https://github.com/rpavlik/vrpn/blob/razer-hydra/vrpn_Tracker_RazerHydra.C

// and with reference to
// http://lxr.free-electrons.com/source/samples/hidraw/hid-example.c

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 *
 * If you need this, please have your distro update the kernel headers.
 */
#ifndef HIDIOCSFEATURE
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

// eventually crawl hidraw file system using this:
// http://www.signal11.us/oss/udev/

Hydra::Hydra()
: hidraw_fd(0)
{
  ros::Time::init();  
}

Hydra::~Hydra()
{
  if (hidraw_fd >= 0)
  {
    ROS_INFO("releasing hydra");
    uint8_t buf[256];
    memset(buf, 0, sizeof(buf));
    buf[6] = 1;
    buf[8] = 4;
    buf[89] = 5;
    int res = ioctl(hidraw_fd, HIDIOCSFEATURE(91), buf);
    if (res < 0)
    {
      ROS_ERROR("unable to stop streaming");
      perror("HIDIOCSFEATURE");
    }
    else
      ROS_INFO("stopped streaming");
    close(hidraw_fd);
  }
}

bool Hydra::init(const char *device)
{
    int res, desc_size = 0;
    uint8_t buf[256];
    struct hidraw_report_descriptor rpt_desc;
    struct hidraw_devinfo info;

    hidraw_fd = open(device, O_RDWR | O_NONBLOCK);
    if (hidraw_fd < 0)
    {
        ROS_ERROR("couldn't open hidraw device");
        return false;
    }
    ROS_INFO("opened hydra");

    memset(&rpt_desc, 0x0, sizeof(rpt_desc));
    memset(&info, 0x0, sizeof(info));
    memset(buf, 0x0, sizeof(buf));

    /* Get Report Descriptor Size */

    ROS_DEBUG("Getting Report Descriptor Size");
    res = ioctl(hidraw_fd, HIDIOCGRDESCSIZE, &desc_size);
    if (res < 0)
        perror("HIDIOCGRDESCSIZE");
    else
        printf("Report Descriptor Size: %d\n", desc_size);

    /* Get Report Descriptor */
    ROS_DEBUG("Getting Report Descriptor");
    rpt_desc.size = desc_size;
    res = ioctl(hidraw_fd, HIDIOCGRDESC, &rpt_desc);
    if (res < 0) {
        perror("HIDIOCGRDESC");
    } else {
        printf("Report Descriptor:\n");
        for (size_t i = 0; i < rpt_desc.size; i++)
        printf("%hhx ", rpt_desc.value[i]);
        puts("\n");
    }

    /* Get Raw Name */
    ROS_DEBUG("Getting Raw Name");
    res = ioctl(hidraw_fd, HIDIOCGRAWNAME(256), buf);
    if (res < 0)
        perror("HIDIOCGRAWNAME");
    else
        printf("Raw Name: %s\n", buf);

    // set feature to start it streaming
    memset(buf, 0x0, sizeof(buf));
    buf[6] = 1;
    buf[8] = 4;
    buf[9] = 3;
    buf[89] = 6;
    int attempt = 0;
    for (; attempt < 50; attempt++)
    {
        res = ioctl(hidraw_fd, HIDIOCSFEATURE(91), buf);
        if (res < 0)
        {
            ROS_ERROR("unable to start streaming");
            perror("HIDIOCSFEATURE");
            usleep(50000);
        }
        else
        {
            ROS_INFO("started streaming");
            break;
        }
    }
    ROS_INFO("%d attempts", attempt);
    return attempt < 60;
}

bool Hydra::poll(uint32_t ms_to_wait)
{
  if (hidraw_fd < 0)
  {
    ROS_ERROR("couldn't poll");
    return false;
  }
  ros::Time t_start(ros::Time::now());
  //if (ms_to_wait < 4)
  //  ms_to_wait = 4;

  uint8_t buf[64];
  while (ros::Time::now() < t_start + ros::Duration(0.001 * ms_to_wait))
  {
    ssize_t nread = read(hidraw_fd, buf, sizeof(buf));
    //ROS_INFO("read %d bytes", (int)nread);
    if (nread > 0)
    {
      //ROS_INFO("read %d bytes", (int)nread);
      /*
      double x0 = *((int16_t *)(buf+8));
      double y0 = *((int16_t *)(buf+10));
      double z0 = *((int16_t *)(buf+12));
      double x1 = *((int16_t *)(buf+30));
      double y1 = *((int16_t *)(buf+32));
      double z1 = *((int16_t *)(buf+34));
      ROS_INFO("%.4f %.4f %.4f   %.4f %.4f %.4f", x0, y0, z0, x1, y1, z1);
      */
      raw_pos[0] = *((int16_t *)(buf+8));
      raw_pos[1] = *((int16_t *)(buf+10));
      raw_pos[2] = *((int16_t *)(buf+12));
      raw_quat[0] = *((int16_t *)(buf+14));
      raw_quat[1] = *((int16_t *)(buf+16));
      raw_quat[2] = *((int16_t *)(buf+18));
      raw_quat[3] = *((int16_t *)(buf+20));
      raw_buttons[0] = buf[22] & 0x7f;
      raw_analog[0] = *((int16_t *)(buf+23));
      raw_analog[1] = *((int16_t *)(buf+25));
      raw_analog[2] = buf[27];

      raw_pos[3] = *((int16_t *)(buf+30));
      raw_pos[4] = *((int16_t *)(buf+32));
      raw_pos[5] = *((int16_t *)(buf+34));
      raw_quat[4] = *((int16_t *)(buf+36));
      raw_quat[5] = *((int16_t *)(buf+38));
      raw_quat[6] = *((int16_t *)(buf+40));
      raw_quat[7] = *((int16_t *)(buf+42));
      raw_buttons[1] = buf[44] & 0x7f;
      raw_analog[3] = *((int16_t *)(buf+45));
      raw_analog[4] = *((int16_t *)(buf+47));
      raw_analog[5] = buf[49];

      // mangle the reported pose into the ROS frame conventions
//      pos[0].setX(-raw_pos[1] * 0.001);
//      pos[0].setY(-raw_pos[0] * 0.001);
//      pos[0].setZ(-raw_pos[2] * 0.001);

//      pos[1].setX(-raw_pos[4] * 0.001);
//      pos[1].setY(-raw_pos[3] * 0.001);
//      pos[1].setZ(-raw_pos[5] * 0.001);

      tf::Matrix3x3 ros_to_razer(  0,  -1,  0,
                                   -1, 0,  0,
                                   0,  0, -1);

      pos[0].setX(raw_pos[0] * 0.001);
      pos[0].setY(raw_pos[1] * 0.001);
      pos[0].setZ(raw_pos[2] * 0.001);

      pos[1].setX(raw_pos[3] * 0.001);
      pos[1].setY(raw_pos[4] * 0.001);
      pos[1].setZ(raw_pos[5] * 0.001);

      // mangle the quaternion into the ROS frame convention
      // this is absolutely terrible. i'm sure this is a much better way.
      for (int i = 0; i < 2; i++)
      {
        tf::Quaternion q(raw_quat[i*4+1] / 32768.0,
                         raw_quat[i*4+2] / 32768.0,
                         raw_quat[i*4+3] / 32768.0,
                         raw_quat[i*4+0] / 32768.0);
        tf::Matrix3x3 mat(q);
        mat = ros_to_razer*mat*tf::Matrix3x3(tf::createQuaternionFromRPY(0, 0, M_PI_2));
        mat.getRotation(q);
//        quat[i] = q*tf::createQuaternionFromRPY(0, M_PI, 0);

        quat[i] = q;
        pos[i] = ros_to_razer*pos[i];
      }

//      for (int i = 0; i < 2; i++)
//      {
//          tf::Matrix3x3 ros_to_razer(  0,  -1,  0,
//                                       -1, 0,  0,
//                                       0,  0, -1);
//          quat[i] = quat[i]*ros_to_razer;

//      }

      analog[0] = raw_analog[0] / 32768.0;
      analog[1] = raw_analog[1] / 32768.0;
      analog[2] = raw_analog[2] / 255.0;
      analog[3] = raw_analog[3] / 32768.0;
      analog[4] = raw_analog[4] / 32768.0;
      analog[5] = raw_analog[5] / 255.0;

      for (int i = 0; i < 2; i++)
      {
        buttons[i*7  ] = (raw_buttons[i] & 0x01) ? 1 : 0;
        buttons[i*7+1] = (raw_buttons[i] & 0x04) ? 1 : 0;
        buttons[i*7+2] = (raw_buttons[i] & 0x08) ? 1 : 0;
        buttons[i*7+3] = (raw_buttons[i] & 0x02) ? 1 : 0;
        buttons[i*7+4] = (raw_buttons[i] & 0x10) ? 1 : 0;
        buttons[i*7+5] = (raw_buttons[i] & 0x20) ? 1 : 0;
        buttons[i*7+6] = (raw_buttons[i] & 0x40) ? 1 : 0;
      }
        
      return true;
    }
    else
    {
        //ROS_ERROR( "Error reading: %s\n", strerror( errno ) );
        usleep(1000);
    }
  }
  //ROS_INFO("Ran out of time, returning!");
  return false;
#if 0
  {
    int r = usb_interrupt_read(hid_dev, hid_dev_ep_in,
                               (char *)buf, sizeof(buf), ms_to_wait);
    if (r > 0)
    {
      ROS_INFO("read %d bytes", r);
#if 0
      for (uint32_t i = 0; i < NUM_SENSORS; i++)
        raw[i] = ((uint16_t)buf[2*i] << 8) + buf[2*i+1]; // << 8); //*((uint16_t *)(buf+2*i));
#endif
      return true;
    }
    else if (r == -110)
    {
      //ROS_INFO("timeout");
      // timeout
    }
    else
    {
      ROS_ERROR("USB HID read error");
      return false;
      //continue;
    }
      /*
      return false;
    else
      */
  }
  return false;
#endif
}

