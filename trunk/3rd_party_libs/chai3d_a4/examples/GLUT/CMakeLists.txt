#-----------------------------------------------------------------------------
# CMakeLists file for building the CHAI3D examples.
#
# Created by Sonny Chan
#	March 2009, Stanford University
#-----------------------------------------------------------------------------

# Recurse into the directory for each example
ADD_SUBDIRECTORY(01-mydevice)
ADD_SUBDIRECTORY(02-multi-devices)
ADD_SUBDIRECTORY(03-analytics)
ADD_SUBDIRECTORY(04-shapes)
ADD_SUBDIRECTORY(05-fonts)
ADD_SUBDIRECTORY(10-oring)
ADD_SUBDIRECTORY(11-effects)
ADD_SUBDIRECTORY(12-polygons)
ADD_SUBDIRECTORY(13-primitives)
ADD_SUBDIRECTORY(14-textures)
ADD_SUBDIRECTORY(15-paint)
ADD_SUBDIRECTORY(16-friction)
ADD_SUBDIRECTORY(17-shading)
ADD_SUBDIRECTORY(20-map)
ADD_SUBDIRECTORY(21-object)
ADD_SUBDIRECTORY(22-chrome)
ADD_SUBDIRECTORY(23-tooth)
IF(MSVC)
ADD_SUBDIRECTORY(24-turntable)
ENDIF(MSVC)
ADD_SUBDIRECTORY(25-cubic)

#ADD_SUBDIRECTORY(26-spinning_book)
#ADD_SUBDIRECTORY(27-projectile)
#ADD_SUBDIRECTORY(28-disk)
