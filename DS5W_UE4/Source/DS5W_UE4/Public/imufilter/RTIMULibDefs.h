////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//  The MPU-9250 and SPI driver code is based on code generously supplied by
//  staslock@gmail.com (www.clickdrive.io)

#ifndef _RTIMULIBDEFS_H
#define	_RTIMULIBDEFS_H

#include <cstdint>
#include <cmath>
#include <memory>

#include "RTMath.h"

#ifndef HAL_QUIET
#define HAL_INFO(m) { printf("%s", m); fflush(stdout); }
#define HAL_INFO1(m, x) { printf(m, x); fflush(stdout); }
#define HAL_INFO2(m, x, y) { printf(m, x, y); fflush(stdout); }
#define HAL_INFO3(m, x, y, z) { printf(m, x, y, z); fflush(stdout); }
#define HAL_INFO4(m, x, y, z, a) { printf(m, x, y, z, a); fflush(stdout); }
#define HAL_INFO5(m, x, y, z, a, b) { printf(m, x, y, z, a, b); fflush(stdout); }
#define HAL_ERROR(m)    fprintf(stderr, m);
#define HAL_ERROR1(m, x)    fprintf(stderr, m, x);
#define HAL_ERROR2(m, x, y)    fprintf(stderr, m, x, y);
#define HAL_ERROR3(m, x, y, z)    fprintf(stderr, m, x, y, z);
#define HAL_ERROR4(m, x, y, z, a)    fprintf(stderr, m, x, y, z, a);

#else

#define HAL_INFO(m)
#define HAL_INFO1(m, x)
#define HAL_INFO2(m, x, y)
#define HAL_INFO3(m, x, y, z)
#define HAL_INFO4(m, x, y, z, a)
#define HAL_INFO5(m, x, y, z, a, b)
#define HAL_ERROR(m)
#define HAL_ERROR1(m, x)
#define HAL_ERROR2(m, x, y)
#define HAL_ERROR3(m, x, y, z)
#define HAL_ERROR4(m, x, y, z, a)

#endif

//  these defines describe the various fusion filter options

#define RTFUSION_TYPE_NULL                  0                   // just a dummy to keep things happy if not needed
#define RTFUSION_TYPE_KALMANSTATE4          1                   // kalman state is the quaternion pose
#define RTFUSION_TYPE_RTQF                  2                   // RT quaternion fusion

#define RTFUSION_TYPE_COUNT                 3                   // number of fusion algorithm types

//  This is a convenience structure that can be used to pass IMU data around

typedef struct
{
    uint64_t timestamp;
    RTFLOAT delta_time;
    bool fusionPoseValid;
    RTVector3 fusionPose;
    bool fusionQPoseValid;
    RTQuaternion fusionQPose;
    bool gyroValid;
    RTVector3 gyro;
    bool accelValid;
    RTVector3 accel;
    bool compassValid;
    RTVector3 compass;
} RTIMU_DATA;

#endif // _RTIMULIBDEFS_H
