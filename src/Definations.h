#pragma once
/*
 *      Copyright (C) 2015-2018 Alwin Esch (Team KODI)
 *      http://kodi.tv
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with KODI; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301  USA
 *  http://www.gnu.org/copyleft/gpl.html
 *
 */

#include <complex>
#include "client.h"

#define DEVICE_RESTART_TRIES 5

#define OUTPUT_SAMPLERATE  48000

#undef USE_DOUBLE_PRECISION

typedef enum eFilterType
{
  ftLP,
  ftHP,
  ftBP,
  ftBR,
  ftConst
} eFilterType;

#ifdef USE_DOUBLE_PRECISION
  typedef std::complex<double> ComplexType;
  typedef double RealType;

 #define MSIN(x)      sin(x)
 #define MCOS(x)      cos(x)
 #define MPOW(x,y)    pow(x,y)
 #define MEXP(x)      exp(x)
 #define MFABS(x)     fabs(x)
 #define MLOG(x)      log(x)
 #define MLOG10(x)    log10(x)
 #define MSQRT(x)     sqrt(x)
 #define MATAN(x)     atan(x)
 #define MFMOD(x,y)   fmod(x,y)
 #define MATAN2(x,y)  atan2(x,y)
#else
  typedef std::complex<float> ComplexType;
  typedef float RealType;

 #define MSIN(x)      sinf(x)
 #define MCOS(x)      cosf(x)
 #define MPOW(x,y)    powf(x,y)
 #define MEXP(x)      expf(x)
 #define MFABS(x)     fabsf(x)
 #define MLOG(x)      logf(x)
 #define MLOG10(x)    log10f(x)
 #define MSQRT(x)     sqrtf(x)
 #define MATAN(x)     atanf(x)
 #define MFMOD(x,y)   fmodf(x,y)
 #define MATAN2(x,y)  atan2f(x,y)
#endif

#define K_2PI (2.0 * 3.14159265358979323846)
#define K_PI (3.14159265358979323846)
#define K_PI4 (K_PI/4.0)
#define K_PI2 (K_PI/2.0)
#define K_3PI4 (3.0*K_PI4)

#define  Hz(x)    (x)
#define  Khz(x)    (x * 1000)
#define  KHz(x)    (x * 1000)
#define  Mhz(x)    (Khz (x) * 1000)
#define  MHz(x)    (KHz (x) * 1000)

#include <math.h>

static inline void sincos_LP(RealType x, RealType& sin, RealType& cos)
{
  //always wrap input angle to -PI..PI
  if (x < -3.14159265)
    x += 6.28318531;
  else if (x >  3.14159265)
    x -= 6.28318531;

  //compute sine
  if (x < 0)
    sin = 1.27323954 * x + .405284735 * x * x;
  else
    sin = 1.27323954 * x - 0.405284735 * x * x;

  //compute cosine: sin(x + PI/2) = cos(x)
  x += 1.57079632;
  if (x >  3.14159265)
    x -= 6.28318531;

  if (x < 0)
    cos = 1.27323954 * x + 0.405284735 * x * x;
  else
    cos = 1.27323954 * x - 0.405284735 * x * x;
}

static inline void sincos_HP(RealType x, RealType& sin, RealType& cos)
{
  //always wrap input angle to -PI..PI
  if (x < -3.14159265)
    x += 6.28318531;
  else if (x >  3.14159265)
    x -= 6.28318531;

  //compute sine
  if (x < 0)
  {
    sin = 1.27323954 * x + .405284735 * x * x;
    if (sin < 0)
        sin = .225 * (sin *-sin - sin) + sin;
    else
        sin = .225 * (sin * sin - sin) + sin;
  }
  else
  {
    sin = 1.27323954 * x - 0.405284735 * x * x;
    if (sin < 0)
        sin = .225 * (sin *-sin - sin) + sin;
    else
        sin = .225 * (sin * sin - sin) + sin;
  }

  //compute cosine: sin(x + PI/2) = cos(x)
  x += 1.57079632;
  if (x >  3.14159265)
    x -= 6.28318531;

  if (x < 0)
  {
    cos = 1.27323954 * x + 0.405284735 * x * x;
    if (cos < 0)
        cos = .225 * (cos *-cos - cos) + cos;
    else
        cos = .225 * (cos * cos - cos) + cos;
  }
  else
  {
    cos = 1.27323954 * x - 0.405284735 * x * x;
    if (cos < 0)
        cos = .225 * (cos *-cos - cos) + cos;
    else
        cos = .225 * (cos * cos - cos) + cos;
  }
}
