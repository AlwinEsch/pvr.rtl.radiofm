/*
 *  Copyright (C) 2005-2020 Team Kodi
 *  https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#pragma once

#include <complex>
#include <string>
#include <vector>

typedef std::complex<float> IQSample;
typedef std::vector<IQSample> IQSampleVector;

typedef double Sample;
typedef std::vector<Sample> SampleVector;

/** Compute mean and RMS over a sample vector. */
inline void samples_mean_rms(const SampleVector& samples, double& mean, double& rms)
{
  Sample vsum = 0;
  Sample vsumsq = 0;

  unsigned int n = samples.size();
  for (unsigned int i = 0; i < n; i++)
  {
    Sample v = samples[i];
    vsum += v;
    vsumsq += v * v;
  }

  mean = vsum / n;
  rms = sqrt(vsumsq / n);
}

class StringUtils
{
public:
  static std::string Format(const char* fmt, ...);
  static std::string FormatV(const char* fmt, va_list args);
  static std::string& Trim(std::string& str);
  static std::string& TrimLeft(std::string& str);
  static std::string& TrimRight(std::string& str);
};
