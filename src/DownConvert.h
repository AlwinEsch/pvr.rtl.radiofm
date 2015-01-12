#pragma once
/*
 *      Copyright (C) 2013, Joris van Rantwijk.
 *      Copyright (C) 2015, Alwin Esch (Team KODI)
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
 *  This part of code is taken from SoftFM created by Joris van Rantwijk
 *  (Copyright 2013) and handled by GNU GPL
 */

#include "Definations.h"

/*!
 *  Downsampler with low-pass FIR filter for real-valued signals.
 *
 *  Step 1: Low-pass filter based on Lanczos FIR filter
 *  Step 2: (optional) Decimation by an arbitrary factor (integer or float)
 */
class cDownsampleFilter
{
public:
  /*!
   * Construct low-pass filter with optional downsampling.
   *
   * filter_order :: FIR filter order
   * cutoff       :: Cutoff frequency relative to the full input sample rate
   *                 (valid range 0.0 .. 0.5)
   * downsample   :: Decimation factor (>= 1) or 1 to disable
   * integer_factor :: Enables a faster and more precise algorithm that
   *                   only works for integer downsample factors.
   *
   * The output sample rate is (input_sample_rate / downsample)
   */
  cDownsampleFilter(unsigned int filter_order, double cutoff,
                    double downsample=1, bool integer_factor=true);
  virtual ~cDownsampleFilter();

  void Reset();

  /*! Process samples. */
  unsigned int Process(const RealType *samples_in, RealType *samples_out, unsigned int length);
  unsigned int Process(const ComplexType *samples_in, ComplexType *samples_out, unsigned int length);

private:
  double          m_downsample;
  unsigned int    m_downsample_int;
  unsigned int    m_pos_int;
  RealType        m_pos_frac;
  RealType       *m_coeff;

  unsigned int    m_stateOrderSize;
  RealType       *m_stateReal;
  ComplexType    *m_stateComplex;
};
