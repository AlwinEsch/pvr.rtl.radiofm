/*
 *      Copyright (C) 2013, Joris van Rantwijk.
 *      Copyright (C) 2015-2018, Alwin Esch (Team KODI)
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

#include "DownConvert.h"

/*!
 * Prepare Lanczos FIR filter coefficients.
 */
static RealType *MakeLanczosCoeff(unsigned int filter_order, double cutoff)
{
  RealType *coeff = (RealType*)calloc(filter_order+3, sizeof(RealType));
  /*!
   * Prepare Lanczos FIR filter.
   *   t[i]     =  (i - order/2)
   *   coeff[i] =  Sinc(2 * cutoff * t[i]) * Sinc(t[i] / (order/2 + 1))
   *   coeff    /= sum(coeff)
   */

  double ysum = 0.0;

  //! Calculate filter kernel.
  for (int i = 1; i <= (int)filter_order+1; i++)
  {
    int t2 = 2 * i - filter_order;

    double y;
    if (t2 == 0)
    {
      y = 1.0;
    }
    else
    {
      double x1 = cutoff * t2;
      double x2 = t2 / double(filter_order + 2);
      y = ( MSIN(K_PI * x1) / K_PI / x1 ) *
          ( MSIN(K_PI * x2) / K_PI / x2 );
    }

    coeff[i] = y;
    ysum += y;
  }

  //! Apply correction factor to ensure unit gain at DC.
  for (unsigned i = 1; i <= filter_order+1; i++)
    coeff[i] /= ysum;

  return coeff;
}

/*! ****************  class cDownsampleFilter  **************** */

/*!
 * Construct low-pass filter with optional downsampling.
 */
cDownsampleFilter::cDownsampleFilter(unsigned int filter_order, double cutoff,
                                     double downsample, bool integer_factor)
  : m_downsample(downsample)
  , m_downsample_int(integer_factor ? lrint(downsample) : 0)
  , m_pos_int(0)
  , m_pos_frac(0)
  , m_stateOrderSize(filter_order)
{
  /*!
   * Force the first coefficient to zero and append an extra zero at the
   * end of the array. This ensures we can always obtain (filter_order+1)
   * coefficients by linear interpolation between adjacent array elements.
   */
  m_coeff        = MakeLanczosCoeff(filter_order - 1, cutoff);
  m_stateComplex = (ComplexType*)calloc(m_stateOrderSize, sizeof(ComplexType));
  m_stateReal    = (RealType*)calloc(m_stateOrderSize, sizeof(RealType));
}

cDownsampleFilter::~cDownsampleFilter()
{
  free(m_coeff);
  free(m_stateComplex);
  free(m_stateReal);
}

void cDownsampleFilter::Reset()
{
  m_pos_int = 0;
  m_pos_frac = 0;
  memset(m_stateReal, 0, m_stateOrderSize*sizeof(RealType));
  memset(m_stateComplex, 0, m_stateOrderSize*sizeof(ComplexType));
}

unsigned int cDownsampleFilter::Process(const ComplexType *samples_in, ComplexType *samples_out, unsigned int length)
{
  unsigned int order = m_stateOrderSize;
  unsigned int n = length;
  unsigned int i = 0;

  //! Integer downsample factor, no linear interpolation.
  //! This is relatively simple.
  unsigned int p = m_pos_int;
  unsigned int pstep = m_downsample_int;

  //! The first few samples need data from m_state.
  for (; p < n && p < order; p += pstep, i++)
  {
    ComplexType y = 0;
    for (unsigned int j = 1; j <= p; j++)
      y += samples_in[p-j] * m_coeff[j];
    for (unsigned int j = p + 1; j <= order; j++)
      y += m_stateComplex[order+p-j] * m_coeff[j];
    samples_out[i] = y;
  }

  //! Remaining samples only need data from samples_in.
  for (; p < n; p += pstep, i++)
  {
    ComplexType y = 0;
    for (unsigned int j = 1; j <= order; j++)
      y += samples_in[p-j] * m_coeff[j];
    samples_out[i] = y;
  }

  //! Update index of start position in text sample block.
  m_pos_int = p - n;

  //! Update m_state.
  unsigned int j;
  if (length < order)
  {
    j = 0;
    for (unsigned int i = length; i < order; i++)
      m_stateComplex[j++] = m_stateComplex[i];

    j = order-length;
    for (unsigned int i = 0; i < length; i++)
      m_stateComplex[j++] = samples_in[i];
  }
  else
  {
    j = 0;
    for (unsigned int i = length-order; i < length; i++)
      m_stateComplex[j++] = samples_in[i];
  }

  return i;
}

unsigned int cDownsampleFilter::Process(const RealType *samples_in, RealType *samples_out, unsigned int length)
{
  unsigned int order = m_stateOrderSize;
  unsigned int n = length;
  unsigned int i = 0;

  if (m_downsample_int != 0)
  {
    //! Integer downsample factor, no linear interpolation.
    //! This is relatively simple.

    unsigned int p = m_pos_int;
    unsigned int pstep = m_downsample_int;

    //! The first few samples need data from m_state.
    for (; p < n && p < order; p += pstep, i++)
    {
      RealType y = 0;
      for (unsigned int j = 1; j <= p; j++)
        y += samples_in[p-j] * m_coeff[j];
      for (unsigned int j = p + 1; j <= order; j++)
        y += m_stateReal[order+p-j] * m_coeff[j];
      samples_out[i] = y;
    }

    //! Remaining samples only need data from samples_in.
    for (; p < n; p += pstep, i++)
    {
      RealType y = 0;
      for (unsigned int j = 1; j <= order; j++)
        y += samples_in[p-j] * m_coeff[j];
      samples_out[i] = y;
    }

    //! Update index of start position in text sample block.
    m_pos_int = p - n;
  }
  else
  {
    /*!
     * Fractional downsample factor via linear interpolation of
     * the FIR coefficient table. This is a bitch.
     */

    //! Estimate number of output samples we can produce in this run.
    RealType p = m_pos_frac;
    RealType pstep = m_downsample;

    //! Produce output samples.
    RealType pf = p;
    unsigned int pi = int(pf);
    while (pi < n)
    {
      RealType k1 = pf - pi;
      RealType k0 = 1 - k1;

      RealType y = 0;
      for (unsigned int j = 0; j <= order; j++)
      {
        RealType k = m_coeff[j] * k0 + m_coeff[j+1] * k1;
        RealType s = (j <= pi) ? samples_in[pi-j] : m_stateReal[order+pi-j];
        y += k * s;
      }
      samples_out[i] = y;

      i++;
      pf = p + i * pstep;
      pi = int(pf);
    }

    //! Update fractional index of start position in text sample block.
    //! Limit to 0 to avoid catastrophic results of rounding errors.
    m_pos_frac = pf - n;
    if (m_pos_frac < 0)
      m_pos_frac = 0;
  }


  //! Update m_state.
  unsigned int j;
  if (length < order)
  {
    j = 0;
    for (unsigned int i = length; i < order; i++)
      m_stateReal[j++] = m_stateReal[i];

    j = order-length;
    for (unsigned int i = 0; i < length; i++)
      m_stateReal[j++] = samples_in[i];
  }
  else
  {
    j = 0;
    for (unsigned int i = length-order; i < length; i++)
      m_stateReal[j++] = samples_in[i];
  }

  return i;
}
