/*
 *  Copyright (C) 2010-2013, Moe Wheatley
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

/*!
 * This class implements a FIR  filter using a dual flat coefficient
 * array to eliminate testing for buffer wrap around.
 *
 * Filter coefficients can be from a fixed table or this class will create
 * a lowpass or highpass filter from frequency and attenuation specifications
 * using a Kaiser-Bessel windowed sinc algorithm
 *
 * History of Moe Wheatley:
 *  2011-01-29  Initial creation MSW
 *  2011-03-27  Initial release
 *  2011-08-07  Modified FIR filter initialization to force fixed size
 *  2013-07-28  Added single/double precision math macros
 */

#include "FirFilter.h"

#define MAX_HALF_BAND_BUFSIZE 8192

cFirFilter::cFirFilter()
{
  m_NumTaps = 1;
  m_State = 0;
}

/*!
 * private helper function to Compute Modified Bessel function I0(x)
 * using a series approximation.
 * I0(x) = 1.0 + { sum from k=1 to infinity ---->  [(x/2)^k / k!]^2 }
 */
RealType cFirFilter::Izero(RealType x)
{
  RealType x2 = x / 2.0f;
  RealType sum = 1.0f;
  RealType ds = 1.0f;
  RealType di = 1.0f;
  RealType errorlimit = 1e-9;
  RealType tmp;

  do
  {
    tmp = x2 / di;
    tmp *= tmp;
    ds *= tmp;
    sum += ds;
    di += 1.0;
  } while (ds >= errorlimit * sum);

  return (sum);
}

/*!
 * Create a FIR Low Pass filter with scaled amplitude 'Scale'
 * NumTaps if non-zero, forces filter design to be this number of taps
 * Scale is linear amplitude scale factor.
 * Astop = Stopband Atenuation in dB (ie 40dB is 40dB stopband attenuation)
 * Fpass = Lowpass passband frequency in Hz
 * Fstop = Lowpass stopband frequency in Hz
 * Fsamprate = Sample Rate in Hz
 *
 *           -------------
 *                        |
 *                         |
 *                          |
 *                           |
 *    Astop                   ---------------
 *                    Fpass   Fstop
 *
 */
int cFirFilter::InitLPFilter(unsigned int NumTaps,
                             RealType Scale,
                             RealType Astop,
                             RealType Fpass,
                             RealType Fstop,
                             RealType Fsamprate)
{
  RealType Beta;
  m_SampleRate = Fsamprate;
  //! create normalized frequency parameters
  RealType normFpass = Fpass / Fsamprate;
  RealType normFstop = Fstop / Fsamprate;
  RealType normFcut = (normFstop + normFpass) / 2.0f; //!< low pass filter 6dB cutoff

  //! calculate Kaiser-Bessel window shape factor, Beta, from stopband attenuation
  if (Astop < 20.96f)
    Beta = 0;
  else if (Astop >= 50.0f)
    Beta = .1102 * (Astop - 8.71f);
  else
    Beta = .5842 * MPOW((Astop - 20.96f), 0.4) + .07886f * (Astop - 20.96f);

  //! Now Estimate number of filter taps required based on filter specs
  m_NumTaps = (Astop - 8.0f) / (2.285f * K_2PI * (normFstop - normFpass)) + 1;

  //! clamp range of filter taps
  if (m_NumTaps > MAX_NUMCOEF)
    m_NumTaps = MAX_NUMCOEF;
  if (m_NumTaps < 3)
    m_NumTaps = 3;

  if (NumTaps) //!< if need to force to to a number of taps
    m_NumTaps = NumTaps;

  RealType fCenter = .5 * (RealType)(m_NumTaps - 1);
  RealType izb = Izero(Beta); //!< precalculate denominator since is same for all points
  for (unsigned int n = 0; n < m_NumTaps; ++n)
  {
    RealType x = (RealType)n - fCenter;
    RealType c;
    //! create ideal Sinc() LP filter with normFcut
    if ((RealType)n == fCenter) //!< deal with odd size filter singularity where sin(0)/0==1
      c = 2.0 * normFcut;
    else
      c = MSIN(K_2PI * x * normFcut) / (K_PI * x);
    //! calculate Kaiser window and multiply to get coefficient
    x = ((RealType)n - ((RealType)m_NumTaps - 1.0f) / 2.0f) / (((RealType)m_NumTaps - 1.0f) / 2.0f);
    m_Coef[n] = Scale * c * Izero(Beta * MSQRT(1 - (x * x))) / izb;
  }

  //! make a 2x length array for FIR flat calculation efficiency
  for (unsigned int n = 0; n < m_NumTaps; ++n)
    m_Coef[n + m_NumTaps] = m_Coef[n];

  //! copy into complex coef buffers
  for (unsigned int n = 0; n < m_NumTaps * 2; ++n)
  {
    m_ICoef[n] = m_Coef[n];
    m_QCoef[n] = m_Coef[n];
  }

  //! Initialize the FIR buffers and state
  for (unsigned int i = 0; i < m_NumTaps; i++)
  {
    m_rZBuf[i] = 0.0;
    m_cZBuf[i] = 0.0;
  }
  m_State = 0;

  return m_NumTaps;
}

/*!
 * function to convert LP filter coefficients into complex hilbert bandpass
 * filter coefficients.
 * Hbpreal(n)= 2*Hlp(n)*cos( 2PI*FreqOffset*(n-(N-1)/2)/samplerate );
 * Hbpimaj(n)= 2*Hlp(n)*sin( 2PI*FreqOffset*(n-(N-1)/2)/samplerate );
 */
void cFirFilter::GenerateHBFilter(RealType FreqOffset)
{
  for (unsigned int n = 0; n < m_NumTaps; ++n)
  {
    //! apply complex frequency shift transform to low pass filter coefficients
    RealType psin;
    RealType pcos;
    sincos_HP((K_2PI * FreqOffset / m_SampleRate) *
                  ((RealType)n - ((RealType)(m_NumTaps - 1) / 2.0)),
              psin, pcos);
    m_ICoef[n] = 2.0f * m_Coef[n] * pcos;
    m_QCoef[n] = 2.0f * m_Coef[n] * psin;
  }

  //! make a 2x length array for FIR flat calculation efficiency
  for (unsigned int n = 0; n < m_NumTaps; ++n)
  {
    m_ICoef[n + m_NumTaps] = m_ICoef[n];
    m_QCoef[n + m_NumTaps] = m_QCoef[n];
  }
}

/*!
 * Create a FIR high Pass filter with scaled amplitude 'Scale'
 * NumTaps if non-zero, forces filter design to be this number of taps
 * Astop = Stopband Atenuation in dB (ie 40dB is 40dB stopband attenuation)
 * Fpass = Highpass passband frequency in Hz
 * Fstop = Highpass stopband frequency in Hz
 * Fsamprate = Sample Rate in Hz
 *
 *  Apass              -------------
 *                    /
 *                   /
 *                  /
 *                 /
 *  Astop ---------
 *            Fstop   Fpass
 */
int cFirFilter::InitHPFilter(unsigned int NumTaps,
                             RealType Scale,
                             RealType Astop,
                             RealType Fpass,
                             RealType Fstop,
                             RealType Fsamprate)
{
  RealType Beta;
  m_SampleRate = Fsamprate;
  //! create normalized frequency parameters
  RealType normFpass = Fpass / Fsamprate;
  RealType normFstop = Fstop / Fsamprate;
  RealType normFcut = (normFstop + normFpass) / 2.0; //high pass filter 6dB cutoff

  //! calculate Kaiser-Bessel window shape factor, Beta, from stopband attenuation
  if (Astop < 20.96f)
    Beta = 0;
  else if (Astop >= 50.0f)
    Beta = .1102f * (Astop - 8.71f);
  else
    Beta = .5842f * MPOW((Astop - 20.96f), 0.4f) + .07886f * (Astop - 20.96f);

  //! Now Estimate number of filter taps required based on filter specs
  m_NumTaps = (Astop - 8.0f) / (2.285f * K_2PI * (normFpass - normFstop)) + 1;

  //! clamp range of filter taps
  if (m_NumTaps > (MAX_NUMCOEF - 1))
    m_NumTaps = MAX_NUMCOEF - 1;
  if (m_NumTaps < 3)
    m_NumTaps = 3;

  m_NumTaps |= 1; //!< force to next odd number

  if (NumTaps) //!< if need to force to to a number of taps
    m_NumTaps = NumTaps;

  RealType izb = Izero(Beta); //!< precalculate denominator since is same for all points
  RealType fCenter = .5f * (RealType)(m_NumTaps - 1);
  for (unsigned int n = 0; n < m_NumTaps; n++)
  {
    RealType x = (RealType)n - (RealType)(m_NumTaps - 1) / 2.0;
    RealType c;
    //! create ideal Sinc() HP filter with normFcut
    if ((RealType)n == fCenter) //!< deal with odd size filter singularity where sin(0)/0==1
      c = 1.0 - 2.0 * normFcut;
    else
      c = MSIN(K_PI * x) / (K_PI * x) - MSIN(K_2PI * x * normFcut) / (K_PI * x);

    //! calculate Kaiser window and multiply to get coefficient
    x = ((RealType)n - ((RealType)m_NumTaps - 1.0f) / 2.0f) / (((RealType)m_NumTaps - 1.0f) / 2.0f);
    m_Coef[n] = Scale * c * Izero(Beta * MSQRT(1 - (x * x))) / izb;
  }

  //! make a 2x length array for FIR flat calculation efficiency
  for (unsigned int n = 0; n < m_NumTaps; ++n)
    m_Coef[n + m_NumTaps] = m_Coef[n];

  //! copy into complex coef buffers
  for (unsigned int n = 0; n < m_NumTaps * 2; ++n)
  {
    m_ICoef[n] = m_Coef[n];
    m_QCoef[n] = m_Coef[n];
  }

  //! Initialize the FIR buffers and state
  for (unsigned int i = 0; i < m_NumTaps; ++i)
  {
    m_rZBuf[i] = 0.0;
    m_cZBuf[i] = 0.0;
  }
  m_State = 0;

  return m_NumTaps;
}

/*!
 * Initializes a pre-designed complex FIR filter with fixed coefficients
 * Iniitalize FIR variables and clear out buffers.
 */
void cFirFilter::InitConstFir(unsigned int NumTaps,
                              const RealType* pICoef,
                              const RealType* pQCoef,
                              RealType Fsamprate)
{
  m_SampleRate = Fsamprate;
  if (NumTaps > MAX_NUMCOEF)
    m_NumTaps = MAX_NUMCOEF;
  else
    m_NumTaps = NumTaps;
  for (unsigned int i = 0; i < m_NumTaps; i++)
  {
    m_ICoef[i] = pICoef[i];
    m_ICoef[m_NumTaps + i] = pICoef[i]; //!< create duplicate for calculation efficiency
    m_QCoef[i] = pQCoef[i];
    m_QCoef[m_NumTaps + i] = pQCoef[i]; //!< create duplicate for calculation efficiency
  }
  for (unsigned int i = 0; i < m_NumTaps; i++)
  { //! zero input buffers
    m_rZBuf[i] = 0.0;
    m_cZBuf[i] = 0.0;
  }
  m_State = 0; //!< zero filter state variable
}

/*!
 * Initializes a pre-designed FIR filter with fixed coefficients
 * Iniitalize FIR variables and clear out buffers.
 */
void cFirFilter::InitConstFir(unsigned int NumTaps, const RealType* pCoef, RealType Fsamprate)
{
  m_SampleRate = Fsamprate;
  if (NumTaps > MAX_NUMCOEF)
    m_NumTaps = MAX_NUMCOEF;
  else
    m_NumTaps = NumTaps;
  for (unsigned int i = 0; i < m_NumTaps; ++i)
  {
    m_Coef[i] = pCoef[i];
    m_Coef[m_NumTaps + i] = pCoef[i]; //!< create duplicate for calculation efficiency
  }
  for (unsigned int i = 0; i < m_NumTaps; ++i)
  { //! zero input buffers
    m_rZBuf[i] = 0.0;
    m_cZBuf[i] = 0.0;
  }
  m_State = 0; //!< zero filter state variable
}

/*!
 * Process InLength InBuf[] samples and place in OutBuf[]
 * Note the Coefficient array is twice the length and has a duplicated set
 * in order to eliminate testing for buffer wrap in the inner loop
 * ex: if 3 tap FIR with coefficients{21,-43,15} is made into a array of 6 entries
 *  {21, -43, 15, 21, -43, 15 }
 * Complex single buffer version
 */
void cFirFilter::Process(ComplexType* buffer, unsigned int length)
{
  ComplexType acc;
  RealType* HIptr;
  RealType* HQptr;

  for (unsigned int i = 0; i < length; ++i)
  {
    m_cZBuf[m_State] = buffer[i];
    HIptr = m_ICoef + m_NumTaps - m_State;
    HQptr = m_QCoef + m_NumTaps - m_State;

    acc = ComplexType((*HIptr++ * m_cZBuf[0].real()), (*HQptr++ * m_cZBuf[0].imag()));
    for (unsigned int j = 1; j < m_NumTaps; j++)
      acc += ComplexType((*HIptr++ * m_cZBuf[j].real()), (*HQptr++ * m_cZBuf[j].imag()));

    if (--m_State < 0)
      m_State += m_NumTaps;
    buffer[i] = acc;
  }
}

/*!
 * Process InLength InBuf[] samples and place in OutBuf[]
 * Note the Coefficient array is twice the length and has a duplicated set
 * in order to eliminate testing for buffer wrap in the inner loop
 * ex: if 3 tap FIR with coefficients{21,-43,15} is made into a array of 6 entries
 *  {21, -43, 15, 21, -43, 15 }
 * Real single buffer version
 */
void cFirFilter::Process(RealType* buffer, unsigned int length)
{
  RealType acc;
  const RealType* Hptr;

  for (unsigned int i = 0; i < length; ++i)
  {
    m_rZBuf[m_State] = buffer[i];
    Hptr = &m_Coef[m_NumTaps - m_State];
    acc = Hptr[0] * m_rZBuf[0]; //do the 1st MAC
    for (unsigned int j = 1; j < m_NumTaps; ++j)
      acc += Hptr[j] * m_rZBuf[j]; //do the remaining MACs

    if (--m_State < 0)
      m_State += m_NumTaps;
    buffer[i] = acc;
  }
}

/*!
 * Process InLength InBuf[] samples and place in OutBuf[]
 * Note the Coefficient array is twice the length and has a duplicated set
 * in order to eliminate testing for buffer wrap in the inner loop
 * ex: if 3 tap FIR with coefficients{21,-43,15} is made into a array of 6 entries
 *  {21, -43, 15, 21, -43, 15 }
 * Real single buffer version with two streams
 */
void cFirFilter::ProcessTwo(RealType* bufferA, RealType* bufferB, unsigned int length)
{
  RealType valueA;
  RealType valueB;
  RealType* HIptr;
  RealType* HQptr;

  for (unsigned int i = 0; i < length; ++i)
  {
    m_cZBuf[m_State] = ComplexType(bufferA[i], bufferB[i]);
    HIptr = m_ICoef + m_NumTaps - m_State;
    HQptr = m_QCoef + m_NumTaps - m_State;

    valueA = (*HIptr++ * m_cZBuf[0].real()); //do the first MAC
    valueB = (*HQptr++ * m_cZBuf[0].imag());
    for (unsigned int j = 1; j < m_NumTaps; ++j)
    {
      valueA += (*HIptr++ * m_cZBuf[j].real()); //do the remaining MACs
      valueB += (*HQptr++ * m_cZBuf[j].imag());
    }

    if (--m_State < 0)
      m_State += m_NumTaps;
    bufferA[i] = valueA;
    bufferB[i] = valueB;
  }
}

/*!
 * Process InLength InBuf[] samples and place in OutBuf[]
 * Note the Coefficient array is twice the length and has a duplicated set
 * in order to eliminate testing for buffer wrap in the inner loop
 * ex: if 3 tap FIR with coefficients{21,-43,15} is made into a array of 6 entries
 *  {21, -43, 15, 21, -43, 15 }
 * Real double in / out buffer version
 */
void cFirFilter::Process(ComplexType* InBuf, ComplexType* OutBuf, unsigned int length)
{
  ComplexType acc;
  RealType* HIptr;
  RealType* HQptr;

  for (unsigned int i = 0; i < length; ++i)
  {
    m_cZBuf[m_State] = InBuf[i];
    HIptr = m_ICoef + m_NumTaps - m_State;
    HQptr = m_QCoef + m_NumTaps - m_State;

    acc = ComplexType((*HIptr++ * m_cZBuf[0].real()), (*HQptr++ * m_cZBuf[0].imag()));
    for (unsigned int j = 1; j < m_NumTaps; ++j)
      acc += ComplexType((*HIptr++ * m_cZBuf[j].real()), (*HQptr++ * m_cZBuf[j].imag()));

    if (--m_State < 0)
      m_State += m_NumTaps;
    OutBuf[i] = acc;
  }
}

/*!
 * Process InLength InBuf[] samples and place in OutBuf[]
 * Note the Coefficient array is twice the length and has a duplicated set
 * in order to eliminate testing for buffer wrap in the inner loop
 * ex: if 3 tap FIR with coefficients{21,-43,15} is made into a array of 6 entries
 *  {21, -43, 15, 21, -43, 15 }
 * Complex double in / out buffer version (for Hilbert filter pair)
 */
void cFirFilter::Process(RealType* InBuf, ComplexType* OutBuf, unsigned int length)
{
  ComplexType acc;
  RealType* HIptr;
  RealType* HQptr;

  for (unsigned int i = 0; i < length; ++i)
  {
    m_cZBuf[m_State] = ComplexType(InBuf[i], InBuf[i]);
    HIptr = m_ICoef + m_NumTaps - m_State;
    HQptr = m_QCoef + m_NumTaps - m_State;

    acc = ComplexType(*HIptr++ * m_cZBuf[0].real(), *HQptr++ * m_cZBuf[0].imag());
    for (unsigned int j = 1; j < m_NumTaps; ++j)
      acc = ComplexType(*HIptr++ * m_cZBuf[j].real(),
                        *HQptr++ * m_cZBuf[j].imag()); //do the remaining MACs

    if (--m_State < 0)
      m_State += m_NumTaps;

    OutBuf[i] = acc;
  }
}
