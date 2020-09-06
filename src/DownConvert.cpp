/*
 *  Copyright (C) 2013, Joris van Rantwijk.
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#include "DownConvert.h"

#include "filtercoef.h"

#include <string.h>

/*!
 * Prepare Lanczos FIR filter coefficients.
 */
static RealType* MakeLanczosCoeff(unsigned int filter_order, double cutoff)
{
  RealType* coeff = (RealType*)calloc(filter_order + 3, sizeof(RealType));
  /*!
   * Prepare Lanczos FIR filter.
   *   t[i]     =  (i - order/2)
   *   coeff[i] =  Sinc(2 * cutoff * t[i]) * Sinc(t[i] / (order/2 + 1))
   *   coeff    /= sum(coeff)
   */

  double ysum = 0.0;

  //! Calculate filter kernel.
  for (int i = 1; i <= (int)filter_order + 1; i++)
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
      y = (MSIN(K_PI * x1) / K_PI / x1) * (MSIN(K_PI * x2) / K_PI / x2);
    }

    coeff[i] = y;
    ysum += y;
  }

  //! Apply correction factor to ensure unit gain at DC.
  for (unsigned i = 1; i <= filter_order + 1; i++)
    coeff[i] /= ysum;

  return coeff;
}

/*! ****************  class cDownsampleFilter  **************** */

/*!
 * Construct low-pass filter with optional downsampling.
 */
cDownsampleFilter::cDownsampleFilter(unsigned int filter_order,
                                     double cutoff,
                                     double downsample,
                                     bool integer_factor)
  : m_downsample(downsample),
    m_downsample_int(integer_factor ? lrint(downsample) : 0),
    m_pos_int(0),
    m_pos_frac(0),
    m_stateOrderSize(filter_order)
{
  /*!
   * Force the first coefficient to zero and append an extra zero at the
   * end of the array. This ensures we can always obtain (filter_order+1)
   * coefficients by linear interpolation between adjacent array elements.
   */
  m_coeff = MakeLanczosCoeff(filter_order - 1, cutoff);
  m_stateComplex = (ComplexType*)calloc(m_stateOrderSize, sizeof(ComplexType));
  m_stateReal = (RealType*)calloc(m_stateOrderSize, sizeof(RealType));
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
  memset(m_stateReal, 0, m_stateOrderSize * sizeof(RealType));
  memset(m_stateComplex, 0, m_stateOrderSize * sizeof(ComplexType));
}

unsigned int cDownsampleFilter::Process(const ComplexType* samples_in,
                                        ComplexType* samples_out,
                                        unsigned int length)
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
      y += samples_in[p - j] * m_coeff[j];
    for (unsigned int j = p + 1; j <= order; j++)
      y += m_stateComplex[order + p - j] * m_coeff[j];
    samples_out[i] = y;
  }

  //! Remaining samples only need data from samples_in.
  for (; p < n; p += pstep, i++)
  {
    ComplexType y = 0;
    for (unsigned int j = 1; j <= order; j++)
      y += samples_in[p - j] * m_coeff[j];
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

    j = order - length;
    for (unsigned int i = 0; i < length; i++)
      m_stateComplex[j++] = samples_in[i];
  }
  else
  {
    j = 0;
    for (unsigned int i = length - order; i < length; i++)
      m_stateComplex[j++] = samples_in[i];
  }

  return i;
}

unsigned int cDownsampleFilter::Process(const RealType* samples_in,
                                        RealType* samples_out,
                                        unsigned int length)
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
        y += samples_in[p - j] * m_coeff[j];
      for (unsigned int j = p + 1; j <= order; j++)
        y += m_stateReal[order + p - j] * m_coeff[j];
      samples_out[i] = y;
    }

    //! Remaining samples only need data from samples_in.
    for (; p < n; p += pstep, i++)
    {
      RealType y = 0;
      for (unsigned int j = 1; j <= order; j++)
        y += samples_in[p - j] * m_coeff[j];
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
        RealType k = m_coeff[j] * k0 + m_coeff[j + 1] * k1;
        RealType s = (j <= pi) ? samples_in[pi - j] : m_stateReal[order + pi - j];
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

    j = order - length;
    for (unsigned int i = 0; i < length; i++)
      m_stateReal[j++] = samples_in[i];
  }
  else
  {
    j = 0;
    for (unsigned int i = length - order; i < length; i++)
      m_stateReal[j++] = samples_in[i];
  }

  return i;
}


//pick a method of calculating the NCO
#define NCO_LIB 0 //normal sin cos library (188nS)
#define NCO_OSC 1 //quadrature oscillator (25nS)
#define NCO_VCASM 0 //Visual C assembly call to floating point sin/cos instruction
#define NCO_GCCASM 0 //GCC assembly call to floating point sin/cos instruction (100nS)

#define MIN_OUTPUT_RATE (7900.0 * 2.0)

#define MAX_HALF_BAND_BUFSIZE 32768


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CRDSDownConvert::CRDSDownConvert()
{
  int i;
  m_NcoInc = 0.0;
  m_NcoTime = 0.0;
  m_NcoFreq = 0.0;
  m_CW_Offset = 0.0;
  m_InRate = 100000.0;
  m_MaxBW = 10000.0;
  for (i = 0; i < MAX_DECSTAGES; i++)
    m_pDecimatorPtrs[i] = NULL;
  m_Osc1.real(1.0); //initialize unit vector that will get rotated
  m_Osc1.imag(0.0);
}

CRDSDownConvert::~CRDSDownConvert()
{
  DeleteFilters();
}

//////////////////////////////////////////////////////////////////////
// Delete all active Filters in m_pDecimatorPtrs array
//////////////////////////////////////////////////////////////////////
void CRDSDownConvert::DeleteFilters()
{
  for (int i = 0; i < MAX_DECSTAGES; i++)
  {
    if (m_pDecimatorPtrs[i])
    {
      delete m_pDecimatorPtrs[i];
      m_pDecimatorPtrs[i] = NULL;
    }
  }
}

//////////////////////////////////////////////////////////////////////
// Sets NCO Frequency parameters
//////////////////////////////////////////////////////////////////////
void CRDSDownConvert::SetFrequency(RealType NcoFreq)
{
  RealType tmpf = NcoFreq + m_CW_Offset;

  m_NcoFreq = tmpf;
  m_NcoInc = K_2PI * m_NcoFreq / m_InRate;
  m_OscCos = MCOS(m_NcoInc);
  m_OscSin = MSIN(m_NcoInc);
  //qDebug()<<"NCO "<<m_NcoFreq;
}

//////////////////////////////////////////////////////////////////////
// Calculates sequence and number of decimation stages based on
// input sample rate and desired output bandwidth.  Returns final output rate
//from divide by 2 stages.
//////////////////////////////////////////////////////////////////////
RealType CRDSDownConvert::SetDataRate(RealType InRate, RealType MaxBW)
{
  int n = 0;
  RealType f = InRate;
  if ((m_InRate != InRate) || (m_MaxBW != MaxBW))
  {
    m_InRate = InRate;
    m_MaxBW = MaxBW;
    m_Mutex.lock();
    DeleteFilters();
    //loop until closest output rate is found and list of pointers to decimate by 2 stages is generated
    while ((f > (m_MaxBW / HB51TAP_MAX)) && (f > MIN_OUTPUT_RATE))
    {
      if (f >= (m_MaxBW / CIC3_MAX)) //See if can use CIC order 3
        m_pDecimatorPtrs[n++] = new CCicN3DecimateBy2;
      else if (f >= (m_MaxBW / HB11TAP_MAX)) //See if can use fixed 11 Tap Halfband
        m_pDecimatorPtrs[n++] = new CHalfBand11TapDecimateBy2();
      else if (f >= (m_MaxBW / HB15TAP_MAX)) //See if can use Halfband 15 Tap
        m_pDecimatorPtrs[n++] = new CHalfBandDecimateBy2(HB15TAP_LENGTH, HB15TAP_H);
      else if (f >= (m_MaxBW / HB19TAP_MAX)) //See if can use Halfband 19 Tap
        m_pDecimatorPtrs[n++] = new CHalfBandDecimateBy2(HB19TAP_LENGTH, HB19TAP_H);
      else if (f >= (m_MaxBW / HB23TAP_MAX)) //See if can use Halfband 23 Tap
        m_pDecimatorPtrs[n++] = new CHalfBandDecimateBy2(HB23TAP_LENGTH, HB23TAP_H);
      else if (f >= (m_MaxBW / HB27TAP_MAX)) //See if can use Halfband 27 Tap
        m_pDecimatorPtrs[n++] = new CHalfBandDecimateBy2(HB27TAP_LENGTH, HB27TAP_H);
      else if (f >= (m_MaxBW / HB31TAP_MAX)) //See if can use Halfband 31 Tap
        m_pDecimatorPtrs[n++] = new CHalfBandDecimateBy2(HB31TAP_LENGTH, HB31TAP_H);
      else if (f >= (m_MaxBW / HB35TAP_MAX)) //See if can use Halfband 35 Tap
        m_pDecimatorPtrs[n++] = new CHalfBandDecimateBy2(HB35TAP_LENGTH, HB35TAP_H);
      else if (f >= (m_MaxBW / HB39TAP_MAX)) //See if can use Halfband 39 Tap
        m_pDecimatorPtrs[n++] = new CHalfBandDecimateBy2(HB39TAP_LENGTH, HB39TAP_H);
      else if (f >= (m_MaxBW / HB43TAP_MAX)) //See if can use Halfband 43 Tap
        m_pDecimatorPtrs[n++] = new CHalfBandDecimateBy2(HB43TAP_LENGTH, HB43TAP_H);
      else if (f >= (m_MaxBW / HB47TAP_MAX)) //See if can use Halfband 47 Tap
        m_pDecimatorPtrs[n++] = new CHalfBandDecimateBy2(HB47TAP_LENGTH, HB47TAP_H);
      else if (f >= (m_MaxBW / HB51TAP_MAX)) //See if can use Halfband 51 Tap
        m_pDecimatorPtrs[n++] = new CHalfBandDecimateBy2(HB51TAP_LENGTH, HB51TAP_H);
      f /= 2.0;
    }
    m_Mutex.unlock();
    m_OutputRate = f;
    SetFrequency(m_NcoFreq);
  }
  return m_OutputRate;
}

//////////////////////////////////////////////////////////////////////
// Calculates sequence and number of decimation stages for WBFM based on
// input sample rate and desired output bandwidth.  Returns final output rate
//from divide by 2 stages.
//////////////////////////////////////////////////////////////////////
RealType CRDSDownConvert::SetWfmDataRate(RealType InRate, RealType MaxBW)
{
  int n = 0;
  RealType f = InRate;
  if ((m_InRate != InRate) || (m_MaxBW != MaxBW))
  {
    m_InRate = InRate;
    m_MaxBW = MaxBW;
    m_Mutex.lock();
    DeleteFilters();
    //loop until closest output rate is found and list of pointers to decimate by 2 stages is generated
    while ((f > 400000.0))
    {
      m_pDecimatorPtrs[n++] = new CRDSDownConvert::CHalfBandDecimateBy2(HB51TAP_LENGTH, HB51TAP_H);
      f /= 2.0;
    }
    m_OutputRate = f;
    m_Mutex.unlock();
    SetFrequency(m_NcoFreq);
  }
  return m_OutputRate;
}

//////////////////////////////////////////////////////////////////////
// Processes 'InLength' I/Q samples of 'pInData' buffer
// and places in 'pOutData' buffer.
// Returns number of samples available in output buffer.
// Make sure number of  input samples is large enough to have enough
// output samples to process in following stages since decimation
// process reduces the number of output samples per block.
// Also InLength must be a multiple of 2^N where N is the maximum
// decimation by 2 stages expected.
// ~50nSec/sample at decimation by 128
//////////////////////////////////////////////////////////////////////
int CRDSDownConvert::ProcessData(int InLength, ComplexType* pInData, ComplexType* pOutData)
{
  int i, j;
  ComplexType dtmp;
  ComplexType Osc;

  //StartPerformance();

#if (NCO_VCASM || NCO_GCCASM)
  RealType dPhaseAcc = m_NcoTime;
  RealType dASMCos = 0.0;
  RealType dASMSin = 0.0;
  RealType* pdCosAns = &dASMCos;
  RealType* pdSinAns = &dASMSin;
#endif

  //263uS using sin/cos or 70uS using quadrature osc or 200uS using _asm
  for (i = 0; i < InLength; i++)
  {
    dtmp = pInData[i];
#if NCO_LIB
    Osc.real() = MCOS(m_NcoTime);
    Osc.imag() = MSIN(m_NcoTime);
    m_NcoTime += m_NcoInc;
#elif NCO_OSC
    RealType OscGn;
    Osc.real(m_Osc1.real() * m_OscCos - m_Osc1.imag() * m_OscSin);
    Osc.imag(m_Osc1.imag() * m_OscCos + m_Osc1.real() * m_OscSin);
    OscGn = 1.95 - (m_Osc1.real() * m_Osc1.real() + m_Osc1.imag() * m_Osc1.imag());
    m_Osc1.real(OscGn * Osc.real());
    m_Osc1.imag(OscGn * Osc.imag());
#elif NCO_VCASM
    _asm
    {
      fld QWORD PTR [dPhaseAcc]
      fsincos
      mov ebx,[pdCosAns]    ;  get the pointer into ebx
      fstp QWORD PTR [ebx]  ;  store the result through the pointer
      mov ebx,[pdSinAns]
      fstp QWORD PTR [ebx]
    }
    dPhaseAcc += m_NcoInc;
    Osc.real() = dASMCos;
    Osc.imag() = dASMSin;
#elif NCO_GCCASM
    asm volatile("fsincos" : "=%&t"(dASMCos), "=%&u"(dASMSin) : "0"(dPhaseAcc));
    dPhaseAcc += m_NcoInc;
    Osc.real() = dASMCos;
    Osc.imag() = dASMSin;
#endif

    //Cpx multiply by shift frequency
    pInData[i].real(((dtmp.real() * Osc.real()) - (dtmp.imag() * Osc.imag())));
    pInData[i].imag(((dtmp.real() * Osc.imag()) + (dtmp.imag() * Osc.real())));
  }
#if (NCO_VCASM || NCO_GCCASM)
  m_NcoTime = dPhaseAcc;
#elif !NCO_OSC
  m_NcoTime = MFMOD(m_NcoTime, K_2PI); //keep radian counter bounded
#endif

  //now perform decimation of pInData by calling decimate by 2 stages
  //until NULL pointer encountered designating end of chain
  int n = InLength;
  j = 0;
  m_Mutex.lock();
  while (m_pDecimatorPtrs[j])
  {
    n = m_pDecimatorPtrs[j++]->DecBy2(n, pInData, pInData);
    //if(1==j)
    //g_pTestBench->DisplayData(n, 1.0, (ComplexType*)pInData, 615385/2.0);
  }
  m_Mutex.unlock();
  for (i = 0; i < n; i++)
    pOutData[i] = pInData[i];
  //StopPerformance(InLength);
  return n;
}

// *&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*

//////////////////////////////////////////////////////////////////////
//Decimate by 2 Halfband filter class implementation
//////////////////////////////////////////////////////////////////////
CRDSDownConvert::CHalfBandDecimateBy2::CHalfBandDecimateBy2(int len, const RealType* pCoef)
  : m_FirLength(len), m_pCoef(pCoef)
{
  //create buffer for FIR implementation
  m_pHBFirBuf = new ComplexType[MAX_HALF_BAND_BUFSIZE];
  ComplexType CPXZERO = {0.0, 0.0};
  for (int i = 0; i < MAX_HALF_BAND_BUFSIZE; i++)
    m_pHBFirBuf[i] = CPXZERO;
}

//////////////////////////////////////////////////////////////////////
// Half band filter and decimate by 2 function.
// Two restrictions on this routine:
// InLength must be larger or equal to the Number of Halfband Taps
// InLength must be an even number  ~37nS
//////////////////////////////////////////////////////////////////////
int CRDSDownConvert::CHalfBandDecimateBy2::DecBy2(int InLength,
                                                  ComplexType* pInData,
                                                  ComplexType* pOutData)
{
  int i;
  int j;
  int numoutsamples = 0;
  if (InLength < m_FirLength) //safety net to make sure InLength is large enough to process
    return InLength / 2;
  //StartPerformance();
  //copy input samples into buffer starting at position m_FirLength-1
  for (i = 0, j = m_FirLength - 1; i < InLength; i++)
    m_pHBFirBuf[j++] = pInData[i];
  //perform decimation FIR filter on even samples
  for (i = 0; i < InLength; i += 2)
  {
    ComplexType acc;
    acc.real((m_pHBFirBuf[i].real() * m_pCoef[0]));
    acc.imag((m_pHBFirBuf[i].imag() * m_pCoef[0]));
    for (j = 0; j < m_FirLength;
         j += 2) //only use even coefficients since odd are zero(except center point)
    {
      acc.real(acc.real() + (m_pHBFirBuf[i + j].real() * m_pCoef[j]));
      acc.imag(acc.imag() + (m_pHBFirBuf[i + j].imag() * m_pCoef[j]));
    }
    //now multiply the center coefficient
    acc.real(acc.real() +
             (m_pHBFirBuf[i + (m_FirLength - 1) / 2].real() * m_pCoef[(m_FirLength - 1) / 2]));
    acc.imag(acc.imag() +
             (m_pHBFirBuf[i + (m_FirLength - 1) / 2].imag() * m_pCoef[(m_FirLength - 1) / 2]));
    pOutData[numoutsamples++] = acc; //put output buffer
  }
  //need to copy last m_FirLength - 1 input samples in buffer to beginning of buffer
  // for FIR wrap around management
  for (i = 0, j = InLength - m_FirLength + 1; i < m_FirLength - 1; i++)
    m_pHBFirBuf[i] = pInData[j++];
  //StopPerformance(InLength);
  return numoutsamples;
}

// *&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*

//////////////////////////////////////////////////////////////////////
//Decimate by 2 Fixed 11 Tap Halfband filter class implementation
// Loop unrolled for speed ~9nS /samp
//////////////////////////////////////////////////////////////////////
CRDSDownConvert::CHalfBand11TapDecimateBy2::CHalfBand11TapDecimateBy2()
{
  //preload only the taps that are used since evey other one is zero
  //except center tap 5
  H0 = HB11TAP_H[0];
  H2 = HB11TAP_H[2];
  H4 = HB11TAP_H[4];
  H5 = HB11TAP_H[5];
  H6 = HB11TAP_H[6];
  H8 = HB11TAP_H[8];
  H10 = HB11TAP_H[10];
  ComplexType CPXZERO = {0.0, 0.0};
  d0 = CPXZERO;
  d1 = CPXZERO;
  d2 = CPXZERO;
  d3 = CPXZERO;
  d4 = CPXZERO;
  d5 = CPXZERO;
  d6 = CPXZERO;
  d7 = CPXZERO;
  d8 = CPXZERO;
  d9 = CPXZERO;
}

//////////////////////////////////////////////////////////////////////
//Decimate by 2 Fixed 11 Tap Halfband filter class implementation
// Two restrictions on this routine:
// InLength must be larger or equal to the Number of Halfband Taps(11)
// InLength must be an even number
// Loop unrolled for speed ~15nS/samp
//////////////////////////////////////////////////////////////////////
int CRDSDownConvert::CHalfBand11TapDecimateBy2::DecBy2(int InLength,
                                                       ComplexType* pInData,
                                                       ComplexType* pOutData)
{
  //StartPerformance();
  //first calculate beginning 10 samples using previous samples in delay buffer
  ComplexType tmpout[9]; //use temp buffer so outbuf can be same as inbuf
  tmpout[0].real(H0 * d0.real() + H2 * d2.real() + H4 * d4.real() + H5 * d5.real() +
                 H6 * d6.real() + H8 * d8.real() + H10 * pInData[0].real());
  tmpout[0].imag(H0 * d0.imag() + H2 * d2.imag() + H4 * d4.imag() + H5 * d5.imag() +
                 H6 * d6.imag() + H8 * d8.imag() + H10 * pInData[0].imag());

  tmpout[1].real(H0 * d2.real() + H2 * d4.real() + H4 * d6.real() + H5 * d7.real() +
                 H6 * d8.real() + H8 * pInData[0].real() + H10 * pInData[2].real());
  tmpout[1].imag(H0 * d2.imag() + H2 * d4.imag() + H4 * d6.imag() + H5 * d7.imag() +
                 H6 * d8.imag() + H8 * pInData[0].imag() + H10 * pInData[2].imag());

  tmpout[2].real(H0 * d4.real() + H2 * d6.real() + H4 * d8.real() + H5 * d9.real() +
                 H6 * pInData[0].real() + H8 * pInData[2].real() + H10 * pInData[4].real());
  tmpout[2].imag(H0 * d4.imag() + H2 * d6.imag() + H4 * d8.imag() + H5 * d9.imag() +
                 H6 * pInData[0].imag() + H8 * pInData[2].imag() + H10 * pInData[4].imag());

  tmpout[3].real(H0 * d6.real() + H2 * d8.real() + H4 * pInData[0].real() + H5 * pInData[1].real() +
                 H6 * pInData[2].real() + H8 * pInData[4].real() + H10 * pInData[6].real());
  tmpout[3].imag(H0 * d6.imag() + H2 * d8.imag() + H4 * pInData[0].imag() + H5 * pInData[1].imag() +
                 H6 * pInData[2].imag() + H8 * pInData[4].imag() + H10 * pInData[6].imag());

  tmpout[4].real(H0 * d8.real() + H2 * pInData[0].real() + H4 * pInData[2].real() +
                 H5 * pInData[3].real() + H6 * pInData[4].real() + H8 * pInData[6].real() +
                 H10 * pInData[8].real());
  tmpout[4].imag(H0 * d8.imag() + H2 * pInData[0].imag() + H4 * pInData[2].imag() +
                 H5 * pInData[3].imag() + H6 * pInData[4].imag() + H8 * pInData[6].imag() +
                 H10 * pInData[8].imag());

  tmpout[5].real(H0 * pInData[0].real() + H2 * pInData[2].real() + H4 * pInData[4].real() +
                 H5 * pInData[5].real() + H6 * pInData[6].real() + H8 * pInData[8].real() +
                 H10 * pInData[10].real());
  tmpout[5].imag(H0 * pInData[0].imag() + H2 * pInData[2].imag() + H4 * pInData[4].imag() +
                 H5 * pInData[5].imag() + H6 * pInData[6].imag() + H8 * pInData[8].imag() +
                 H10 * pInData[10].imag());

  tmpout[6].real(H0 * pInData[2].real() + H2 * pInData[4].real() + H4 * pInData[6].real() +
                 H5 * pInData[7].real() + H6 * pInData[8].real() + H8 * pInData[10].real() +
                 H10 * pInData[12].real());
  tmpout[6].imag(H0 * pInData[2].imag() + H2 * pInData[4].imag() + H4 * pInData[6].imag() +
                 H5 * pInData[7].imag() + H6 * pInData[8].imag() + H8 * pInData[10].imag() +
                 H10 * pInData[12].imag());

  tmpout[7].real(H0 * pInData[4].real() + H2 * pInData[6].real() + H4 * pInData[8].real() +
                 H5 * pInData[9].real() + H6 * pInData[10].real() + H8 * pInData[12].real() +
                 H10 * pInData[14].real());
  tmpout[7].imag(H0 * pInData[4].imag() + H2 * pInData[6].imag() + H4 * pInData[8].imag() +
                 H5 * pInData[9].imag() + H6 * pInData[10].imag() + H8 * pInData[12].imag() +
                 H10 * pInData[14].imag());

  tmpout[8].real(H0 * pInData[6].real() + H2 * pInData[8].real() + H4 * pInData[10].real() +
                 H5 * pInData[11].real() + H6 * pInData[12].real() + H8 * pInData[14].real() +
                 H10 * pInData[16].real());
  tmpout[8].imag(H0 * pInData[6].imag() + H2 * pInData[8].imag() + H4 * pInData[10].imag() +
                 H5 * pInData[11].imag() + H6 * pInData[12].imag() + H8 * pInData[14].imag() +
                 H10 * pInData[16].imag());

  //now loop through remaining input samples
  ComplexType* pIn = &pInData[8];
  ComplexType* pOut = &pOutData[9];
  for (int i = 0; i < (InLength - 11 - 6) / 2; i++)
  {
    (*pOut).real(H0 * pIn[0].real() + H2 * pIn[2].real() + H4 * pIn[4].real() + H5 * pIn[5].real() +
                 H6 * pIn[6].real() + H8 * pIn[8].real() + H10 * pIn[10].real());
    (*pOut++).imag(H0 * pIn[0].imag() + H2 * pIn[2].imag() + H4 * pIn[4].imag() +
                   H5 * pIn[5].imag() + H6 * pIn[6].imag() + H8 * pIn[8].imag() +
                   H10 * pIn[10].imag());
    pIn += 2;
  }
  //copy first outputs back into output array so outbuf can be same as inbuf
  pOutData[0] = tmpout[0];
  pOutData[1] = tmpout[1];
  pOutData[2] = tmpout[2];
  pOutData[3] = tmpout[3];
  pOutData[4] = tmpout[4];
  pOutData[5] = tmpout[5];
  pOutData[6] = tmpout[6];
  pOutData[7] = tmpout[7];
  pOutData[8] = tmpout[8];

  //copy last 10 input samples into delay buffer for next time
  pIn = &pInData[InLength - 1];
  d9 = *pIn--;
  d8 = *pIn--;
  d7 = *pIn--;
  d6 = *pIn--;
  d5 = *pIn--;
  d4 = *pIn--;
  d3 = *pIn--;
  d2 = *pIn--;
  d1 = *pIn--;
  d0 = *pIn;
  //StopPerformance(InLength);
  return InLength / 2;
}

// *&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*&*

//////////////////////////////////////////////////////////////////////
//Decimate by 2 CIC 3 stage
// -80dB alias rejection up to Fs * (.5 - .4985)
//////////////////////////////////////////////////////////////////////
CRDSDownConvert::CCicN3DecimateBy2::CCicN3DecimateBy2()
{
  m_Xodd = 0.0;
  m_Xeven = 0.0;
}

//////////////////////////////////////////////////////////////////////
//Function performs decimate by 2 using polyphase decompostion
// implemetation of a CIC N=3 filter.
// InLength must be an even number
//returns number of output samples processed
// 6nS/sample
//////////////////////////////////////////////////////////////////////
int CRDSDownConvert::CCicN3DecimateBy2::DecBy2(int InLength,
                                               ComplexType* pInData,
                                               ComplexType* pOutData)
{
  int i, j;
  ComplexType even, odd;
  //StartPerformance();
  for (i = 0, j = 0; i < InLength; i += 2, j++)
  { //mag gn=8
    even = pInData[i];
    odd = pInData[i + 1];
    pOutData[j].real(.125 * (odd.real() + m_Xeven.real() + 3.0 * (m_Xodd.real() + even.real())));
    pOutData[j].imag(.125 * (odd.imag() + m_Xeven.imag() + 3.0 * (m_Xodd.imag() + even.imag())));
    m_Xodd = odd;
    m_Xeven = even;
  }
  //StopPerformance(InLength);
  return j;
}
