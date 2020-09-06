/*
 *  Copyright (C) 2013, Joris van Rantwijk.
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#pragma once

#include "Definitions.h"

#include <mutex>

/*!
 *  Downsampler with low-pass FIR filter for real-valued signals.
 *
 *  Step 1: Low-pass filter based on Lanczos FIR filter
 *  Step 2: (optional) Decimation by an arbitrary factor (integer or float)
 */
class ATTRIBUTE_HIDDEN cDownsampleFilter
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
  cDownsampleFilter(unsigned int filter_order,
                    double cutoff,
                    double downsample = 1,
                    bool integer_factor = true);
  virtual ~cDownsampleFilter();

  void Reset();

  /*! Process samples. */
  unsigned int Process(const RealType* samples_in, RealType* samples_out, unsigned int length);
  unsigned int Process(const ComplexType* samples_in,
                       ComplexType* samples_out,
                       unsigned int length);

private:
  double m_downsample;
  unsigned int m_downsample_int;
  unsigned int m_pos_int;
  RealType m_pos_frac;
  RealType* m_coeff;

  unsigned int m_stateOrderSize;
  RealType* m_stateReal;
  ComplexType* m_stateComplex;
};


#define MAX_DECSTAGES 10 //one more than max to make sure is a null at end of list

//////////////////////////////////////////////////////////////////////////////////
// Main Downconverter Class
//////////////////////////////////////////////////////////////////////////////////
class CRDSDownConvert
{
public:
  CRDSDownConvert();
  virtual ~CRDSDownConvert();
  void SetFrequency(RealType NcoFreq);
  void SetCwOffset(RealType offset) { m_CW_Offset = offset; }
  int ProcessData(int InLength, ComplexType* pInData, ComplexType* pOutData);
  RealType SetDataRate(RealType InRate, RealType MaxBW);
  RealType SetWfmDataRate(RealType InRate, RealType MaxBW);

private:
  ////////////
  //pure abstract base class for all the different types of decimate by 2 stages
  //DecBy2 function is defined in derived classes
  ////////////
  class CDec2
  {
  public:
    CDec2() {}
    virtual ~CDec2() {}
    virtual int DecBy2(int InLength, ComplexType* pInData, ComplexType* pOutData) = 0;
  };

  ////////////
  //private class for the Half Band decimate by 2 stages
  ////////////
  class CHalfBandDecimateBy2 : public CDec2
  {
  public:
    CHalfBandDecimateBy2(int len, const RealType* pCoef);
    ~CHalfBandDecimateBy2()
    {
      if (m_pHBFirBuf)
        delete m_pHBFirBuf;
    }
    int DecBy2(int InLength, ComplexType* pInData, ComplexType* pOutData);
    ComplexType* m_pHBFirBuf;
    int m_FirLength;
    const RealType* m_pCoef;
  };


  ////////////
  //private class for the fixed 11 tap Half Band decimate by 2 stages
  ////////////
  class CHalfBand11TapDecimateBy2 : public CDec2
  {
  public:
    CHalfBand11TapDecimateBy2();
    ~CHalfBand11TapDecimateBy2() {}
    int DecBy2(int InLength, ComplexType* pInData, ComplexType* pOutData);
    RealType H0; //unwrapped coeeficients
    RealType H2;
    RealType H4;
    RealType H5;
    RealType H6;
    RealType H8;
    RealType H10;
    ComplexType d0; //unwrapped delay buffer
    ComplexType d1;
    ComplexType d2;
    ComplexType d3;
    ComplexType d4;
    ComplexType d5;
    ComplexType d6;
    ComplexType d7;
    ComplexType d8;
    ComplexType d9;
  };

  ////////////
  //private class for the N=3 CIC decimate by 2 stages
  ////////////
  class CCicN3DecimateBy2 : public CDec2
  {
  public:
    CCicN3DecimateBy2();
    ~CCicN3DecimateBy2() {}
    int DecBy2(int InLength, ComplexType* pInData, ComplexType* pOutData);
    ComplexType m_Xodd;
    ComplexType m_Xeven;
  };

private:
  //private helper functions
  void DeleteFilters();

  RealType m_OutputRate;
  RealType m_NcoFreq;
  RealType m_CW_Offset;
  RealType m_NcoInc;
  RealType m_NcoTime;
  RealType m_InRate;
  RealType m_MaxBW;
  ComplexType m_Osc1;
  RealType m_OscCos;
  RealType m_OscSin;
  std::mutex m_Mutex; //for keeping threads from stomping on each other
  //array of pointers for performing decimate by 2 stages
  CDec2* m_pDecimatorPtrs[MAX_DECSTAGES];
};
