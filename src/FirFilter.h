/*
 *  Copyright (C) 2010-2013, Moe Wheatley
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#pragma once

#include "Definitions.h"

#include <mutex>

#define MAX_NUMCOEF 75

class ATTRIBUTE_HIDDEN cFirFilter
{
public:
  cFirFilter();
  virtual ~cFirFilter() {}

  void InitConstFir(unsigned int NumTaps, const RealType* pCoef, RealType Fsamprate);
  void InitConstFir(unsigned int NumTaps,
                    const RealType* pICoef,
                    const RealType* pQCoef,
                    RealType Fsamprate);
  int InitLPFilter(unsigned int NumTaps,
                   RealType Scale,
                   RealType Astop,
                   RealType Fpass,
                   RealType Fstop,
                   RealType Fsamprate);
  int InitHPFilter(unsigned int NumTaps,
                   RealType Scale,
                   RealType Astop,
                   RealType Fpass,
                   RealType Fstop,
                   RealType Fsamprate);
  void GenerateHBFilter(RealType FreqOffset);

  void Process(ComplexType* buffer, unsigned int length);
  void Process(RealType* buffer, unsigned int length);
  void Process(RealType* InBuf, ComplexType* OutBuf, unsigned int length);
  void Process(ComplexType* InBuf, ComplexType* OutBuf, unsigned int length);

  void ProcessTwo(RealType* bufferA, RealType* bufferB, unsigned int length);

private:
  RealType Izero(RealType x);

  RealType m_SampleRate;
  unsigned int m_NumTaps;
  int m_State;
  RealType m_Coef[MAX_NUMCOEF * 2];
  RealType m_ICoef[MAX_NUMCOEF * 2];
  RealType m_QCoef[MAX_NUMCOEF * 2];
  RealType m_rZBuf[MAX_NUMCOEF];
  ComplexType m_cZBuf[MAX_NUMCOEF];
};
