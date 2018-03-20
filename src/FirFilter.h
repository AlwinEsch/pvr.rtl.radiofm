#pragma once
/*
 *      Copyright (C) 2010-2013, Moe Wheatley
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
 *  This part of code is taken from CuteSDR created by Moe Wheatley (Copyright 2010)
 *  and handled by Simplified BSD License
 */

#include "Definations.h"

#define MAX_NUMCOEF 75

class cFirFilter
{
public:
  cFirFilter();
  virtual ~cFirFilter() {}

  void InitConstFir(unsigned int NumTaps, const RealType* pCoef, RealType Fsamprate);
  void InitConstFir(unsigned int NumTaps, const RealType* pICoef, const RealType* pQCoef, RealType Fsamprate);
  int InitLPFilter(unsigned int NumTaps, RealType Scale, RealType Astop, RealType Fpass, RealType Fstop, RealType Fsamprate);
  int InitHPFilter(unsigned int NumTaps, RealType Scale, RealType Astop, RealType Fpass, RealType Fstop, RealType Fsamprate);
  void GenerateHBFilter( RealType FreqOffset);

  void Process(ComplexType* buffer, unsigned int length);
  void Process(RealType* buffer, unsigned int length);
  void Process(RealType* InBuf, ComplexType* OutBuf, unsigned int length);
  void Process(ComplexType* InBuf, ComplexType* OutBuf, unsigned int length);

  void ProcessTwo(RealType* bufferA, RealType* bufferB, unsigned int length);

private:
  RealType      Izero(RealType x);

  RealType      m_SampleRate;
  unsigned int  m_NumTaps;
  int           m_State;
  RealType      m_Coef[MAX_NUMCOEF*2];
  RealType      m_ICoef[MAX_NUMCOEF*2];
  RealType      m_QCoef[MAX_NUMCOEF*2];
  RealType      m_rZBuf[MAX_NUMCOEF];
  ComplexType   m_cZBuf[MAX_NUMCOEF];
};
