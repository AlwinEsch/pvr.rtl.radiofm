/*
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#include "FreqShift.h"

cFreqShift::cFreqShift(RealType NcoFreq, RealType InRate)
{
  m_NcoTime = 0.0;
  m_InRate = InRate;
  m_NcoFreq = NcoFreq;
  m_NcoInc = K_2PI * m_NcoFreq / m_InRate;
}

void cFreqShift::Reset()
{
  m_NcoTime = 0.0;
}

void cFreqShift::Process(ComplexType* pInData, unsigned int InLength)
{
  ComplexType dtmp;
  ComplexType Osc;

#if (defined(__i386__) || defined(__x86_64__) || defined(__arm__) || defined(TARGET_WINDOWS))
  RealType dPhaseAcc = m_NcoTime;
  RealType dASMCos = 0.0;
  RealType dASMSin = 0.0;
#endif
#if TARGET_WINDOWS
  RealType* pdCosAns = &dASMCos;
  RealType* pdSinAns = &dASMSin;
#endif

  //263uS using sin/cos or 70uS using quadrature osc or 200uS using _asm
  for (unsigned int i = 0; i < InLength; ++i)
  {
    dtmp = pInData[i];
#if TARGET_WINDOWS
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
    Osc.re = dASMCos;
    Osc.im = dASMSin;
#elif (defined(__i386__) || defined(__x86_64__))
    asm volatile("fsincos" : "=%&t"(dASMCos), "=%&u"(dASMSin) : "0"(dPhaseAcc));
    dPhaseAcc += m_NcoInc;
    Osc = ComplexType(dASMCos, dASMSin);
#elif defined(__arm__)
    sincos_LP(m_NcoTime, dASMSin, dASMCos);
    Osc = ComplexType(dASMCos, dASMSin);
#else
    Osc = ComplexType(MCOS(m_NcoTime), MSIN(m_NcoTime));
    m_NcoTime += m_NcoInc;
#endif

    //Cpx multiply by shift frequency
    pInData[i] = ComplexType((dtmp.real() * Osc.real()) - (dtmp.imag() * Osc.imag()),
                             (dtmp.real() * Osc.imag()) + (dtmp.imag() * Osc.real()));
  }
#if (defined(__i386__) || defined(__x86_64__) || defined(TARGET_WINDOWS))
  m_NcoTime = dPhaseAcc;
#else
  m_NcoTime = MFMOD(m_NcoTime, K_2PI); //keep radian counter bounded
#endif
}
