/*
 *      Copyright (C) 2015 Alwin Esch (Team KODI)
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

#include "FreqShift.h"

using namespace std;

cFreqShift::cFreqShift(RealType NcoFreq, RealType InRate)
{
	m_NcoTime = 0.0;
	m_InRate  = InRate;
	m_NcoFreq = NcoFreq;
	m_NcoInc  = K_2PI * m_NcoFreq / m_InRate;
}

void cFreqShift::Process(ComplexType* pInData, unsigned int InLength)
{
  ComplexType dtmp;
  ComplexType Osc;

#if (defined(__i386__) || defined(__x86_64__) || defined(TARGET_WINDOWS))
  RealType	dPhaseAcc = m_NcoTime;
  RealType	dASMCos   = 0.0;
  RealType	dASMSin   = 0.0;
#endif
#if TARGET_WINDOWS
  RealType*	pdCosAns  = &dASMCos;
  RealType*	pdSinAns  = &dASMSin;
#endif

//263uS using sin/cos or 70uS using quadrature osc or 200uS using _asm
	for (unsigned int i = 0; i < InLength; i++)
	{
		dtmp = pInData[i];
#if TARGET_WINDOWS
		_asm
		{
			fld QWORD PTR [dPhaseAcc]
			fsincos
			mov ebx,[pdCosAns]		;	get the pointer into ebx
			fstp QWORD PTR [ebx]	;	store the result through the pointer
			mov ebx,[pdSinAns]
			fstp QWORD PTR [ebx]
		}
		dPhaseAcc += m_NcoInc;
		Osc.re = dASMCos;
		Osc.im = dASMSin;
#elif (defined(__i386__) || defined(__x86_64__))
		asm volatile ("fsincos" : "=%&t" (dASMCos), "=%&u" (dASMSin) : "0" (dPhaseAcc));
		dPhaseAcc += m_NcoInc;
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
	m_NcoTime = MFMOD(m_NcoTime, K_2PI);	//keep radian counter bounded
#endif
}
