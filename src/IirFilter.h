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

class cIirFilter
{
public:
  cIirFilter();
  virtual ~cIirFilter();

  bool Init(eFilterType type, RealType F0Freq, RealType FilterQ, RealType SampleRate);

  void Process(ComplexType* buffer, unsigned int length);
  void Process(RealType* buffer, unsigned int length);
  void ProcessTwo(RealType* bufferA, RealType* bufferB, unsigned int length);

private:
  RealType m_A1;    //!< direct form 2 coefficients
  RealType m_A2;
  RealType m_B0;
  RealType m_B1;
  RealType m_B2;

  RealType m_w1a;    //!< biquad delay storage
  RealType m_w2a;
  RealType m_w1b;    //!< biquad delay storage
  RealType m_w2b;
};
