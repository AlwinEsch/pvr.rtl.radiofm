#pragma once
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

#include "Definations.h"

class cFreqShift
{
public:
  cFreqShift(RealType NcoFreq, RealType InRate);
  virtual ~cFreqShift() {};

  void Reset();

  void Process(ComplexType* pInData, unsigned int InLength);

private:
  RealType    m_NcoFreq;
  RealType    m_NcoInc;
  RealType    m_NcoTime;
  RealType    m_InRate;
};
