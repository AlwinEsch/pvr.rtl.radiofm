/*
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#pragma once

#include "Definitions.h"

class ATTRIBUTE_HIDDEN cFreqShift
{
public:
  cFreqShift(RealType NcoFreq, RealType InRate);
  virtual ~cFreqShift() = default;

  void Reset();

  void Process(ComplexType* pInData, unsigned int InLength);

private:
  RealType m_NcoFreq;
  RealType m_NcoInc;
  RealType m_NcoTime;
  RealType m_InRate;
};
