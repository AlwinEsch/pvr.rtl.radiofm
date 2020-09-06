/*
 *  Copyright (C) 2010-2013, Moe Wheatley
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#pragma once

#include "Definitions.h"

class ATTRIBUTE_HIDDEN cIirFilter
{
public:
  cIirFilter() = default;
  virtual ~cIirFilter() = default;

  bool Init(eFilterType type, RealType F0Freq, RealType FilterQ, RealType SampleRate);

  void Process(ComplexType* buffer, unsigned int length);
  void Process(RealType* buffer, unsigned int length);
  void ProcessTwo(RealType* bufferA, RealType* bufferB, unsigned int length);

private:
  RealType m_A1; //!< direct form 2 coefficients
  RealType m_A2;
  RealType m_B0;
  RealType m_B1;
  RealType m_B2;

  RealType m_w1a; //!< biquad delay storage
  RealType m_w2a;
  RealType m_w1b; //!< biquad delay storage
  RealType m_w2b;
};
