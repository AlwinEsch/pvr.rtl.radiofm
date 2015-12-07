/*
 *      Copyright (C) 2010-2013, Moe Wheatley
 *      Copyright (C) 2015, Alwin Esch (Team KODI)
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

#include "IirFilter.h"

cIirFilter::cIirFilter()
{
}

cIirFilter::~cIirFilter()
{
}

bool cIirFilter::Init(eFilterType type, RealType F0Freq, RealType FilterQ, RealType SampleRate)
{
  bool ret = true;

  RealType w0 = K_2PI * F0Freq / SampleRate;  //normalized corner frequency
  RealType alpha = MSIN(w0) / (2.0 * FilterQ);
  RealType A = 1.0 / (1.0 + alpha);  //scale everything by 1/A0 for direct form 2

  switch (type)
  {
    case ftLP:
      m_B0 =  A * ( (1.0 - MCOS(w0)) / 2.0);
      m_B1 =  A * (  1.0 - MCOS(w0));
      m_B2 =  A * ( (1.0 - MCOS(w0)) / 2.0);
      m_A1 =  A * ( -2.0 * MCOS(w0));
      m_A2 =  A * (  1.0 - alpha);
      break;
    case ftHP:
      m_B0 =  A * ( (1.0 + MCOS(w0))/2.0);
      m_B1 = -A * (  1.0 + MCOS(w0));
      m_B2 =  A * ( (1.0 + MCOS(w0))/ 2.0);
      m_A1 =  A * ( -2.0 * MCOS(w0));
      m_A2 =  A * (  1.0 - alpha);
      break;
    case ftBP:
      m_B0 =  A * alpha;
      m_B1 =  0.0;
      m_B2 =  A * -alpha;
      m_A1 =  A * ( -2.0 * MCOS(w0));
      m_A2 =  A * (  1.0 - alpha);
      break;
    case ftBR:
      m_B0 =  A * 1.0;
      m_B1 =  A * ( -2.0 * MCOS(w0));
      m_B2 =  A * 1.0;
      m_A1 =  A * ( -2.0 * MCOS(w0));
      m_A2 =  A * ( 1.0 - alpha);
      break;
    default:
      ret = false;
      break;
  }

  m_w1a = 0.0;
  m_w2a = 0.0;
  m_w1b = 0.0;
  m_w2b = 0.0;

  return ret;
}

void cIirFilter::Process(ComplexType* buffer, unsigned int length)
{
  for (unsigned int i = 0; i < length; ++i)
  {
    RealType w0a = buffer[i].real() - m_A1 * m_w1a - m_A2 * m_w2a;
    RealType w0b = buffer[i].imag() - m_A1 * m_w1b - m_A2 * m_w2b;
    buffer[i] = ComplexType(m_B0 * w0a + m_B1 * m_w1a + m_B2 * m_w2a,
                            m_B0 * w0b + m_B1 * m_w1b + m_B2 * m_w2b);
    m_w2a = m_w1a;
    m_w1a = w0a;

    m_w2b = m_w1b;
    m_w1b = w0b;
  }
}

void cIirFilter::Process(RealType* buffer, unsigned int length)
{
  for (unsigned int i = 0; i < length; ++i)
  {
    RealType w0 = buffer[i] - m_A1 * m_w1a - m_A2 * m_w2a;
    buffer[i] = m_B0 * w0 + m_B1 * m_w1a + m_B2 * m_w2a;
    m_w2a = m_w1a;
    m_w1a = w0;
  }
}

void cIirFilter::ProcessTwo(RealType* bufferA, RealType* bufferB, unsigned int length)
{
  for (unsigned int i = 0; i < length; ++i)
  {
    RealType w0a = bufferA[i] - m_A1 * m_w1a - m_A2 * m_w2a;
    RealType w0b = bufferB[i] - m_A1 * m_w1b - m_A2 * m_w2b;

    bufferA[i] = m_B0 * w0a + m_B1 * m_w1a + m_B2 * m_w2a;
    bufferB[i] = m_B0 * w0b + m_B1 * m_w1b + m_B2 * m_w2b;

    m_w2a = m_w1a;
    m_w1a = w0a;

    m_w2b = m_w1b;
    m_w1b = w0b;
  }
}

