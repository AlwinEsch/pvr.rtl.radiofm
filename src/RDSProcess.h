#pragma once
/*
 *      Copyright (C) 2010-2013, Moe Wheatley
 *      Copyright (C) 2015-2018 Team KODI (Alwin Esch)
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
 *  along with this Software; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *  http://www.gnu.org/copyleft/gpl.html
 *
 *  This part of code is taken from CuteSDR created by Moe Wheatley (Copyright 2010)
 *  and handled there by Simplified BSD License and slightly modified
 *
 */

#include "Definations.h"
#include "DownConvert.h"
#include "IirFilter.h"
#include "FirFilter.h"
#include "FreqShift.h"
#include "RDSGroupDecoder.h"

#define MAX_RDS_ARRAY_SIZE      (16 * 32 * 512)

#define RDS_FREQUENCY           57000.0
#define RDS_BITRATE             (RDS_FREQUENCY/48.0)            //!< 1187.5 bps bitrate
#define RDSPLL_RANGE            12.0                            //!< maximum deviation limit of PLL
#define RDSPLL_BW               1.00                            //!< natural frequency ~loop bandwidth
#define RDSPLL_ZETA             0.707                           //!< PLL Loop damping factor

#define USE_FEC                 1                               //!< set to zero to disable FEC correction

#define NUMBITS_CRC             10
#define NUMBITS_MSG             16
#define NUMBITS_BLOCK           (NUMBITS_CRC + NUMBITS_MSG)
#define BLOCK_ERROR_LIMIT       0                               //!< number of bad blocks before trying to resync at the bit level

#define CRC_POLY                0x5B9                           //!< RDS crc polynomial  x^10+x^8+x^7+x^5+x^4+x^3+1

#define GROUPB_BIT              0x0800                          //!< bit position in BlockB for determining group A or B msgs

//! RDS decoder states
#define STATE_BITSYNC           0                               //!< looking for initial bit position in Block 1
#define STATE_BLOCKSYNC         1                               //!< looking for initial correct block order
#define STATE_GROUPDECODE       2                               //!< decode groups after achieving bit  and block sync
#define STATE_GROUPRESYNC       3                               //!< waiting for beginning of new group after getting a block error

/*!
 * Block offset syndromes for chkword checking
 * created multiplying Block Offsets by parity check matrix
 * Takes into account that the (26,16)code is a shortened cyclic block code
 * from the original (341,331) length cyclic code.
 */
#define OFFSET_SYNDROME_BLOCK_A  0x3D8
#define OFFSET_SYNDROME_BLOCK_B  0x3D4
#define OFFSET_SYNDROME_BLOCK_C  0x25C
#define OFFSET_SYNDROME_BLOCK_CP 0x3CC
#define OFFSET_SYNDROME_BLOCK_D  0x258

class cRadioReceiver;

class cRDSRxSignalProcessor
{
public:
  cRDSRxSignalProcessor(cRadioReceiver *proc, RealType SampleRate);
  virtual ~cRDSRxSignalProcessor();
  void Reset();

  void Process(const RealType *inputStream, unsigned int items);

private:
  void ProcessRdsPll(ComplexType* pInData, RealType* pOutData, unsigned int InLength);
  void ProcessNewRdsBit(int bit);
  uint32_t CheckBlock(uint32_t SyndromeOffset, int UseFec);

  RealType            m_SampleRate;
  cRDSGroupDecoder    m_Decoder;
  RealType            m_ProcessRate;

  ComplexType        *m_ProcessArrayIn;
  ComplexType        *m_RdsRaw;              //variables for RDS processing
  RealType           *m_RdsMag;
  RealType           *m_RdsData;
  RealType           *m_RdsMatchCoef;
  unsigned int        m_RdsMatchCoefLength;

  RealType            m_RdsNcoPhase;    //variables for RDS PLL
  RealType            m_RdsNcoFreq;
  RealType            m_RdsNcoAcc;
  RealType            m_RdsNcoLLimit;
  RealType            m_RdsNcoHLimit;
  RealType            m_RdsPllAlpha;
  RealType            m_RdsPllBeta;
  RealType            m_RdsLastData;
  int                 m_DownConvertFactor;

  cFirFilter          m_firLPFilter;
  cFirFilter          m_RdsMatchedFilter;
  cIirFilter          m_RdsBitSyncFilter;
  cFreqShift          m_InputFreqShift;
  cDownsampleFilter   m_DownConvert;

  int                 m_RdsLastBit;
  uint32_t            m_InBitStream;    //input shift register for incoming raw data
  int                 m_CurrentBlock;
  int                 m_CurrentBitPosition;
  int                 m_DecodeState;
  int                 m_BGroupOffset;
  int                 m_BlockErrors;
  uint16_t            m_BlockData[4];

  RealType            m_ClockLastValue;
  bool                m_ClockInput;
  unsigned int        m_DataInputPrev;
};
