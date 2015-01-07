#pragma once
/*
 *      Copyright (C) 2015 Team KODI (Alwin Esch)
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
 */

#include  <stdint.h>
#include  <vector>

#define BLOCK__A 0  //indexes for the four blocks
#define BLOCK__B 1
#define BLOCK__C 2
#define BLOCK__D 3

class  cRadioReceiver;

#define RT_MEL 66

class  cRDSGroupDecoder
{
public:
  cRDSGroupDecoder(cRadioReceiver *proc);
  virtual ~cRDSGroupDecoder() {};

  void DecodeRDS(uint16_t *blockData);
  void Reset();

private:
  enum eGroupType
  {
    RDS_GROUP_TYPE__0A            = 0x00,
    RDS_GROUP_TYPE__0B            = 0x01,
    RDS_GROUP_TYPE__1A            = 0x02,
    RDS_GROUP_TYPE__1B            = 0x03,
    RDS_GROUP_TYPE__2A            = 0x04,
    RDS_GROUP_TYPE__2B            = 0x05,
    RDS_GROUP_TYPE__3A            = 0x06,
    RDS_GROUP_TYPE__3B            = 0x07,
    RDS_GROUP_TYPE__4A            = 0x08,
    RDS_GROUP_TYPE__4B            = 0x09,
    RDS_GROUP_TYPE__5A            = 0x0A,
    RDS_GROUP_TYPE__5B            = 0x0B,
    RDS_GROUP_TYPE__6A            = 0x0C,
    RDS_GROUP_TYPE__6B            = 0x0D,
    RDS_GROUP_TYPE__7A            = 0x0E,
    RDS_GROUP_TYPE__7B            = 0x0F,
    RDS_GROUP_TYPE__8A            = 0x10,
    RDS_GROUP_TYPE__8B            = 0x11,
    RDS_GROUP_TYPE__9A            = 0x12,
    RDS_GROUP_TYPE__9B            = 0x13,
    RDS_GROUP_TYPE_10A            = 0x14,
    RDS_GROUP_TYPE_10B            = 0x15,
    RDS_GROUP_TYPE_11A            = 0x16,
    RDS_GROUP_TYPE_11B            = 0x17,
    RDS_GROUP_TYPE_12A            = 0x18,
    RDS_GROUP_TYPE_12B            = 0x19,
    RDS_GROUP_TYPE_13A            = 0x1A,
    RDS_GROUP_TYPE_13B            = 0x1B,
    RDS_GROUP_TYPE_14A            = 0x1C,
    RDS_GROUP_TYPE_14B            = 0x1D,
    RDS_GROUP_TYPE_15A            = 0x1E,
    RDS_GROUP_TYPE_15B            = 0x1F
  };

  cRadioReceiver *m_RadioProc;

  uint8_t m_UECPDataFrameSeqCnt;
  int     m_UECPDataMsgLength;
  int     m_UECPStuffingPtr;
  uint8_t m_UECPDataFrame[263];

  int     m_ODATypeMap[32];

  int     m_PTY;

  int     m_TA_TP;

  char    m_PTYN[9];
  bool    m_PTYN_ABFlag;
  int     m_PTYN_SetFlag;

  uint8_t m_DI;
  uint8_t m_DI_Prev;
  int     m_DI_Finished;

  uint8_t m_MS;
  uint8_t m_MS_Prev;

  char    m_PS_Name[9];  ///< Programme service name
  int     m_PS_SetFlag;

  uint16_t m_PIN;
  uint16_t m_ProgramIdentCode;

  char    m_RadioText_Temp[RT_MEL];
  bool    m_RadioText_FirstPtr;
  int     m_RadioText_LastPtr;
  int     m_RadioText_ABFlag;
  bool    m_RadioText_Ready;
  uint32_t m_RadioText_SegmentRegister;
  int     m_RadioText_Count;

  int     m_RTPlus_templateNo;
  int     m_RTPlus_scb;
  int     m_RTPlus_cbflag;
  int     m_RTPlus_rfu;

  bool    m_AF_MethodStarted;
  bool    m_AF_MethodFinished;
  bool    m_AF_LowFreqFollow;
  int     m_AF_CalcCount;
  bool    m_AF_MethodAB;
  struct sAFMethod
  {
    uint8_t Value;
    bool  Regional;
    float Frequency;
  };
  std::vector<sAFMethod>    m_AF_AltFrequencies;

  void ClearUECPFrame();
  uint16_t crc16_ccitt(const uint8_t *daten, int len, bool skipfirst = true);
  void AddStuffingValue(uint8_t value);
  void SendUECPFrame();

  void Decode_PI(uint16_t identifier);
  void Decode_PTY(int pty);
  void Decode_Type0___PS_DI_MS                  (const uint16_t *msgElement, bool versionCode);
  void Decode_Type1___ProgItemNum_SlowLabelCodes(const uint16_t *msgElement, bool versionCode);
  void Decode_Type2___Radiotext                 (const uint16_t *msgElement, bool versionCode);
  void Decode_Type3A__AppIdentOpenData          (const uint16_t *msgElement);
  void Decode_Type4A__Clock                     (const uint16_t *msgElement);
  void Decode_Type5___TransparentDataChannels   (const uint16_t *msgElement, bool versionCode);
  void Decode_Type6___InHouseApplications       (const uint16_t *msgElement, bool versionCode);
  void Decode_Type7A__RadioPaging               (const uint16_t *msgElement);
  void Decode_Type8A__TrafficMessageChannel     (const uint16_t *msgElement);
  void Decode_Type9A__EmergencyWarningSystem    (const uint16_t *msgElement);
  void Decode_Type10A_PTYN                      (const uint16_t *msgElement);
  void Decode_Type13A_EnhancedRadioPaging       (const uint16_t *msgElement);
  void Decode_Type14__EnhancedOtherNetworksInfo (const uint16_t *msgElement, bool versionCode);
  void Decode_Type15A_RBDS                      (const uint16_t *msgElement);
  void Decode_Type15B_FastSwitchingInfo         (const uint16_t *msgElement);
  void Decode_Type____ODA                       (const uint16_t *msgElement, int odaFunction);
  void Decode_TypeRTPlus(const uint16_t *msgElement);
};
