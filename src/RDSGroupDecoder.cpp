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

#include  <string.h>

#include "client.h"
#include "RadioReceiver.h"
#include "RDSGroupDecoder.h"
#include "utils.h"

//#define IMPROVE_CHECK 1

using namespace std;
using namespace ADDON;

/**
 * Universal Encoder Communication Protocol (UECP)
 * List of defined commands
 * iaw.: SPB 490
 */

/// UECP Message element pointers (different on several commands)
#define UECP_ME_MEC                     0       // Message Element Code
#define UECP_ME_DSN                     1       // Data Set Number
#define UECP_ME_PSN                     2       // Program Service Number
#define UECP_ME_MEL                     3       // Message Element data Length
#define UECP_ME_DATA                    4       //

/// RDS message commands
#define UECP_RDS_PI                     0x01    // Program Identification
#define UECP_RDS_PS                     0x02    // Program Service name
#define UECP_RDS_PIN                    0x06    // Program Item Number
#define UECP_RDS_DI                     0x04    // Decoder Identification and dynamic PTY indicator
#define UECP_RDS_TA_TP                  0x03    // Traffic Announcement identification / Traffic Program identification
#define UECP_RDS_MS                     0x05    // Music/Speech switch
#define UECP_RDS_PTY                    0x07    // Program TYpe
#define UECP_RDS_PTYN                   0x3A    // Program TYpe Name
#define UECP_RDS_RT                     0x0A    // RadioText
#define UECP_RDS_AF                     0x13    // Alternative Frequencies list
#define UECP_RDS_EON_AF                 0x14    // Enhanced Other Networks information
#define UECP_SLOW_LABEL_CODES           0x1A    // Slow Labeling codes
#define UECP_LINKAGE_INFO               0x2E    // Linkage information

/// Open Data Application commands
#define UECP_ODA_CONF_SHORT_MSG_CMD     0x40    // ODA configuration and short message command
#define UECP_ODA_IDENT_GROUP_USAGE_SEQ  0x41    // ODA identification group usage sequence
#define UECP_ODA_FREE_FORMAT_GROUP      0x42    // ODA free-format group
#define UECP_ODA_REL_PRIOR_GROUP_SEQ    0x43    // ODA relative priority group sequence
#define UECP_ODA_BURST_MODE_CONTROL     0x44    // ODA “Burst mode” control
#define UECP_ODA_SPINN_WHEEL_TIMING_CTL 0x45    // ODA “Spinning Wheel” timing control
#define UECP_ODA_DATA                   0x46    // ODA Data
#define UECP_ODA_DATA_CMD_ACCESS_RIGHT  0x47    // ODA data command access right

/// DAB
#define UECP_DAB_DYN_LABEL_CMD          0x48    // DAB: Dynamic Label command
#define UECP_DAB_DYN_LABEL_MSG          0xAA    // DAB: Dynamic Label message (DL)

/// Transparent data commands
#define UECP_TDC_TDC                    0x26    // TDC
#define UECP_TDC_EWS                    0x2B    // EWS
#define UECP_TDC_IH                     0x25    // IH
#define UECP_TDC_TMC                    0x30    // TMC
#define UECP_TDC_FREE_FMT_GROUP         0x24    // Free-format group

/// Paging commands
#define UECP_PAGING_CALL_WITHOUT_MESSAGE                                        0x0C
#define UECP_PAGING_CALL_NUMERIC_MESSAGE_10DIGITS                               0x08
#define UECP_PAGING_CALL_NUMERIC_MESSAGE_18DIGITS                               0x20
#define UECP_PAGING_CALL_ALPHANUMERIC_MESSAGE_80CHARACTERS                      0x1B
#define UECP_INTERNATIONAL_PAGING_NUMERIC_MESSAGE_15DIGITS                      0x11
#define UECP_INTERNATIONAL_PAGING_FUNCTIONS_MESSAGE                             0x10
#define UECP_TRANSMITTER_NETWORK_GROUP_DESIGNATION                              0x12
#define UECP_EPP_TM_INFO                                                        0x31
#define UECP_EPP_CALL_WITHOUT_ADDITIONAL_MESSAGE                                0x32
#define UECP_EPP_NATIONAL_INTERNATIONAL_CALL_ALPHANUMERIC_MESSAGE               0x33
#define UECP_EPP_NATIONAL_INTERNATIONAL_CALL_VARIABLE_LENGTH_NUMERIC_MESSAGE    0x34
#define UECP_EPP_NATIONAL_INTERNATIONAL_CALL_VARIABLE_LENGTH_FUNCTIONS_MESSAGE  0x35

/// Clock setting and control
#define UECP_CLOCK_RTC                0x0D    // Real time clock
#define UECP_CLOCK_RTC_CORR           0x09    // Real time clock correction
#define UECP_CLOCK_CT_ON_OFF          0x19    // CT On/Off

/// RDS adjustment and control
#define RDS_ON_OFF                    0x1E
#define RDS_PHASE                     0x22
#define RDS_LEVEL                     0x0E

/// ARI adjustment and control
#define UECP_ARI_ARI_ON_OFF           0x21
#define UECP_ARI_ARI_AREA (BK)        0x0F
#define UECP_ARI_ARI_LEVEL            0x1F

/// Control and set up commands
#define UECP_CTR_SITE_ADDRESS         0x23
#define UECP_CTR_ENCODER_ADDRESS      0x27
#define UECP_CTR_MAKE_PSN_LIST        0x28
#define UECP_CTR_PSN_ENABLE_DISABLE   0x0B
#define UECP_CTR_COMMUNICATION_MODE   0x2C
#define UECP_CTR_TA_CONTROL           0x2A
#define UECP_CTR_EON_TA_CONTROL       0x15
#define UECP_CTR_REFERENCE_INPUT_SEL  0x1D
#define UECP_CTR_DATA_SET_SELECT      0x1C
#define UECP_CTR_GROUP_SEQUENCE       0x16
#define UECP_CTR_GROUP_VAR_CODE_SEQ   0x29
#define UECP_CTR_EXTENDED_GROUP_SEQ   0x38
#define UECP_CTR_PS_CHAR_CODE_TBL_SEL 0x2F
#define UECP_CTR_ENCODER_ACCESS_RIGHT 0x3A
#define UECP_CTR_COM_PORT_CONF_MODE   0x3B
#define UECP_CTR_COM_PORT_CONF_SPEED  0x3C
#define UECP_CTR_COM_PORT_CONF_TMEOUT 0x3D

/// Other commands
#define UECP_OTHER_RASS               0xda

/// Bi-directional commands (Remote and configuration commands)
#define BIDIR_MESSAGE_ACKNOWLEDGMENT  0x18
#define BIDIR_REQUEST_MESSAGE         0x17

/// Specific message commands
#define SPEC_MFG_SPECIFIC_CMD         0x2D

/**
 * Open data application id's
 */

#define ODA_AID_RADIOTEXT_PLUS        0x4bd7
#define ODA_AID_TFC                   0xcd46


cRDSGroupDecoder::cRDSGroupDecoder(cRadioReceiver *proc)
  : m_RadioProc(proc)
{
  Reset();
}

void cRDSGroupDecoder::Reset()
{
  m_ProgramIdentCode = 0;
  m_RadioText_SegmentRegister  = 0;
  m_RadioText_Count = 0;
  m_RadioText_FirstPtr = false;
  m_DI = 0;
  m_DI_Prev = -1;
  m_MS = 0;
  m_MS_Prev = -1;
  m_PIN = -1;
  m_PTYN_SetFlag = 0;
  m_PS_Name[8] = 0;
  m_PS_SetFlag = 0;
  m_TA_TP = -1;
  m_RTPlus_Ready = false;

  m_AF_MethodStarted = false;
  m_AF_MethodFinished = false;
  m_AF_LowFreqFollow = false;
  m_AF_CalcCount = 0;
  m_AF_MethodAB = false;
  m_AF_AltFrequencies.clear();

  memset(m_RadioText_Temp, 0, sizeof(m_RadioText_Temp));
  memset(m_ODATypeMap, 0, sizeof(m_ODATypeMap));
  memset(m_PTYN, 0x20, sizeof(m_PTYN));
  memset(m_PS_Name, 0x20, sizeof(m_PS_Name));
}

void cRDSGroupDecoder::DecodeRDS(uint16_t *blockData)
{
  unsigned int group_type = (blockData[BLOCK__B] >> 11) & 0x1F;
  bool VersionCode = (blockData[BLOCK__B] >> 11) & 0x1;

  uint16_t pIdent = blockData[BLOCK__A] & 0xFFFF;                // "PI"
  if (pIdent != m_ProgramIdentCode)
    Decode_PI(pIdent);

  int pType = (blockData[BLOCK__B] >> 5) & 0x1F;            // "PTY"
  if (pType != m_PTY)
    Decode_PTY(pType);

  switch (group_type)
  {
    case RDS_GROUP_TYPE__0A:
    case RDS_GROUP_TYPE__0B:
      Decode_Type0___PS_DI_MS(blockData, VersionCode);
      break;
    case RDS_GROUP_TYPE__1A:
    case RDS_GROUP_TYPE__1B:
      Decode_Type1___ProgItemNum_SlowLabelCodes(blockData, VersionCode);
      break;
    case RDS_GROUP_TYPE__2A:
    case RDS_GROUP_TYPE__2B:
      Decode_Type2___Radiotext(blockData, VersionCode);
      break;
    case RDS_GROUP_TYPE__3A:
      Decode_Type3A__AppIdentOpenData(blockData);
      break;
    case RDS_GROUP_TYPE__4A:
      Decode_Type4A__Clock(blockData);
      break;
    case RDS_GROUP_TYPE__5A:
    case RDS_GROUP_TYPE__5B:
      if (m_ODATypeMap[group_type] > 0)
        Decode_Type____ODA(blockData, m_ODATypeMap[group_type]);
      else
        Decode_Type5___TransparentDataChannels(blockData, VersionCode);
      break;
    case RDS_GROUP_TYPE__6A:
    case RDS_GROUP_TYPE__6B:
      if (m_ODATypeMap[group_type] > 0)
        Decode_Type____ODA(blockData, m_ODATypeMap[group_type]);
      else
        Decode_Type6___InHouseApplications(blockData, VersionCode);
      break;
    case RDS_GROUP_TYPE__7A:
      if (m_ODATypeMap[group_type] > 0)
        Decode_Type____ODA(blockData, m_ODATypeMap[group_type]);
      else
        Decode_Type7A__RadioPaging(blockData);
      break;
    case RDS_GROUP_TYPE__8A:
      if (m_ODATypeMap[group_type] > 0)
        Decode_Type____ODA(blockData, m_ODATypeMap[group_type]);
      else
        Decode_Type8A__TrafficMessageChannel(blockData);
      break;
    case RDS_GROUP_TYPE__9A:
      if (m_ODATypeMap[group_type] > 0)
        Decode_Type____ODA(blockData, m_ODATypeMap[group_type]);
      else
        Decode_Type9A__EmergencyWarningSystem(blockData);
      break;
    case RDS_GROUP_TYPE_10A:
      Decode_Type10A_PTYN(blockData);
      break;
    case RDS_GROUP_TYPE_13A:
      if (m_ODATypeMap[group_type] > 0)
        Decode_Type____ODA(blockData, m_ODATypeMap[group_type]);
      else
        Decode_Type13A_EnhancedRadioPaging(blockData);
      break;
    case RDS_GROUP_TYPE_14A:
    case RDS_GROUP_TYPE_14B:
      Decode_Type14__EnhancedOtherNetworksInfo(blockData, VersionCode);
      break;
    case RDS_GROUP_TYPE_15A:
      Decode_Type15A_RBDS(blockData);
      break;
    case RDS_GROUP_TYPE_15B:
      Decode_Type15B_FastSwitchingInfo(blockData);
      break;
    case RDS_GROUP_TYPE__3B:
    case RDS_GROUP_TYPE__4B:
    case RDS_GROUP_TYPE__7B:
    case RDS_GROUP_TYPE__8B:
    case RDS_GROUP_TYPE__9B:
    case RDS_GROUP_TYPE_10B:
    case RDS_GROUP_TYPE_11A:
    case RDS_GROUP_TYPE_11B:
    case RDS_GROUP_TYPE_12A:
    case RDS_GROUP_TYPE_12B:
    case RDS_GROUP_TYPE_13B:
      if (m_ODATypeMap[group_type] > 0)
      {
        Decode_Type____ODA(blockData, m_ODATypeMap[group_type]);
        break;
      }
    default:
      break;
  }
}

void cRDSGroupDecoder::Decode_PI(uint16_t identifier)
{
  Reset();

  m_ProgramIdentCode = identifier;

  ClearUECPFrame();

  AddStuffingValue(UECP_RDS_PI);
  AddStuffingValue(0x00);
  AddStuffingValue(0x01);
  AddStuffingValue(identifier&0xff);
  AddStuffingValue((identifier>>8)&0xff);

  SendUECPFrame();
}

void cRDSGroupDecoder::Decode_PTY(int pty)
{
  if (pty >= 0 && pty < 32)
  {
    m_PTY = pty;

    ClearUECPFrame();

    AddStuffingValue(UECP_RDS_PTY);
    AddStuffingValue(0x00);
    AddStuffingValue(0x01);
    AddStuffingValue(pty&0xff);

    SendUECPFrame();
  }
}

/*!
 * 62106IEC:1999 - 3.1.5.1 Type 0 groups: Basic tuning and switching information
 * Load programme service name segment
 */
void cRDSGroupDecoder::Decode_Type0___PS_DI_MS(const uint16_t *msgElement, bool versionCode)
{
  static char ps_text[9]; ///< Temp storage to become full before published

  bool decoder_control_bit      = msgElement[BLOCK__B] & 0x04;  // "DI"
  unsigned char segment_address = msgElement[BLOCK__B] & 0x03;  // "DI segment"
  switch (segment_address)
  {
    case 3: ///< DI segment: d0
      if (decoder_control_bit)   m_DI |= 0x01;
      else if (m_DI & 0x01)      m_DI ^= 0x01;
      m_DI_Finished++;
      break;
    case 2: ///< DI segment: d1
      if (decoder_control_bit)   m_DI |= 0x02;
      else if (m_DI & 0x02)      m_DI ^= 0x02;
      m_DI_Finished++;
      break;
    case 1: ///< DI segment: d2
      if (decoder_control_bit)   m_DI |= 0x04;
      else if (m_DI & 0x04)      m_DI ^= 0x04;
      m_DI_Finished++;
      break;
    case 0: ///< DI segment: d3
      if (decoder_control_bit)   m_DI |= 0x08;
      else if (m_DI & 0x08)      m_DI ^= 0x08;
      m_DI_Finished++;
      break;
    default:
      break;
  }

  int TA_TP;
  TA_TP  = msgElement[BLOCK__B] & 0x10 ? 1 : 0;
  TA_TP |= msgElement[BLOCK__B] & 0x400 ? 2 : 0;
  if (m_TA_TP != TA_TP)
  {
    m_TA_TP = TA_TP;

    ClearUECPFrame();

    AddStuffingValue(UECP_RDS_TA_TP);
    AddStuffingValue(0x00);
    AddStuffingValue(0x01);
    AddStuffingValue(m_TA_TP);

    SendUECPFrame();
  }

  if (m_DI_Finished >= 4 && m_DI_Prev != m_DI)
  {
    m_DI_Finished = 0;
    m_DI_Prev = m_DI;

    ClearUECPFrame();

    AddStuffingValue(UECP_RDS_DI);
    AddStuffingValue(0x00);
    AddStuffingValue(0x01);
    AddStuffingValue(m_DI&0xf);

    SendUECPFrame();
  }
//
  m_MS = (msgElement[BLOCK__B] & 0x08) ? 1 : 0;
  if (m_MS_Prev != m_MS)
  {
    m_MS_Prev = m_MS;

    ClearUECPFrame();

    AddStuffingValue(UECP_RDS_MS);
    AddStuffingValue(0x00);
    AddStuffingValue(0x01);
    AddStuffingValue(m_MS);

    SendUECPFrame();
  }

  unsigned int textPtr = msgElement[BLOCK__B]&0x03;
  ps_text[textPtr*2]   = (msgElement[BLOCK__D]>>8)&0xff;
  ps_text[textPtr*2+1] = msgElement[BLOCK__D]&0xff;
  m_PS_SetFlag |= 1<<textPtr;

  if (m_PS_SetFlag == 0x0F)
  {
    if (m_RadioProc->IsSettingActive() || (memcmp(m_PS_Name, ps_text, 8)) != 0)
    {
      if (m_RadioProc->SetChannelName(ps_text))
      {
        ClearUECPFrame();

        AddStuffingValue(UECP_RDS_PS);
        AddStuffingValue(0x00);
        AddStuffingValue(0x01);
        for (int i = 0; i < 8; i++)
          AddStuffingValue(ps_text[i]);

        SendUECPFrame();

        memcpy(m_PS_Name, ps_text, 8);
      }

      m_PS_SetFlag = 0;
    }
  }

  /*!>
   * Scan alternative Frequencies on type 0A
   */
#ifdef IMPROVE_CHECK_ALT_FREQ
  if (!versionCode)
  {
    uint8_t AFCode[2];
    AFCode[1] = msgElement[BLOCK__C]&0xff;
    AFCode[0] = (msgElement[BLOCK__C]>>8)&0xff;

    if (AFCode[0] >= 224 && AFCode[0] <= 249)
    {
      m_AF_LowFreqFollow = false;
      if (m_AF_MethodStarted && !m_AF_MethodFinished)
      {
        ClearUECPFrame();

        AddStuffingValue(UECP_RDS_AF);
        AddStuffingValue(0x00);
        AddStuffingValue(0x01);

        if (AFCode[0] == 224)
        {
          fprintf(stderr, "Radio RDS/RaSS Processor - No alternative frequencies exists\n");
          AddStuffingValue(3);
          AddStuffingValue(0x00);
          AddStuffingValue(0x00);
          AddStuffingValue(0x00);
        }
        else
        {
          fprintf(stderr, "Radio RDS/RaSS Processor - %i alternative frequencies exists (%i entries)\n", (int)m_AF_AltFrequencies.size(), AFCode[0] - 224);
          AddStuffingValue(m_AF_AltFrequencies.size()+3);

          for (unsigned int i = 0; i < m_AF_AltFrequencies.size(); i++)
          {
            string variant;
            if (m_AF_MethodAB == true && i == 0)
              variant = "(tuning frequency)";
            else
              variant = m_AF_AltFrequencies[i].Regional ? "(regional variant)" : "(same programme)";
            if (m_AF_AltFrequencies[i].Frequency > 87500000.0f)
              fprintf(stderr, " - FM %.01f MHz %s\n", m_AF_AltFrequencies[i].Frequency / 1000000.0f, variant.c_str());
            else if (m_AF_AltFrequencies[i].Frequency > 531000.0f)
              fprintf(stderr, " - MF %.01f ḱHz %s\n", m_AF_AltFrequencies[i].Frequency / 1000.0f, variant.c_str());
            else if (m_AF_AltFrequencies[i].Frequency > 153000.0f)
              fprintf(stderr, " - LF %.01f ḱHz %s\n", m_AF_AltFrequencies[i].Frequency / 1000.0f, variant.c_str());

            AddStuffingValue(m_AF_AltFrequencies[i].Value);
          }

          AddStuffingValue(0x00);
        }
        m_AF_MethodFinished = true;
      }
      m_AF_MethodStarted = true;
    }
    if (m_AF_MethodStarted)
    {
      for (int i = 0; i <= 1; i++)
      {
        if (AFCode[i] == 205)
          continue;
        else if (AFCode[i] == 250)
          m_AF_LowFreqFollow = true;
        else if (AFCode[i] <= 204)
        {
          float freq = 0.0f;
          if (!m_AF_LowFreqFollow) // FM
            freq = (AFCode[i] * 100000.0f) + 87500000.0f;
          else if (AFCode[i] <= 15) // LF
            freq = (AFCode[i] * 9000.0f) + 153000.0f - 9000.0f;
          else if (AFCode[i] <= 135) // MF
            freq = (AFCode[i] * 9000.0f) + 531000.0f - 9000.0f;

          if (freq > 0.0f)
          {
            if (!m_AF_MethodFinished)
            {
              m_AF_CalcCount++;

              sAFMethod method;
              method.Frequency = freq;
              method.Regional = false;
              method.Value = AFCode[i];
              if (m_AF_AltFrequencies.size() > 0 && m_AF_AltFrequencies[0].Frequency == freq)
              {
                m_AF_MethodAB = true;
                if ((m_AF_CalcCount % 2) != 0)
                  m_AF_AltFrequencies[m_AF_AltFrequencies.size()-1].Regional = true;
              }
              else
                m_AF_AltFrequencies.push_back(method);
            }
            else
            {
              bool present = false;
              for (unsigned int i = 0; i < m_AF_AltFrequencies.size(); i++)
              {
                if (m_AF_AltFrequencies[i].Frequency == freq)
                {
                  present = true;
                  break;
                }
              }
              if (!present)
              {
                m_AF_MethodStarted  = false;
                m_AF_MethodFinished = false;
                m_AF_CalcCount      = 0;
                m_AF_MethodAB       = false;
                m_AF_AltFrequencies.clear();

                fprintf(stderr, "Radio RDS/RaSS Processor - Change in alternative frequencies, restarted scan\n");
              }
            }
          }
        }
      }
    }
  }
#endif // IMPROVE_CHECK_ALT_FREQ
}

void cRDSGroupDecoder::Decode_Type1___ProgItemNum_SlowLabelCodes(const uint16_t *msgElement, bool VersionABCode)
{
#ifdef IMPROVE_CHECK2
  uint8_t radioPagingCodes = msgElement[BLOCK__B]&0x1f;
  if (radioPagingCodes)
    printf("paging codes: %i ", (int)radioPagingCodes);
#endif // IMPROVE_CHECK

  if (m_PIN != msgElement[BLOCK__D])
  {
    m_PIN = msgElement[BLOCK__D];

    ClearUECPFrame();
    AddStuffingValue(UECP_RDS_PIN);
    AddStuffingValue(0x00);
    AddStuffingValue(0x01);
    AddStuffingValue((m_PIN>>8)&0xff);
    AddStuffingValue( m_PIN    &0xff);
    SendUECPFrame();
  }

  if (!VersionABCode)
  {
    ClearUECPFrame();
    AddStuffingValue(UECP_SLOW_LABEL_CODES);
    AddStuffingValue(0x00);
    AddStuffingValue((msgElement[BLOCK__C]>>8)&0x7F);
    AddStuffingValue( msgElement[BLOCK__C]    &0xff);
    SendUECPFrame();
#ifdef IMPROVE_CHECK2
    bool linkageActuator = (msgElement[BLOCK__C]>>15)&0x1;
#endif // IMPROVE_CHECK
  }
}

/*!
 * 62106IEC:1999 - 3.1.5.6 Type 2 groups:  RadioText
 */
void cRDSGroupDecoder::Decode_Type2___Radiotext(const uint16_t *element, bool VersionCode)
{
  unsigned int textPtr = element[1]&0x0f;

  m_RTPlus_Ready = false;

  if (textPtr == 0 && m_RadioText_FirstPtr && m_RadioText_Count > 1)
  {
    bool radioText_Ready = true;
    for (int i = 0; i < m_RadioText_Count; i++)
    {
      if (!(m_RadioText_SegmentRegister & (1<<i)))
      {
        radioText_Ready = false;
        m_RadioText_SegmentRegister = 0;
        m_RadioText_Count = 0;
        break;
      }
    }
    if (radioText_Ready)
    {
      ClearUECPFrame();

      AddStuffingValue(UECP_RDS_RT);
      AddStuffingValue(0x00);
      AddStuffingValue(0x01);
      AddStuffingValue(65);
      AddStuffingValue(m_RadioText_ABFlag);

      for (int i = 0; i < 64; i++)
        AddStuffingValue(m_RadioText_Temp[i]);

      SendUECPFrame();
      m_RTPlus_Ready = true;
    }
  }

  /* when the A/B flag is toggled, flush your current radiotext */
  if (m_RadioText_ABFlag != ((element[1]>>4)&0x01))
  {
    memset(m_RadioText_Temp, 0x20, RT_MEL);

    m_RadioText_ABFlag = (element[1]>>4)&0x01;
    m_RadioText_FirstPtr = false;
    m_RadioText_SegmentRegister = 0;
    m_RadioText_Count = 0;
  }

  if (!VersionCode)
  {
    m_RadioText_Temp[textPtr*4]   = (element[2]>>8)&0xff;
    m_RadioText_Temp[textPtr*4+1] = element[2]&0xff;
    m_RadioText_Temp[textPtr*4+2] = (element[3]>>8)&0xff;
    m_RadioText_Temp[textPtr*4+3] = element[3]&0xff;
  }
  else
  {
    m_RadioText_Temp[textPtr*2]   = (element[3]>>8)&0xff;
    m_RadioText_Temp[textPtr*2+1] = element[3]&0xff;
  }

  m_RadioText_SegmentRegister |= 1 << textPtr;
  m_RadioText_Count++;

  if (!m_RadioText_FirstPtr && textPtr == 0)
    m_RadioText_FirstPtr = true;
}

/*!
 * 62106IEC:1999 - 3.1.5.4 Type 3A groups: Application identification for Open data
 */
void cRDSGroupDecoder::Decode_Type3A__AppIdentOpenData(const uint16_t *msgElement)
{
  int  application_group = (msgElement[BLOCK__B]>>1)&0xf;  //!< Application Group Type Code A0-A3
  bool group_type        = msgElement[BLOCK__B]&0x1;       //!<       "        "    "    "  B0
  int  message           = msgElement[BLOCK__C];           //!<  Message bits
  int  aid               = msgElement[BLOCK__D];           //!<  Application Identification (AID)

  ClearUECPFrame();
  AddStuffingValue(UECP_ODA_CONF_SHORT_MSG_CMD);
  AddStuffingValue(msgElement[BLOCK__B]&0x1F);             //!< Application Group Type Code
  AddStuffingValue((msgElement[BLOCK__D]>>8)&0xFF);        //!<  Application Identification (AID)
  AddStuffingValue( msgElement[BLOCK__D]    &0xFF);
  AddStuffingValue(0);
  AddStuffingValue((msgElement[BLOCK__C]>>8)&0xFF);        //!<  Message bits
  AddStuffingValue( msgElement[BLOCK__C]    &0xFF);
  AddStuffingValue(0);
  SendUECPFrame();

  switch (aid)
  {
    case ODA_AID_RADIOTEXT_PLUS:
    {
      m_ODATypeMap[msgElement[BLOCK__B]&0x1F] = ODA_AID_RADIOTEXT_PLUS;
      m_RTPlus_templateNo = msgElement[BLOCK__C]&0xFF;
      m_RTPlus_scb        = (msgElement[BLOCK__C]>>8)&0xF;
      m_RTPlus_cbflag     = (msgElement[BLOCK__C]>>12)&0x1;
      m_RTPlus_rfu        = (msgElement[BLOCK__C]>>13)&0x7;
      break;
    }
    case ODA_AID_TFC:
      m_ODATypeMap[msgElement[BLOCK__B]&0x1F] = ODA_AID_TFC;
      break;
    default:
      m_ODATypeMap[msgElement[BLOCK__B]&0x1F] = 0;
#ifdef IMPROVE_CHECK
      printf("NOT SUPPORTED! >> aid group: %i%c - message: %04X - aid: %04X\n", application_group, group_type?'B':'A', message, aid);
#endif
      break;
  }
}

/*!
 * 62106IEC:1999 - 3.1.5.6 Type 4A groups: Clock-time and date
 */
void cRDSGroupDecoder::Decode_Type4A__Clock(const uint16_t *msgElement)
{
  double modified_julian_date = ((msgElement[BLOCK__B]&0x03)<<15) | ((msgElement[BLOCK__C]>>1)&0x7fff);

  unsigned int hours   = ((msgElement[BLOCK__C] & 0x01)<<4) | ((msgElement[BLOCK__D]>>12) & 0x0f);
  unsigned int minutes = (msgElement[BLOCK__D]>>6) & 0x3f;
  int          offset  = msgElement[BLOCK__D] & 0x3f;
  unsigned int year    = (int)((modified_julian_date-15078.2) / 365.25);
  unsigned int month   = (int)((modified_julian_date-14956.1 - (int)(year*365.25)) / 30.6001);
  unsigned int day     = modified_julian_date - 14956 - (int)(year*365.25) - (int)(month*30.6001);
  int          K       = ((month==14) || (month==15)) ? 1 : 0;
  year  += K + 1900;
  month -= 1 + K * 12;

  ClearUECPFrame();

  AddStuffingValue(UECP_CLOCK_RTC);
  AddStuffingValue(year % 100);
  AddStuffingValue(month);
  AddStuffingValue(day);
  AddStuffingValue(hours);
  AddStuffingValue(minutes);
  AddStuffingValue(0);
  AddStuffingValue(0);
  AddStuffingValue(offset);

  SendUECPFrame();
}

/*!
 * 62106IEC:1999 - 3.1.5.8 Type 5 groups: Transparent Data Channels (32 channels)
 */
void cRDSGroupDecoder::Decode_Type5___TransparentDataChannels(const uint16_t *msgElement, bool versionCode)
{
#ifdef IMPROVE_CHECK2
  unsigned int group_type = (unsigned int)((msgElement[BLOCK__B]>>12)&0xf);
  printf("Radio RDS/RaSS Processor %s - RDS group type: %u%c - 04%X 04%X 04%X 04%X\n", __FUNCTION__, group_type, versionCode ? 'A' : 'B', msgElement[0], msgElement[BLOCK__B], msgElement[BLOCK__C], msgElement[BLOCK__D]);
#endif
}

/*!
 * 62106IEC:1999 - 3.1.5.9 Type 6 groups: In House applications
 * Consumer receivers shall ignore the in-house information coded in these groups.
 */
void cRDSGroupDecoder::Decode_Type6___InHouseApplications(const uint16_t *msgElement, bool versionCode)
{
  (void)msgElement;
  (void)versionCode;
}

/*!
 * 62106IEC:1999 - 3.1.5.10 Type 7A groups: Radio Paging
 */
void cRDSGroupDecoder::Decode_Type7A__RadioPaging(const uint16_t *msgElement)
{
#ifdef IMPROVE_CHECK
  unsigned int group_type = (unsigned int)((msgElement[BLOCK__B]>>12)&0xf);
  printf("Radio RDS/RaSS Processor %s - RDS group type: %uA - 04%X 04%X 04%X 04%X\n", __FUNCTION__, group_type, msgElement[0], msgElement[BLOCK__B], msgElement[BLOCK__C], msgElement[BLOCK__D]);
#endif
}

/*!
 * 62106IEC:1999 - 3.1.5.12 Type 8A groups: Traffic Message Channel
 */
void cRDSGroupDecoder::Decode_Type8A__TrafficMessageChannel(const uint16_t *msgElement)
{
  ClearUECPFrame();

  AddStuffingValue(UECP_TDC_TMC);
  AddStuffingValue(6);
  AddStuffingValue(0);
  AddStuffingValue(msgElement[BLOCK__B]&0x1F);
  AddStuffingValue((msgElement[BLOCK__C]>>8)&0xFF);
  AddStuffingValue( msgElement[BLOCK__C]    &0xFF);
  AddStuffingValue((msgElement[BLOCK__D]>>8)&0xFF);
  AddStuffingValue( msgElement[BLOCK__D]    &0xFF);

  SendUECPFrame();
}

/*!
 * 62106IEC:1999 - 3.1.5.13 Type 9A groups: Emergency Warning System
 */
void cRDSGroupDecoder::Decode_Type9A__EmergencyWarningSystem(const uint16_t *msgElement)
{
#ifdef IMPROVE_CHECK
  unsigned int group_type = (unsigned int)((msgElement[BLOCK__B]>>12)&0xf);
  printf("Radio RDS/RaSS Processor %s - RDS group type: %uA - 04%X 04%X 04%X 04%X\n", __FUNCTION__, group_type, msgElement[0], msgElement[BLOCK__B], msgElement[BLOCK__C], msgElement[BLOCK__D]);
#endif
}

/*!
 * 62106IEC:1999 - 3.1.5.14 Type 10A groups: Programme Type Name
 */
void cRDSGroupDecoder::Decode_Type10A_PTYN(const uint16_t *msgElement)
{
  unsigned int textPtr = msgElement[BLOCK__B]&0x01;

  /* when the A/B flag is toggled, flush your current radiotext */
  if (m_PTYN_ABFlag != ((msgElement[BLOCK__B]>>4)&0x01))
  {
    memset(m_PTYN, 0x20, 8);

    m_PTYN_ABFlag = (msgElement[BLOCK__B]>>4)&0x01;
    m_PTYN_SetFlag = 0;
  }

  m_PTYN[textPtr*4]   = (msgElement[BLOCK__C]>>8)&0xff;
  m_PTYN[textPtr*4+1] = msgElement[BLOCK__C]&0xff;
  m_PTYN[textPtr*4+2] = (msgElement[BLOCK__D]>>8)&0xff;
  m_PTYN[textPtr*4+3] = msgElement[BLOCK__D]&0xff;
  m_PTYN_SetFlag     |= 1<<textPtr;

  if (m_PTYN_SetFlag & 3)
  {
    ClearUECPFrame();

    AddStuffingValue(UECP_RDS_PTYN);
    AddStuffingValue(0x00);
    AddStuffingValue(0x01);

    for (unsigned int i = 0; i < 8; i++)
      AddStuffingValue(m_PTYN[i]);

    SendUECPFrame();
  }
}

/*!
 * 62106IEC:1999 - Annex M Type 13A groups: Enhanced Radio Paging
 * @todo check presence and usage of it
 */
void cRDSGroupDecoder::Decode_Type13A_EnhancedRadioPaging(const uint16_t *msgElement)
{
#ifdef IMPROVE_CHECK2
  unsigned int group_type = (unsigned int)((msgElement[BLOCK__B]>>12)&0xf);
  printf("Radio RDS/RaSS Processor %s - RDS group type: %uA - 04%X 04%X 04%X 04%X\n", __FUNCTION__, group_type, msgElement[0], msgElement[BLOCK__B], msgElement[BLOCK__C], msgElement[BLOCK__D]);
#endif
}

/*!
 * 62106IEC:1999 - 3.1.5.19 Type 14 groups: Enhanced Other Networks information only
 * @todo Is present on several channels
 */
void cRDSGroupDecoder::Decode_Type14__EnhancedOtherNetworksInfo(const uint16_t *msgElement, bool versionCode)
{
#ifdef IMPROVE_CHECK2
  unsigned int group_type = (unsigned int)((msgElement[BLOCK__B]>>12)&0xf);
  printf("Radio RDS/RaSS Processor %s - RDS group type: %uA - 04%X 04%X 04%X 04%X\n", __FUNCTION__, group_type, msgElement[0], msgElement[BLOCK__B], msgElement[BLOCK__C], msgElement[BLOCK__D]);
#endif
}

/*!
 * 62106IEC:1999 - Type 15A groups: Defined in RBDS
 * @todo not seen
 */
void cRDSGroupDecoder::Decode_Type15A_RBDS(const uint16_t *msgElement)
{
#ifdef IMPROVE_CHECK2
  unsigned int group_type = (unsigned int)((msgElement[BLOCK__B]>>12)&0xf);
  printf("Radio RDS/RaSS Processor %s - RDS group type: %uA - 04%X 04%X 04%X 04%X\n", __FUNCTION__, group_type, msgElement[0], msgElement[BLOCK__B], msgElement[BLOCK__C], msgElement[BLOCK__D]);
#endif
}

/*!
 * 62106IEC:1999 - 3.1.5.20 Type 15B groups: Fast switching information only
 * @todo Is present on several channels
 */
void cRDSGroupDecoder::Decode_Type15B_FastSwitchingInfo(const uint16_t *msgElement)
{
#ifdef IMPROVE_CHECK2
  unsigned int group_type = (unsigned int)((msgElement[BLOCK__B]>>12)&0xf);
  printf("Radio RDS/RaSS Processor %s - RDS group type: %uA - 04%X 04%X 04%X 04%X\n", __FUNCTION__, group_type, msgElement[0], msgElement[BLOCK__B], msgElement[BLOCK__C], msgElement[BLOCK__D]);
#endif
}

/*!
 * @todo Check right of traffic message
 */
void cRDSGroupDecoder::Decode_Type____ODA(const uint16_t *msgElement, int odaFunction)
{
  switch (odaFunction)
  {
    case ODA_AID_RADIOTEXT_PLUS:
      if (m_RTPlus_Ready)
      {
        ClearUECPFrame();
        AddStuffingValue(UECP_ODA_DATA);
        AddStuffingValue(8);
        AddStuffingValue(0x4b);
        AddStuffingValue(0xd7);
        AddStuffingValue((msgElement[BLOCK__B]>>8)&0xFF);
        AddStuffingValue( msgElement[BLOCK__B]    &0xFF);
        AddStuffingValue((msgElement[BLOCK__C]>>8)&0xFF);
        AddStuffingValue( msgElement[BLOCK__C]    &0xFF);
        AddStuffingValue((msgElement[BLOCK__D]>>8)&0xFF);
        AddStuffingValue( msgElement[BLOCK__D]    &0xFF);
        SendUECPFrame();

        m_RTPlus_Ready = false;
      }
      break;
    case ODA_AID_TFC:
      ClearUECPFrame();
      AddStuffingValue(UECP_ODA_DATA);
      AddStuffingValue(7);
      AddStuffingValue(0xcd);
      AddStuffingValue(0x46);
      AddStuffingValue( msgElement[BLOCK__B]    &0xFF);
      AddStuffingValue((msgElement[BLOCK__C]>>8)&0xFF);
      AddStuffingValue( msgElement[BLOCK__C]    &0xFF);
      AddStuffingValue((msgElement[BLOCK__D]>>8)&0xFF);
      AddStuffingValue( msgElement[BLOCK__D]    &0xFF);
      SendUECPFrame();
      break;
    default:
      break;
  }
}

void cRDSGroupDecoder::ClearUECPFrame()
{
  memset(m_UECPDataFrame+4, 0xFF, sizeof(m_UECPDataFrame)-4); //!< STP
  m_UECPDataFrame[0] = 0xFE;                          //!< STA
  m_UECPDataFrame[1] = 0;                             //!< ADD
  m_UECPDataFrame[2] = 0;                             //!<  "
  m_UECPDataFrame[3] = m_UECPDataFrameSeqCnt++;       //!< SQC
  m_UECPDataFrame[4] = 0;                             //!< MFL

  m_UECPStuffingPtr = 0;
}

void cRDSGroupDecoder::SendUECPFrame()
{
  if (m_RadioProc->IsSettingActive())
    return;

  m_UECPDataFrame[4] = m_UECPStuffingPtr;
  uint16_t crc = crc16_ccitt(m_UECPDataFrame, m_UECPStuffingPtr+4);
  m_UECPDataFrame[5+m_UECPStuffingPtr+0] = (crc>>8)&0xff;
  m_UECPDataFrame[5+m_UECPStuffingPtr+1] = crc & 0xff;
  m_RadioProc->AddUECPDataFrame(m_UECPDataFrame);
}

uint16_t cRDSGroupDecoder::crc16_ccitt(const uint8_t *data, int len, bool skipfirst)
{
  // CRC16-CCITT: x^16 + x^12 + x^5 + 1
  // with start 0xffff and result invers
  register uint16_t crc = 0xffff;

  if (skipfirst)
    data++;

  while (len--)
  {
    crc = (crc >> 8) | (crc << 8);
    crc ^= *data++;
    crc ^= (crc & 0xff) >> 4;
    crc ^= (crc << 8) << 4;
    crc ^= ((crc & 0xff) << 4) << 1;
  }

  return ~(crc);
}

void cRDSGroupDecoder::AddStuffingValue(uint8_t value)
{
  if (m_UECPStuffingPtr > 255)
  {
    KODI->Log(LOG_ERROR, "value reach end of allowed UECP frame size");
    return;
  }

  if (value < 0xFD)
  {
    m_UECPDataFrame[5+m_UECPStuffingPtr++] = value;
  }
  else
  {
    m_UECPDataFrame[5+m_UECPStuffingPtr++] = 0xFD;
    m_UECPDataFrame[5+m_UECPStuffingPtr++] = value^0xFD;
  }
}
