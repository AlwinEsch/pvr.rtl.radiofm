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
 */

#include "p8-platform/util/StringUtils.h"

#include "RadioReceiver.h"
#include "ChannelSettings.h"
#include "FmDecode.h"

using namespace ADDON;
using namespace P8PLATFORM;

#define DVD_TIME_BASE 1000000
#define DVD_NOPTS_VALUE    (-1LL<<52) // should be possible to represent in both double and int64_t

#define DVD_TIME_TO_SEC(x)  ((int)((double)(x) / DVD_TIME_BASE))
#define DVD_TIME_TO_MSEC(x) ((int)((double)(x) * 1000 / DVD_TIME_BASE))
#define DVD_SEC_TO_TIME(x)  ((double)(x) * DVD_TIME_BASE)
#define DVD_MSEC_TO_TIME(x) ((double)(x) * DVD_TIME_BASE / 1000)

cRadioReceiver::cRadioReceiver()
  : m_RtlSdrReceiver(this)
{
  m_strDefaultIcon  = "";
  m_activeIndex     = -1;
  m_DeviceIndex     = -1;
  m_IfRate          = 1.0e6;
  m_PCMRate         = OUTPUT_SAMPLERATE;
  m_LnaGain         = INT_MIN;
  m_AgcMode         = false;
  m_StreamChange    = false;
  m_FMDecoder       = NULL;
  m_StreamActive    = false;
  m_SettingsDialog  = NULL;

  LoadChannelData(true);
}

cRadioReceiver::~cRadioReceiver()
{
  CloseChannel();

  m_channels.clear();
}

std::string cRadioReceiver::CreateChannelName(FMRadioChannel &channel) const
{
  std::string name;
  if (!channel.strChannelName.empty() && channel.strChannelName != "-")
    name = StringUtils::Trim(channel.strChannelName);
  else if (channel.fChannelFreq >= 87500000.0f)
    name = StringUtils::Format("FM %.01f MHz", channel.strChannelName.c_str(), channel.fChannelFreq / 1000000.0f);
  else if (channel.fChannelFreq >= 531000.0f)
    name = StringUtils::Format("MF %.01f ḱHz", channel.strChannelName.c_str(), channel.fChannelFreq / 1000.0f);
  else if (channel.fChannelFreq >= 153000.0f)
    name = StringUtils::Format("LF %.01f ḱHz", channel.strChannelName.c_str(), channel.fChannelFreq / 1000.0f);

  return name;
}

int cRadioReceiver::GetChannelsAmount(void)
{
  return m_channels.size();
}

PVR_ERROR cRadioReceiver::DeleteChannel(const PVR_CHANNEL &channel)
{
  for (int iChannelPtr = 0; iChannelPtr < (int)m_channels.size(); iChannelPtr++)
  {
    if (m_channels[iChannelPtr].iUniqueId == channel.iUniqueId)
    {
      if (iChannelPtr == m_activeIndex)
        return PVR_ERROR_REJECTED;
      m_channels.erase(m_channels.begin()+iChannelPtr);
      SaveChannelData();
      break;
    }
  }
  return PVR_ERROR_NO_ERROR;
}

PVR_ERROR cRadioReceiver::GetChannels(ADDON_HANDLE handle)
{
  for (unsigned int iChannelPtr = 0; iChannelPtr < m_channels.size(); iChannelPtr++)
  {
    FMRadioChannel &channel = m_channels.at(iChannelPtr);
    PVR_CHANNEL kodiChannel;
    memset(&kodiChannel, 0, sizeof(PVR_CHANNEL));
    std::string name = CreateChannelName(channel);

    kodiChannel.iUniqueId         = channel.iUniqueId;
    kodiChannel.bIsRadio          = true;
    strncpy(kodiChannel.strChannelName, name.c_str(), sizeof(kodiChannel.strChannelName) - 1);
    strncpy(kodiChannel.strIconPath, channel.strIconPath.c_str(), sizeof(kodiChannel.strIconPath) - 1);
    kodiChannel.bIsHidden         = false;

    PVR->TransferChannelEntry(handle, &kodiChannel);
  }

  return PVR_ERROR_NO_ERROR;
}

bool cRadioReceiver::SetChannel(const PVR_CHANNEL &channel)
{
  m_activeIndex = -1;
  for (unsigned int iChannelPtr = 0; iChannelPtr < m_channels.size(); iChannelPtr++)
  {
    if (m_channels[iChannelPtr].iUniqueId == channel.iUniqueId)
    {
      float freq = m_channels[iChannelPtr].fChannelFreq;
      if (freq < 87500000.0f || freq > 108000000.0f)
      {
        KODI->Log(LOG_ERROR, "Used frequency %f0.2 MHz not allowed", freq / 1000.0f / 1000.0f);
        return false;
      }

      m_activeChannelInfo       = channel;
      m_activeIndex             = iChannelPtr;
      m_activeChannelFrequency  = freq + 0.10 * m_IfRate; //!< Intentionally tune at a higher frequency to avoid DC offset.
      break;
    }
  }

  if (m_activeIndex == -1)
  {
    KODI->Log(LOG_ERROR, "channel '%s' unknown", channel.strChannelName);
    return false;
  }

  m_activeChannelFrequency = m_channels[m_activeIndex].fChannelFreq;
  return true;
}

bool cRadioReceiver::OpenChannel(const PVR_CHANNEL &channel)
{
  KODI->Log(LOG_INFO, "Starting usage of software decoder for AM, FM and DAB+ broadcast radio with RTL-SDR");

  m_activeIndex = -1;
  for (unsigned int iChannelPtr = 0; iChannelPtr < m_channels.size(); iChannelPtr++)
  {
    if (m_channels[iChannelPtr].iUniqueId == channel.iUniqueId)
    {
      float freq = m_channels[iChannelPtr].fChannelFreq;
      if (freq < 87500000.0f || freq > 108000000.0f)
      {
        KODI->Log(LOG_ERROR, "Used frequency %f0.2 MHz not allowed", freq / 1000.0f / 1000.0f);
        return false;
      }

      m_activeChannelInfo       = channel;
      m_activeIndex             = iChannelPtr;
      m_activeChannelFrequency  = freq + 0.10 * m_IfRate; //!< Intentionally tune at a higher frequency to avoid DC offset.
      break;
    }
  }

  if (!SetChannel(channel))
    return false;

  m_activeIndex                 = -1;
  m_DeviceIndex                 = -1;
  m_IfRate                      = 1.0e6;
  m_PCMRate                     = OUTPUT_SAMPLERATE;
  m_LnaGain                     = INT_MIN;
  m_AgcMode                     = true;
  m_AudioLevel                  = 0.0f;
  m_AudioSourceEndMarked        = false;
  m_AudioSourceSize             = 0;
  m_AudioSourceBufferWarning    = false;

  std::vector<std::string> devnames;
  if (!cRtlSdrSource::GetDeviceNames(devnames))
  {
    KODI->Log(LOG_ERROR, "No devices present");
    return false;
  }

  if (m_DeviceIndex < 0 || (unsigned int)m_DeviceIndex >= devnames.size())
  {
    if (m_DeviceIndex != -1)
      KODI->Log(LOG_ERROR, "invalid device index %d", m_DeviceIndex);

    KODI->Log(LOG_INFO, "Found %u device(s):", (unsigned int)devnames.size());
    for (unsigned int i = 0; i < devnames.size(); i++)
      KODI->Log(LOG_INFO, " %2u: %s", i, devnames[i].c_str());
    m_DeviceIndex = 0;
  }
  KODI->Log(LOG_INFO, "Using device %d: %s", m_DeviceIndex, devnames[m_DeviceIndex].c_str());

  //! Intentionally tune at a higher frequency to avoid DC offset.
  m_activeTunerFreq = m_activeChannelFrequency + 0.25 * m_IfRate;

  //! Open RTL-SDR device.
  if (!m_RtlSdrReceiver.OpenDevice(m_DeviceIndex))
  {
    KODI->Log(LOG_ERROR, "Device open failed: %s", m_RtlSdrReceiver.error().c_str());
    return false;
  }

  //! Check LNA gain.
  if (m_LnaGain != INT_MIN)
  {
    std::vector<int> gains;
    m_RtlSdrReceiver.GetTunerGains(gains);
    if (find(gains.begin(), gains.end(), m_LnaGain) == gains.end())
    {
      if (m_LnaGain != INT_MIN + 1)
        KODI->Log(LOG_ERROR, "LNA gain %.1f dB not supported by tuner", m_LnaGain * 0.1);
      KODI->Log(LOG_INFO, "  Supported LNA gains:");
      for (int i = 0; gains.size(); i++)
        KODI->Log(LOG_INFO, " - %.1f dB", 0.1 * gains[i]);
      return false;
    }
  }

  //! Configure RTL-SDR device and start streaming.
  bool ret = m_RtlSdrReceiver.Configure(m_IfRate,
                                 m_activeTunerFreq,
                                 m_LnaGain,
                                 cRtlSdrSource::default_block_length,
                                 m_AgcMode);
  if (!ret)
  {
    KODI->Log(LOG_ERROR, "Device configuration failed: %s", m_RtlSdrReceiver.error().c_str());
    return false;
  }

  m_activeTunerFreq = m_RtlSdrReceiver.GetFrequency();
  KODI->Log(LOG_INFO, "device tuned for:  %.6f MHz", m_activeTunerFreq * 1.0e-6);

  if (m_LnaGain == INT_MIN)
    KODI->Log(LOG_INFO, "LNA gain:          auto");
  else
    KODI->Log(LOG_INFO, "LNA gain:          %.1f dB", 0.1 * m_RtlSdrReceiver.GetTunerGain());

  m_IfRate = m_RtlSdrReceiver.GetSampleRate();
  KODI->Log(LOG_INFO, "IF sample rate:    %.0f", m_IfRate);
  KODI->Log(LOG_INFO, "RTL AGC mode:      %s", m_AgcMode ? "enabled" : "disabled");

  // The baseband signal is empty above 100 kHz, so we can
  // downsample to ~ 200 kS/s without loss of information.
  // This will speed up later processing stages.
  unsigned int downsample = std::max(1, int(m_IfRate / 215.0e3));
  KODI->Log(LOG_INFO, "baseband downsampling factor %u", downsample);

  // Prevent aliasing at very low output sample rates.
  double bandwidth_pcm = std::min(DEFAULT_BANDWIDTH_PCM, 0.45 * m_PCMRate);
  KODI->Log(LOG_INFO, "audio sample rate: %.0f", m_PCMRate);
  KODI->Log(LOG_INFO, "audio bandwidth:   %.3f kHz", bandwidth_pcm * 1.0e-3);

  CLockObject lock(m_AudioSignalMutex);

  // Prepare decoder.
  m_FMDecoder = new cFmDecoder(this, m_IfRate,                            // sample_rate_if
                               m_activeChannelFrequency - m_activeTunerFreq,                 // tuning_offset
                               m_PCMRate,                           // sample_rate_pcm
                               bandwidth_pcm,                     // bandwidth_pcm
                               downsample);                       // downsample

  m_streams.iStreamCount = 0;
  CodecDescriptor codecId = CodecDescriptor::GetCodecByName("pcm_f32le");
  if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_AUDIO)
  {
    m_streams.stream[0].iPID            = 1;
    m_streams.stream[0].iCodecType      = codecId.Codec().codec_type;
    m_streams.stream[0].iCodecId        = codecId.Codec().codec_id;
    m_streams.stream[0].iChannels       = 2;
    m_streams.stream[0].iSampleRate     = OUTPUT_SAMPLERATE;
    m_streams.stream[0].iBitsPerSample  = 32;
    m_streams.stream[0].iBitRate        = m_streams.stream[0].iSampleRate * m_streams.stream[0].iChannels * m_streams.stream[0].iBitsPerSample;
    m_streams.stream[0].strLanguage[0]  = 0;
    m_streams.stream[0].strLanguage[1]  = 0;
    m_streams.stream[0].strLanguage[2]  = 0;
    m_streams.stream[0].strLanguage[3]  = 0;

    m_streams.iStreamCount++;
  }
  else
  {
    m_streams.iStreamCount = 0;
    return false;
  }

  codecId = CodecDescriptor::GetCodecByName("rds");
  if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_RDS)
  {
    m_streams.stream[1].iPID            = 2;
    m_streams.stream[1].iCodecType      = codecId.Codec().codec_type;
    m_streams.stream[1].iCodecId        = codecId.Codec().codec_id;
    m_streams.stream[1].iChannels       = 2;
    m_streams.stream[1].iSampleRate     = OUTPUT_SAMPLERATE;
    m_streams.stream[1].iBitsPerSample  = 32;
    m_streams.stream[1].iBitRate        = m_streams.stream[1].iSampleRate * m_streams.stream[1].iChannels * m_streams.stream[1].iBitsPerSample;
    m_streams.stream[1].strLanguage[0]  = 0;
    m_streams.stream[1].strLanguage[1]  = 0;
    m_streams.stream[1].strLanguage[2]  = 0;
    m_streams.stream[1].strLanguage[3]  = 0;

    m_streams.iStreamCount++;
  }
  else
  {
    m_streams.iStreamCount = 0;
    return false;
  }

  m_StreamChange = true;
  m_StreamActive = true;
  m_PTSNext      = DVD_TIME_BASE;

  m_FMDecoder->Reset();

  return true;
}

bool cRadioReceiver::GetStreamProperties(PVR_STREAM_PROPERTIES* props)
{
 for (int i=0; i<m_streams.iStreamCount; i++)
  {
    memcpy(&props->stream[i], &m_streams.stream[i], sizeof(PVR_STREAM_PROPERTIES::PVR_STREAM));
  }

  props->iStreamCount = m_streams.iStreamCount;
  return true;
}

void cRadioReceiver::CloseChannel()
{
  CLockObject lock(m_AudioSignalMutex);

  m_StreamActive = false;

  m_RtlSdrReceiver.Close();
  if (m_FMDecoder)
  {
    delete m_FMDecoder;
    m_FMDecoder = NULL;

    m_channelName.clear();
    m_AudioSourceBuffer.clear();
    m_streams.iStreamCount = 0;
  }
}

bool cRadioReceiver::SwitchChannel(const PVR_CHANNEL &channel)
{
  KODI->Log(LOG_DEBUG, "changing to channel %d", channel.iChannelNumber);

  if (!m_StreamActive || !SetChannel(channel))
    return false;

  m_RtlSdrReceiver.SetFrequency(m_activeChannelFrequency + 0.25 * m_IfRate);
  m_activeTunerFreq = m_RtlSdrReceiver.GetFrequency();
  m_PTSNext         = DVD_TIME_BASE;
  m_StreamChange    = true;

  m_channelName.clear();
  m_AudioSourceBuffer.clear();
  m_FMDecoder->Reset();

  KODI->Log(LOG_INFO, "device tuned for:  %.6f MHz", m_activeTunerFreq * 1.0e-6);

  return true;
}

void cRadioReceiver::Abort(void)
{
  m_streams.iStreamCount = 0;
}

bool cRadioReceiver::AddUECPDataFrame(uint8_t *UECPDataFrame, unsigned int length)
{
  if (m_UECPOutputBuffer.size() > 16384)
    return false;

  CLockObject lock(m_UECPMutex);

  uint8_t value;
  int ptr = 0;

  m_UECPOutputBuffer.push_back(0xFE);
  for (unsigned int i = 0; i < length; i++)
  {
    value = UECPDataFrame[i];
    if (value < 0xFD)
    {
      m_UECPOutputBuffer.push_back(value);
    }
    else
    {
      m_UECPOutputBuffer.push_back(0xFD);
      m_UECPOutputBuffer.push_back((value&3)-1);
    }
  }
  m_UECPOutputBuffer.push_back(0xFF);

  return true;
}

/*!
 * Input stream related functions and data
 */

size_t cRadioReceiver::SourceQueuedSamples()
{
  CLockObject lock(m_AudioSourceMutex);
  return m_AudioSourceSize;
}

void cRadioReceiver::WriteDataBuffer(std::vector<ComplexType> &iqsamples)
{
  if (!iqsamples.empty())
  {
    CLockObject lock(m_AudioSourceMutex);
    m_AudioSourceSize += iqsamples.size();
    m_AudioSourceBuffer.push_back(move(iqsamples));
    if (m_AudioSourceBuffer.size() > 3)
      m_AudioSourceEvent.Signal();
  }
}

void cRadioReceiver::EndDataBuffer()
{
  CLockObject lock(m_AudioSourceMutex);
  m_AudioSourceEndMarked = true;
  m_AudioSourceEvent.Broadcast();
}

bool cRadioReceiver::SourceGetSamples(std::vector<ComplexType> &samples)
{
  while (m_AudioSourceBuffer.empty() && !m_AudioSourceEndMarked)
    m_AudioSourceEvent.Sleep(500);

  CLockObject lock(m_AudioSourceMutex);

  if (!m_AudioSourceBuffer.empty())
  {
    m_AudioSourceSize -= m_AudioSourceBuffer.front().size();
    std::swap(samples, m_AudioSourceBuffer.front());
    m_AudioSourceBuffer.pop_front();
    return true;
  }

  return false;
}

DemuxPacket* cRadioReceiver::Read(void)
{
  unsigned int iSize   = 0;
  DemuxPacket* pPacket = NULL;

  /*!
   * Handle stream change
   */
  if (m_StreamChange)
  {
    pPacket = PVR->AllocateDemuxPacket(iSize);
    pPacket->iStreamId  = DMX_SPECIALID_STREAMCHANGE;
    m_StreamChange = false;
    return pPacket;
  }

  /*!
   * Handle and return RDS radio data if present
   */
  {
    CLockObject lock(m_UECPMutex);
    if (!m_UECPOutputBuffer.empty())
    {
      iSize   = m_UECPOutputBuffer.size();
      pPacket = PVR->AllocateDemuxPacket(iSize);
      if (!pPacket)
        return NULL;

      uint8_t *data = (uint8_t *)pPacket->pData;
      unsigned int i = 0;
      for (auto it = m_UECPOutputBuffer.cbegin(); it != m_UECPOutputBuffer.cend(); ++it, ++i)
        data[i] = *it;

      pPacket->iStreamId  = 2;
      pPacket->iSize      = iSize;
      pPacket->pts        = m_PTSNext;

      m_UECPOutputBuffer.clear();
      return pPacket;
    }
  }

  /*!
   * Handle, process and return radio sound
   */
  {
    //!> Check for overflow of source buffer.
    if (!m_AudioSourceBufferWarning && SourceQueuedSamples() > 10 * m_IfRate)
    {
      KODI->Log(LOG_ERROR, "Input buffer is growing (system too slow)");
      m_AudioSourceBufferWarning = true;
    }

    std::vector<ComplexType> iqsamples;
    if (!SourceGetSamples(iqsamples))                       //!< Pull next block from source buffer.
      return NULL;

    pPacket = PVR->AllocateDemuxPacket(iqsamples.size()*sizeof(float)*2);
    if (!pPacket)
      return NULL;

    iSize = m_FMDecoder->ProcessStream(iqsamples.data(), iqsamples.size(), (float *)pPacket->pData);           //!< Decode FM signal.

    double audio_mean, audio_rms;
    SamplesMeanRMS((float *)pPacket->pData, audio_mean, audio_rms, iSize);   //!< Measure audio level.
    m_AudioLevel = 0.95 * m_AudioLevel + 0.05 * audio_rms;

    double duration = (double)(iSize) * DVD_TIME_BASE / 2 / OUTPUT_SAMPLERATE;

    pPacket->iStreamId  = 1;
    pPacket->iSize      = iSize*sizeof(float);
    pPacket->duration   = duration;
    pPacket->pts        = m_PTSNext;

    m_PTSNext = m_PTSNext + duration;
  }

  return pPacket;
}

bool cRadioReceiver::GetSignalStatus(float &interfaceLevel, float &audioLevel, bool &stereo)
{
  CLockObject lock(m_AudioSignalMutex);

  if (!m_FMDecoder || m_StreamChange)
    return false;

  interfaceLevel  = 20*log10(m_FMDecoder->GetInterfaceLevel());
  audioLevel      = 20*log10(m_AudioLevel) + 3.01;
  stereo          = m_FMDecoder->StereoDetected();
  return true;
}

bool cRadioReceiver::GetSignalStatus(PVR_SIGNAL_STATUS &qualityinfo)
{
  CLockObject lock(m_AudioSignalMutex);

  if (!m_FMDecoder || m_StreamChange)
    return false;

  float interfaceLevel = 20*log10(m_FMDecoder->GetInterfaceLevel());
  float audioLevel = 20*log10(m_AudioLevel) + 3.01;

  std::string freq = StringUtils::Format("Freq.=%8.4fMHz - %s - IF=%+5.1fdB  BB=%+5.1fdB  Audio=%+5.1fdB",
                                          m_activeTunerFreq/1000000, m_FMDecoder->StereoDetected() ? "Stereo" : "Mono",
                                          (m_activeTunerFreq + m_FMDecoder->GetTuningOffset()) * 1.0e-6,
                                          interfaceLevel, 20*log10(m_FMDecoder->GetBasebandLevel()) + 3.01,
                                          audioLevel);

  strncpy(qualityinfo.strAdapterName, m_RtlSdrReceiver.GetDeviceName().c_str(), sizeof(qualityinfo.strAdapterName));
  strncpy(qualityinfo.strAdapterStatus, freq.c_str(), sizeof(qualityinfo.strAdapterStatus));
  if (!m_channelName.empty())
    strncpy(qualityinfo.strProviderName, m_channelName.c_str(), sizeof(qualityinfo.strProviderName));

  qualityinfo.iSignal       = 2.5*(interfaceLevel+40)*656;
  qualityinfo.iSNR          = (audioLevel+100)*656;

  return true;
}

void cRadioReceiver::SamplesMeanRMS(const float* samples, double& mean, double& rms, unsigned int n)
{
  float vsum = 0;
  float vsumsq = 0;

  for (unsigned int i = 0; i < n; ++i)
  {
    float v = samples[i];
    vsum   += v;
    vsumsq += v * v;
  }

  mean = vsum / n;
  rms  = sqrt(vsumsq / n);
}

bool cRadioReceiver::SetChannelName(std::string name)
{
  CLockObject lock(m_AudioSignalMutex);
  m_channelName = StringUtils::Trim(name);

  if (m_SettingsDialog)
  {
    m_SettingsDialog->UpdateName(name);
    return false;
  }

  return true;
}

void cRadioReceiver::RegisterDialog(cChannelSettings *dialog)
{
  CLockObject lock(m_AudioSignalMutex);
  m_SettingsDialog = dialog;
}

/*!
 * Channel setting functions
 */
std::string cRadioReceiver::GetSettingsFile() const
{
  std::string settingFile = g_strClientPath;
  if (settingFile.at(settingFile.size() - 1) == '\\' ||
      settingFile.at(settingFile.size() - 1) == '/')
    settingFile.append("PVRFMRadioAddonSettings.xml");
  else
    settingFile.append("/PVRFMRadioAddonSettings.xml");
  return settingFile;
}

bool cRadioReceiver::LoadChannelData(bool initial)
{
  TiXmlDocument xmlDoc;
  std::string strSettingsFile = GetSettingsFile();

  if (!xmlDoc.LoadFile(strSettingsFile))
  {
    if (initial)
    {
      if (!SaveChannelData())
      {
        KODI->Log(LOG_ERROR, "failed to create initial settings data file at '%s')", strSettingsFile.c_str());
        return false;
      }
      return true;
    }
    else
      KODI->Log(LOG_ERROR, "invalid settings data (no/invalid data file found at '%s')", strSettingsFile.c_str());
    return false;
  }

  TiXmlElement *pRootElement = xmlDoc.RootElement();
  if (strcmp(pRootElement->Value(), "radio") != 0)
  {
    if (!initial)
      KODI->Log(LOG_ERROR, "invalid radio data (no <radio> tag found)");
    return false;
  }

  TiXmlElement *pElement = pRootElement->FirstChildElement("devices");
  if (pElement)
  {
    TiXmlNode *pGroupNode = NULL;
    while ((pGroupNode = pElement->IterateChildren(pGroupNode)) != NULL)
    {
      if (!XMLUtils::GetInt(pGroupNode, "index", m_DeviceIndex))
        m_DeviceIndex = 0;
      break;
    }
  }

  pElement = pRootElement->FirstChildElement("uniqueid");
  if (pElement)
  {
    int iTmp;
    if (!XMLUtils::GetInt(pElement, "next", iTmp) | iTmp == 0)
      iTmp = 1;
    m_UniqueIdNextNew = (unsigned int)iTmp;
  }

  /* load channels */
  pElement = pRootElement->FirstChildElement("channels");
  if (pElement)
  {
    TiXmlNode *pChannelNode = NULL;
    while ((pChannelNode = pElement->IterateChildren(pChannelNode)) != NULL)
    {
      CStdString strTmp;
      FMRadioChannel channel;

      /* Channel unique kodi id */
      int iTmp;
      if (!XMLUtils::GetInt(pChannelNode, "uniqueid", iTmp) || iTmp == 0)
        continue;
      channel.iUniqueId = (unsigned int)iTmp;

      /* Channel frequency */
      if (!XMLUtils::GetFloat(pChannelNode, "freq", channel.fChannelFreq))
        continue;

      /* channel name */
      if (!XMLUtils::GetString(pChannelNode, "name", strTmp))
        channel.strChannelName = "";
      else
        channel.strChannelName = strTmp;

      /* icon path */
      if (!XMLUtils::GetString(pChannelNode, "icon", strTmp))
        channel.strIconPath = m_strDefaultIcon;
      else
        channel.strIconPath = strTmp;

      m_channels.push_back(channel);
    }
  }

  if (m_UniqueIdNextNew == 0)
    m_UniqueIdNextNew = m_channels.size()+1;

  return true;
}

bool cRadioReceiver::SaveChannelData(void)
{
  TiXmlDocument xmlDoc;
  TiXmlElement xmlRootElement("radio");
  TiXmlNode *pRoot = xmlDoc.InsertEndChild(xmlRootElement);
  if (pRoot == NULL)
    return false;

  TiXmlElement xmlDeviceSetting("devices");
  TiXmlNode* pDevicesNode = pRoot->InsertEndChild(xmlDeviceSetting);
  if (pDevicesNode)
  {
    TiXmlElement xmlSetting("device");
    TiXmlNode* pDeviceNode = pDevicesNode->InsertEndChild(xmlSetting);
    if (pDeviceNode)
      XMLUtils::SetInt(pDeviceNode, "index", m_DeviceIndex);
  }

  int iTmp = (int)m_UniqueIdNextNew;
  TiXmlElement xmlUniqueSetting("uniqueid");
  TiXmlNode* pUniquesNode = pRoot->InsertEndChild(xmlUniqueSetting);
  if (pUniquesNode)
    XMLUtils::SetInt(pUniquesNode, "next", iTmp);

  TiXmlElement xmlChannelsSetting("channels");
  TiXmlNode* pChannelsNode = pRoot->InsertEndChild(xmlChannelsSetting);
  if (pChannelsNode)
  {
    for (unsigned int i = 0; i < m_channels.size(); i++)
    {
      TiXmlElement xmlSetting("channel");
      TiXmlNode* pChannelNode = pChannelsNode->InsertEndChild(xmlSetting);
      if (pChannelNode)
      {
        XMLUtils::SetInt(pChannelNode, "uniqueid", (int)m_channels[i].iUniqueId);
        XMLUtils::SetFloat(pChannelNode, "freq", m_channels[i].fChannelFreq);
        XMLUtils::SetString(pChannelNode, "name", m_channels[i].strChannelName.c_str());
        XMLUtils::SetString(pChannelNode, "icon", m_channels[i].strIconPath.c_str());
      }
    }
  }

  if (!xmlDoc.SaveFile(GetSettingsFile()))
  {
    KODI->Log(LOG_ERROR, "failed to write speaker settings data");
    return false;
  }

  PVR->TriggerChannelUpdate();

  return true;
}
