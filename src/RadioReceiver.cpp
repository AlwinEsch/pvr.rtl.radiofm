/*
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#include "RadioReceiver.h"

#include "ChannelSettings.h"
#include "FmDecode.h"
#include "Utils.h"
#include "XMLUtils.h"

#include <algorithm>
#include <limits.h>

cRadioReceiver::cRadioReceiver() : m_RtlSdrReceiver(this)
{
  LoadChannelData(true);
}

cRadioReceiver::~cRadioReceiver()
{
  CloseLiveStream();

  m_channels.clear();
}

PVR_ERROR cRadioReceiver::GetCapabilities(kodi::addon::PVRCapabilities& capabilities)
{
  capabilities.SetSupportsEPG(false);
  capabilities.SetSupportsRecordings(false);
  capabilities.SetSupportsRecordingEdl(false);
  capabilities.SetSupportsRecordingsUndelete(false);
  capabilities.SetSupportsTimers(false);
  capabilities.SetSupportsTV(false);
  capabilities.SetSupportsRadio(true);
  capabilities.SetSupportsChannelGroups(false);
  capabilities.SetHandlesInputStream(true);
  capabilities.SetHandlesDemuxing(true);
  capabilities.SetSupportsChannelScan(false);
  capabilities.SetSupportsChannelSettings(true);

  return PVR_ERROR_NO_ERROR;
}

PVR_ERROR cRadioReceiver::GetBackendName(std::string& name)
{
  name = "RTL-SDR FM Radio receiver";
  return PVR_ERROR_NO_ERROR;
}

PVR_ERROR cRadioReceiver::GetBackendVersion(std::string& version)
{
  version = STR(RTL_RADIOFM_VERSION);
  return PVR_ERROR_NO_ERROR;
}

PVR_ERROR cRadioReceiver::GetConnectionString(std::string& connection)
{
  connection = "connected";
  return PVR_ERROR_NO_ERROR;
}

std::string cRadioReceiver::CreateChannelName(FMRadioChannel& channel) const
{
  std::string name;
  if (!channel.strChannelName.empty() && channel.strChannelName != "-")
    name = StringUtils::Trim(channel.strChannelName);
  else if (channel.fChannelFreq >= 87500000.0f)
    name = StringUtils::Format("FM %.01f MHz", channel.strChannelName.c_str(),
                               channel.fChannelFreq / 1000000.0f);
  else if (channel.fChannelFreq >= 531000.0f)
    name = StringUtils::Format("MF %.01f ḱHz", channel.strChannelName.c_str(),
                               channel.fChannelFreq / 1000.0f);
  else if (channel.fChannelFreq >= 153000.0f)
    name = StringUtils::Format("LF %.01f ḱHz", channel.strChannelName.c_str(),
                               channel.fChannelFreq / 1000.0f);

  return name;
}

PVR_ERROR cRadioReceiver::GetChannelsAmount(int& amount)
{
  amount = m_channels.size();
  return PVR_ERROR_NO_ERROR;
}

PVR_ERROR cRadioReceiver::DeleteChannel(const kodi::addon::PVRChannel& channel)
{
  for (int iChannelPtr = 0; iChannelPtr < (int)m_channels.size(); iChannelPtr++)
  {
    if (m_channels[iChannelPtr].iUniqueId == channel.GetUniqueId())
    {
      if (iChannelPtr == m_activeIndex)
        return PVR_ERROR_REJECTED;
      m_channels.erase(m_channels.begin() + iChannelPtr);
      SaveChannelData();
      break;
    }
  }
  return PVR_ERROR_NO_ERROR;
}

PVR_ERROR cRadioReceiver::RenameChannel(const kodi::addon::PVRChannel& channel)
{
  return PVR_ERROR_NOT_IMPLEMENTED;
}

PVR_ERROR cRadioReceiver::OpenDialogChannelSettings(const kodi::addon::PVRChannel& channel)
{
  cChannelSettings channels;
  return channels.Open(channel, this, false);
}

PVR_ERROR cRadioReceiver::OpenDialogChannelAdd(const kodi::addon::PVRChannel& channel)
{
  cChannelSettings channels;
  return channels.Open(channel, this, true);
}

PVR_ERROR cRadioReceiver::GetChannels(bool radio, kodi::addon::PVRChannelsResultSet& results)
{
  for (unsigned int iChannelPtr = 0; iChannelPtr < m_channels.size(); iChannelPtr++)
  {
    FMRadioChannel& channel = m_channels.at(iChannelPtr);
    kodi::addon::PVRChannel kodiChannel;
    std::string name = CreateChannelName(channel);

    kodiChannel.SetUniqueId(channel.iUniqueId);
    kodiChannel.SetIsRadio(true);
    kodiChannel.SetChannelName(name);
    kodiChannel.SetIconPath(channel.strIconPath);
    kodiChannel.SetIsHidden(false);

    results.Add(kodiChannel);
  }

  return PVR_ERROR_NO_ERROR;
}

bool cRadioReceiver::SetChannel(const kodi::addon::PVRChannel& channel)
{
  m_activeIndex = -1;
  for (unsigned int iChannelPtr = 0; iChannelPtr < m_channels.size(); iChannelPtr++)
  {
    if (m_channels[iChannelPtr].iUniqueId == channel.GetUniqueId())
    {
      float freq = m_channels[iChannelPtr].fChannelFreq;
      if (freq < 87500000.0f || freq > 108000000.0f)
      {
        kodi::Log(ADDON_LOG_ERROR, "Used frequency %f0.2 MHz not allowed",
                  freq / 1000.0f / 1000.0f);
        return false;
      }

      m_activeChannelInfo = channel;
      m_activeIndex = iChannelPtr;
      m_activeChannelFrequency =
          (freq + 0.10) *
          m_IfRate; //!< Intentionally tune at a higher frequency to avoid DC offset.
      break;
    }
  }

  if (m_activeIndex == -1)
  {
    kodi::Log(ADDON_LOG_ERROR, "channel '%s' unknown", channel.GetChannelName().c_str());
    return false;
  }

  m_activeChannelFrequency = m_channels[m_activeIndex].fChannelFreq;
  return true;
}

bool cRadioReceiver::OpenLiveStream(const kodi::addon::PVRChannel& channel)
{
  kodi::Log(ADDON_LOG_INFO,
            "Starting usage of software decoder for AM, FM and DAB+ broadcast radio with RTL-SDR");

  m_activeIndex = -1;
  m_DeviceIndex = -1;
  m_IfRate = 1.0e6;
  m_PCMRate = OUTPUT_SAMPLERATE;
  m_LnaGain = 197; //INT_MIN + 1;
  m_AgcMode = true;
  m_AudioLevel = 0.0f;
  m_AudioSourceEndMarked = false;
  m_AudioSourceSize = 0;
  m_AudioSourceBufferWarning = false;

  m_activeIndex = -1;
  for (unsigned int iChannelPtr = 0; iChannelPtr < m_channels.size(); iChannelPtr++)
  {
    if (m_channels[iChannelPtr].iUniqueId == channel.GetUniqueId())
    {
      float freq = m_channels[iChannelPtr].fChannelFreq;
      if (freq < 87500000.0f || freq > 108000000.0f)
      {
        kodi::Log(ADDON_LOG_ERROR, "Used frequency %f0.2 MHz not allowed",
                  freq / 1000.0f / 1000.0f);
        return false;
      }

      m_activeChannelInfo = channel;
      m_activeIndex = iChannelPtr;
      m_activeChannelFrequency =
          freq + 0.25 * m_IfRate; //!< Intentionally tune at a higher frequency to avoid DC offset.
      break;
    }
  }

  if (!SetChannel(channel))
    return false;

  std::vector<std::string> devnames;
  if (!cRtlSdrSource::GetDeviceNames(devnames))
  {
    kodi::Log(ADDON_LOG_ERROR, "No devices present");
    return false;
  }

  if (m_DeviceIndex < 0 || (unsigned int)m_DeviceIndex >= devnames.size())
  {
    if (m_DeviceIndex != -1)
      kodi::Log(ADDON_LOG_ERROR, "invalid device index %d", m_DeviceIndex);

    kodi::Log(ADDON_LOG_INFO, "Found %u device(s):", (unsigned int)devnames.size());
    for (unsigned int i = 0; i < devnames.size(); i++)
      kodi::Log(ADDON_LOG_INFO, " %2u: %s", i, devnames[i].c_str());
    m_DeviceIndex = 0;
  }
  kodi::Log(ADDON_LOG_INFO, "Using device %d: %s", m_DeviceIndex, devnames[m_DeviceIndex].c_str());

  //! Intentionally tune at a higher frequency to avoid DC offset.
  m_activeTunerFreq = m_activeChannelFrequency + 0.15 * m_IfRate;

  //! Open RTL-SDR device.
  if (!m_RtlSdrReceiver.OpenDevice(m_DeviceIndex))
  {
    kodi::Log(ADDON_LOG_ERROR, "Device open failed: %s", m_RtlSdrReceiver.error().c_str());
    return false;
  }

  //! Check LNA gain.
  if (m_LnaGain != INT_MIN)
  {
    std::vector<int> gains = m_RtlSdrReceiver.GetTunerGains();
    if (find(gains.begin(), gains.end(), m_LnaGain) == gains.end())
    {
      if (m_LnaGain != INT_MIN + 1)
        kodi::Log(ADDON_LOG_ERROR, "LNA gain %.1f dB not supported by tuner", m_LnaGain * 0.1);
      kodi::Log(ADDON_LOG_INFO, "  Supported LNA gains:");
      for (int g : gains)
        kodi::Log(ADDON_LOG_INFO, " - %.1f dB", 0.1 * g);
      return false;
    }
  }

  //! Configure RTL-SDR device and start streaming.
  bool ret = m_RtlSdrReceiver.Configure(m_IfRate, m_activeTunerFreq, m_LnaGain,
                                        cRtlSdrSource::default_block_length, m_AgcMode);
  if (!ret)
  {
    kodi::Log(ADDON_LOG_ERROR, "Device configuration failed: %s", m_RtlSdrReceiver.error().c_str());
    return false;
  }

  m_activeTunerFreq = m_RtlSdrReceiver.GetFrequency();
  kodi::Log(ADDON_LOG_INFO, "device tuned for:  %.6f MHz", m_activeTunerFreq * 1.0e-6);

  if (m_LnaGain == INT_MIN)
    kodi::Log(ADDON_LOG_INFO, "LNA gain:          auto");
  else
    kodi::Log(ADDON_LOG_INFO, "LNA gain:          %.1f dB", 0.1 * m_RtlSdrReceiver.GetTunerGain());

  m_IfRate = m_RtlSdrReceiver.GetSampleRate();
  kodi::Log(ADDON_LOG_INFO, "IF sample rate:    %.0f", m_IfRate);
  kodi::Log(ADDON_LOG_INFO, "RTL AGC mode:      %s", m_AgcMode ? "enabled" : "disabled");

  // The baseband signal is empty above 100 kHz, so we can
  // downsample to ~ 200 kS/s without loss of information.
  // This will speed up later processing stages.
  unsigned int downsample = std::max(1, int(m_IfRate / 215.0e3));
  kodi::Log(ADDON_LOG_INFO, "baseband downsampling factor %u", downsample);

  // Prevent aliasing at very low output sample rates.
  double bandwidth_pcm = std::min(DEFAULT_BANDWIDTH_PCM, 0.45 * m_PCMRate);
  kodi::Log(ADDON_LOG_INFO, "audio sample rate: %.0f", m_PCMRate);
  kodi::Log(ADDON_LOG_INFO, "audio bandwidth:   %.3f kHz", bandwidth_pcm * 1.0e-3);

  std::unique_lock<std::mutex> lock(m_AudioSignalMutex);

  // Prepare decoder.
  m_FMDecoder = new cFmDecoder(this, m_IfRate, // sample_rate_if
                               m_activeChannelFrequency - m_activeTunerFreq, // tuning_offset
                               m_PCMRate, // sample_rate_pcm
                               bandwidth_pcm, // bandwidth_pcm
                               downsample); // downsample


  kodi::addon::PVRCodec codec = kodi::addon::CInstancePVRClient::GetCodecByName("pcm_f32le");
  if (codec.GetCodecType() == PVR_CODEC_TYPE_AUDIO)
  {
    kodi::addon::PVRStreamProperties properties;

    properties.SetPID(1);
    properties.SetCodecType(codec.GetCodecType());
    properties.SetCodecId(codec.GetCodecId());
    properties.SetChannels(2);
    properties.SetSampleRate(OUTPUT_SAMPLERATE);
    properties.SetBitsPerSample(32);
    properties.SetBitRate(properties.GetSampleRate() * properties.GetChannels() *
                          properties.GetBitsPerSample());

    m_streams.emplace_back(properties);
  }
  else
  {
    return false;
  }

  codec = kodi::addon::CInstancePVRClient::GetCodecByName("rds");
  if (codec.GetCodecType() == PVR_CODEC_TYPE_RDS)
  {
    kodi::addon::PVRStreamProperties properties;

    properties.SetPID(2);
    properties.SetCodecType(codec.GetCodecType());
    properties.SetCodecId(codec.GetCodecId());
    properties.SetChannels(2);
    properties.SetSampleRate(OUTPUT_SAMPLERATE);
    properties.SetBitsPerSample(32);
    properties.SetBitRate(properties.GetSampleRate() * properties.GetChannels() *
                          properties.GetBitsPerSample());

    m_streams.emplace_back(properties);
  }
  else
  {
    return false;
  }

  m_StreamChange = true;
  m_StreamActive = true;
  m_PTSNext = STREAM_TIME_BASE;

  m_FMDecoder->Reset();

  return true;
}

PVR_ERROR cRadioReceiver::GetStreamProperties(
    std::vector<kodi::addon::PVRStreamProperties>& properties)
{
  if (m_streams.empty())
    return PVR_ERROR_FAILED;

  properties = m_streams;
  return PVR_ERROR_NO_ERROR;
}

void cRadioReceiver::CloseLiveStream()
{
  std::unique_lock<std::mutex> lock(m_AudioSignalMutex);

  m_StreamActive = false;

  m_RtlSdrReceiver.Close();
  if (m_FMDecoder)
  {
    delete m_FMDecoder;
    m_FMDecoder = nullptr;

    m_channelName.clear();
    m_AudioSourceBuffer.clear();
    m_streams.clear();
  }
}

void cRadioReceiver::DemuxAbort()
{
  m_streams.clear();
}

bool cRadioReceiver::AddUECPDataFrame(uint8_t* UECPDataFrame, unsigned int length)
{
  if (m_UECPOutputBuffer.size() > 16384)
    return false;

  std::unique_lock<std::mutex> lock(m_UECPMutex);

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
      m_UECPOutputBuffer.push_back((value & 3) - 1);
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
  std::unique_lock<std::mutex> lock(m_AudioSourceMutex);
  return m_AudioSourceSize;
}

void cRadioReceiver::WriteDataBuffer(std::vector<ComplexType>& iqsamples)
{
  if (!iqsamples.empty())
  {
    std::unique_lock<std::mutex> lock(m_AudioSourceMutex);
    m_AudioSourceSize += iqsamples.size();
    m_AudioSourceBuffer.push_back(move(iqsamples));
    if (m_AudioSourceBuffer.size() > 3)
      m_AudioSourceEvent.notify_one();
  }
}

void cRadioReceiver::EndDataBuffer()
{
  std::unique_lock<std::mutex> lock(m_AudioSourceMutex);
  m_AudioSourceEndMarked = true;
  m_AudioSourceEvent.notify_all();
}

bool cRadioReceiver::SourceGetSamples(std::vector<ComplexType>& samples)
{
  std::unique_lock<std::mutex> lock(m_AudioSourceMutex);
  while (m_AudioSourceBuffer.empty() && !m_AudioSourceEndMarked)
    m_AudioSourceEvent.wait_for(lock, std::chrono::milliseconds(20));

  if (!m_AudioSourceBuffer.empty())
  {
    m_AudioSourceSize -= m_AudioSourceBuffer.front().size();
    std::swap(samples, m_AudioSourceBuffer.front());
    m_AudioSourceBuffer.pop_front();
    return true;
  }

  return false;
}

DEMUX_PACKET* cRadioReceiver::DemuxRead()
{
  unsigned int iSize = 0;
  DEMUX_PACKET* pPacket = nullptr;

  /*!
   * Handle stream change
   */
  if (m_StreamChange)
  {
    pPacket = kodi::addon::CInstancePVRClient::AllocateDemuxPacket(iSize);
    pPacket->iStreamId = DEMUX_SPECIALID_STREAMCHANGE;
    m_StreamChange = false;
    return pPacket;
  }

  /*!
   * Handle and return RDS radio data if present
   */
  {
    std::unique_lock<std::mutex> lock(m_UECPMutex);
    if (!m_UECPOutputBuffer.empty())
    {
      iSize = m_UECPOutputBuffer.size();
      pPacket = kodi::addon::CInstancePVRClient::AllocateDemuxPacket(iSize);
      if (!pPacket)
        return nullptr;

      uint8_t* data = (uint8_t*)pPacket->pData;
      unsigned int i = 0;
      for (auto it = m_UECPOutputBuffer.cbegin(); it != m_UECPOutputBuffer.cend(); ++it, ++i)
        data[i] = *it;

      pPacket->iStreamId = 2;
      pPacket->iSize = iSize;
      pPacket->pts = m_PTSNext;

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
      kodi::Log(ADDON_LOG_ERROR, "Input buffer is growing (system too slow)");
      m_AudioSourceBufferWarning = true;
    }

    std::vector<ComplexType> iqsamples;
    if (!SourceGetSamples(iqsamples)) //!< Pull next block from source buffer.
      return nullptr;

    pPacket =
        kodi::addon::CInstancePVRClient::AllocateDemuxPacket(iqsamples.size() * sizeof(float) * 2);
    if (!pPacket)
      return nullptr;

    iSize = m_FMDecoder->ProcessStream(iqsamples.data(), iqsamples.size(),
                                       (float*)pPacket->pData); //!< Decode FM signal.

    double audio_mean, audio_rms;
    SamplesMeanRMS((float*)pPacket->pData, audio_mean, audio_rms, iSize); //!< Measure audio level.
    m_AudioLevel = 0.95 * m_AudioLevel + 0.05 * audio_rms;

    double duration = (double)(iSize) * STREAM_TIME_BASE / 2 / OUTPUT_SAMPLERATE;

    pPacket->iStreamId = 1;
    pPacket->iSize = iSize * sizeof(float);
    pPacket->duration = duration;
    pPacket->pts = m_PTSNext;

    m_PTSNext = m_PTSNext + duration;
  }

  return pPacket;
}

bool cRadioReceiver::GetSignalStatus(float& interfaceLevel, float& audioLevel, bool& stereo)
{
  std::unique_lock<std::mutex> lock(m_AudioSignalMutex);

  if (!m_FMDecoder || m_StreamChange)
    return false;

  interfaceLevel = 20 * log10(m_FMDecoder->GetInterfaceLevel());
  audioLevel = 20 * log10(m_AudioLevel) + 3.01;
  stereo = m_FMDecoder->StereoDetected();
  return true;
}

PVR_ERROR cRadioReceiver::GetSignalStatus(int channelUid,
                                          kodi::addon::PVRSignalStatus& signalStatus)
{
  std::unique_lock<std::mutex> lock(m_AudioSignalMutex);

  if (!m_FMDecoder || m_StreamChange)
    return PVR_ERROR_UNKNOWN;

  float interfaceLevel = 20 * log10(m_FMDecoder->GetInterfaceLevel());
  float audioLevel = 20 * log10(m_AudioLevel) + 3.01;

  std::string freq = StringUtils::Format(
      "Freq.=%8.4fMHz - %s - IF=%+5.1fdB  BB=%+5.1fdB  Audio=%+5.1fdB", m_activeTunerFreq / 1000000,
      m_FMDecoder->StereoDetected() ? "Stereo" : "Mono",
      (m_activeTunerFreq + m_FMDecoder->GetTuningOffset()) * 1.0e-6, interfaceLevel,
      20 * log10(m_FMDecoder->GetBasebandLevel()) + 3.01, audioLevel);

  signalStatus.SetAdapterName(m_RtlSdrReceiver.GetDeviceName());
  signalStatus.SetAdapterStatus(freq);
  signalStatus.SetProviderName(m_channelName.c_str());

  signalStatus.SetSignal(2.5 * (interfaceLevel + 40) * 656);
  signalStatus.SetSNR((audioLevel + 100) * 656);

  return PVR_ERROR_NO_ERROR;
}

void cRadioReceiver::SamplesMeanRMS(const float* samples, double& mean, double& rms, unsigned int n)
{
  float vsum = 0;
  float vsumsq = 0;

  for (unsigned int i = 0; i < n; ++i)
  {
    float v = samples[i];
    vsum += v;
    vsumsq += v * v;
  }

  mean = vsum / n;
  rms = sqrt(vsumsq / n);
}

bool cRadioReceiver::SetChannelName(std::string name)
{
  std::unique_lock<std::mutex> lock(m_AudioSignalMutex);
  m_channelName = StringUtils::Trim(name);

  if (m_SettingsDialog)
  {
    m_SettingsDialog->UpdateName(name);
    return false;
  }

  return true;
}

void cRadioReceiver::RegisterDialog(cChannelSettings* dialog)
{
  std::unique_lock<std::mutex> lock(m_AudioSignalMutex);
  m_SettingsDialog = dialog;
}

/*!
 * Channel setting functions
 */
std::string cRadioReceiver::GetSettingsFile() const
{
  return kodi::GetAddonPath("PVRFMRadioAddonSettings.xml");
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
        kodi::Log(ADDON_LOG_ERROR, "failed to create initial settings data file at '%s')",
                  strSettingsFile.c_str());
        return false;
      }
      return true;
    }
    else
      kodi::Log(ADDON_LOG_ERROR, "invalid settings data (no/invalid data file found at '%s')",
                strSettingsFile.c_str());
    return false;
  }

  TiXmlElement* pRootElement = xmlDoc.RootElement();
  if (strcmp(pRootElement->Value(), "radio") != 0)
  {
    if (!initial)
      kodi::Log(ADDON_LOG_ERROR, "invalid radio data (no <radio> tag found)");
    return false;
  }

  TiXmlElement* pElement = pRootElement->FirstChildElement("devices");
  if (pElement)
  {
    TiXmlNode* pGroupNode = nullptr;
    while ((pGroupNode = pElement->IterateChildren(pGroupNode)) != nullptr)
    {
      if (!xml::GetInt(pGroupNode, "index", m_DeviceIndex))
        m_DeviceIndex = 0;
      break;
    }
  }

  pElement = pRootElement->FirstChildElement("uniqueid");
  if (pElement)
  {
    int iTmp;
    if (!xml::GetInt(pElement, "next", iTmp) || iTmp == 0)
      iTmp = 1;
    m_UniqueIdNextNew = (unsigned int)iTmp;
  }

  /* load channels */
  pElement = pRootElement->FirstChildElement("channels");
  if (pElement)
  {
    TiXmlNode* pChannelNode = nullptr;
    while ((pChannelNode = pElement->IterateChildren(pChannelNode)) != nullptr)
    {
      std::string strTmp;
      FMRadioChannel channel;

      /* Channel unique kodi id */
      int iTmp;
      if (!xml::GetInt(pChannelNode, "uniqueid", iTmp) || iTmp == 0)
        continue;
      channel.iUniqueId = (unsigned int)iTmp;

      /* Channel frequency */
      if (!xml::GetFloat(pChannelNode, "freq", channel.fChannelFreq))
        continue;

      /* channel name */
      if (!xml::GetString(pChannelNode, "name", strTmp))
        channel.strChannelName = "";
      else
        channel.strChannelName = strTmp;

      /* icon path */
      if (!xml::GetString(pChannelNode, "icon", strTmp))
        channel.strIconPath = m_strDefaultIcon;
      else
        channel.strIconPath = strTmp;

      m_channels.push_back(channel);
    }
  }

  if (m_UniqueIdNextNew == 0)
    m_UniqueIdNextNew = m_channels.size() + 1;

  return true;
}

bool cRadioReceiver::SaveChannelData(void)
{
  TiXmlDocument xmlDoc;
  TiXmlElement xmlRootElement("radio");
  TiXmlNode* pRoot = xmlDoc.InsertEndChild(xmlRootElement);
  if (pRoot == nullptr)
    return false;

  TiXmlElement xmlDeviceSetting("devices");
  TiXmlNode* pDevicesNode = pRoot->InsertEndChild(xmlDeviceSetting);
  if (pDevicesNode)
  {
    TiXmlElement xmlSetting("device");
    TiXmlNode* pDeviceNode = pDevicesNode->InsertEndChild(xmlSetting);
    if (pDeviceNode)
      xml::SetInt(pDeviceNode, "index", m_DeviceIndex);
  }

  int iTmp = (int)m_UniqueIdNextNew;
  TiXmlElement xmlUniqueSetting("uniqueid");
  TiXmlNode* pUniquesNode = pRoot->InsertEndChild(xmlUniqueSetting);
  if (pUniquesNode)
    xml::SetInt(pUniquesNode, "next", iTmp);

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
        xml::SetInt(pChannelNode, "uniqueid", (int)m_channels[i].iUniqueId);
        xml::SetFloat(pChannelNode, "freq", m_channels[i].fChannelFreq);
        xml::SetString(pChannelNode, "name", m_channels[i].strChannelName.c_str());
        xml::SetString(pChannelNode, "icon", m_channels[i].strIconPath.c_str());
      }
    }
  }

  if (!xmlDoc.SaveFile(GetSettingsFile()))
  {
    kodi::Log(ADDON_LOG_ERROR, "failed to write speaker settings data");
    return false;
  }

  kodi::addon::CInstancePVRClient::TriggerChannelUpdate();

  return true;
}

ADDONCREATOR(cRadioReceiver)
