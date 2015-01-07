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

#include "RadioReceiver.h"
#include "ChannelSettings.h"
#include "FmDecode.h"

using namespace std;
using namespace ADDON;
using namespace PLATFORM;

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

  LoadChannelData();
}

cRadioReceiver::~cRadioReceiver()
{
  CloseChannel();

  m_channels.clear();
}

std::string cRadioReceiver::CreateChannelName(FMRadioChannel &channel) const
{
  CStdString name;
  if (!channel.strChannelName.empty() && channel.strChannelName != "-")
  {
    name = channel.strChannelName;
    while (name[0] == ' ' && !channel.strChannelName.empty())
      name.erase(name.begin());
  }
  else if (channel.fChannelFreq >= 87500000.0f)
    name.Format("FM %.01f MHz", channel.strChannelName.c_str(), channel.fChannelFreq / 1000000.0f);
  else if (channel.fChannelFreq >= 531000.0f)
    name.Format("MF %.01f ḱHz", channel.strChannelName.c_str(), channel.fChannelFreq / 1000.0f);
  else if (channel.fChannelFreq >= 153000.0f)
    name.Format("LF %.01f ḱHz", channel.strChannelName.c_str(), channel.fChannelFreq / 1000.0f);

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
    string name = CreateChannelName(channel);

    kodiChannel.iUniqueId         = channel.iUniqueId;
    kodiChannel.bIsRadio          = true;
    strncpy(kodiChannel.strChannelName, name.c_str(), sizeof(kodiChannel.strChannelName) - 1);
    strncpy(kodiChannel.strIconPath, channel.strIconPath.c_str(), sizeof(kodiChannel.strIconPath) - 1);
    kodiChannel.bIsHidden         = false;

    PVR->TransferChannelEntry(handle, &kodiChannel);
  }

  return PVR_ERROR_NO_ERROR;
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

  if (m_activeIndex == -1)
  {
    KODI->Log(LOG_ERROR, "channel '%s' unknown", channel.strChannelName);
    return false;
  }

  m_activeChannelFrequency      = m_channels[m_activeIndex].fChannelFreq;
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

  vector<string> devnames;
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
      KODI->Log(LOG_INFO, " %2u: %s\n", i, devnames[i].c_str());
    m_DeviceIndex = 0;
  }
  KODI->Log(LOG_INFO, "Using device %d: %s\n", m_DeviceIndex, devnames[m_DeviceIndex].c_str());

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
    vector<int> gains;
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
  KODI->Log(LOG_INFO, "IF sample rate:    %.0f Hz", m_IfRate);
  KODI->Log(LOG_INFO, "RTL AGC mode:      %s", m_AgcMode ? "enabled" : "disabled");

  std::vector<XbmcPvrStream> newStreams;

  CodecDescriptor codecId = CodecDescriptor::GetCodecByName("pcm_f32le");
  if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_AUDIO)
  {
    XbmcPvrStream newStream;
    m_activeChannelStream.GetStreamData(1, &newStream);

    newStream.iCodecType      = codecId.Codec().codec_type;
    newStream.iCodecId        = codecId.Codec().codec_id;
    newStream.iChannels       = 2;
    newStream.iSampleRate     = OUTPUT_SAMPLERATE;
    newStream.iBitsPerSample  = 32;
    newStream.iBitRate        = newStream.iSampleRate * newStream.iChannels * newStream.iBitsPerSample;
    newStream.strLanguage[0]  = 0;
    newStream.strLanguage[1]  = 0;
    newStream.strLanguage[2]  = 0;
    newStream.strLanguage[3]  = 0;
    newStream.iIdentifier     = -1;

    newStreams.push_back(newStream);
    m_activeChannelStream.UpdateStreams(newStreams);
  }
  else
  {
    m_activeChannelStream.Clear();
    return false;
  }

  codecId = CodecDescriptor::GetCodecByName("rds");
  if (codecId.Codec().codec_type == XBMC_CODEC_TYPE_DATA)
  {
    XbmcPvrStream newStream;
    m_activeChannelStream.GetStreamData(2, &newStream);

    newStream.iCodecType      = codecId.Codec().codec_type;
    newStream.iCodecId        = codecId.Codec().codec_id;
    newStream.iChannels       = 2;
    newStream.iSampleRate     = OUTPUT_SAMPLERATE;
    newStream.iBitsPerSample  = 32;
    newStream.iBitRate        = newStream.iSampleRate * newStream.iChannels * newStream.iBitsPerSample;
    newStream.strLanguage[0]  = 0;
    newStream.strLanguage[1]  = 0;
    newStream.strLanguage[2]  = 0;
    newStream.strLanguage[3]  = 0;
    newStream.iIdentifier     = -1;

    newStreams.push_back(newStream);
    m_activeChannelStream.UpdateStreams(newStreams);
  }
  else
  {
    m_activeChannelStream.Clear();
    return false;
  }

  // The baseband signal is empty above 100 kHz, so we can
  // downsample to ~ 200 kS/s without loss of information.
  // This will speed up later processing stages.
  unsigned int downsample = max(1, int(m_IfRate / 215.0e3));
  KODI->Log(LOG_INFO, "baseband downsampling factor %u", downsample);

  // Prevent aliasing at very low output sample rates.
  double bandwidth_pcm = min(DEFAULT_BANDWIDTH_PCM, 0.45 * m_PCMRate);
  KODI->Log(LOG_INFO, "audio sample rate: %u Hz", m_PCMRate);
  KODI->Log(LOG_INFO, "audio bandwidth:   %.3f kHz", bandwidth_pcm * 1.0e-3);

  CLockObject lock(m_AudioSignalMutex);

  // Prepare decoder.
  m_FMDecoder = new cFmDecoder(this, m_IfRate,                            // sample_rate_if
                               m_activeChannelFrequency - m_activeTunerFreq,                 // tuning_offset
                               m_PCMRate,                           // sample_rate_pcm
                               bandwidth_pcm,                     // bandwidth_pcm
                               downsample);                       // downsample

  m_StreamChange = true;
  m_StreamActive = true;

  return true;
}

bool cRadioReceiver::GetStreamProperties(PVR_STREAM_PROPERTIES* props)
{
  return m_activeChannelStream.GetProperties(props);
}

void cRadioReceiver::CloseChannel()
{
  CLockObject lock(m_AudioSignalMutex);

  m_StreamActive = false;

  m_RtlSdrReceiver.Close();
  if (m_FMDecoder)
    delete m_FMDecoder;
  m_FMDecoder = NULL;

  m_channelName.clear();
  m_AudioSourceBuffer.clear();
  m_activeChannelStream.Clear();
}

bool cRadioReceiver::SwitchChannel(const PVR_CHANNEL &channel)
{
  KODI->Log(LOG_DEBUG, "changing to channel %d", channel.iChannelNumber);
  CloseChannel();
  return OpenChannel(channel);
}

void cRadioReceiver::Abort(void)
{
//  CloseChannel();
//  m_activeChannelStream.Clear();
}

bool cRadioReceiver::AddUECPDataFrame(uint8_t *UECPDataFrame)
{
  if (m_UECPOutputBuffer.size() > 16384)
    return false;

  CLockObject lock(m_UECPMutex);

  int ptr = 0;
  while (UECPDataFrame[ptr] != 0xFF)
  {
    m_UECPOutputBuffer.push_back(UECPDataFrame[ptr]);
    ptr++;
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
    m_AudioSourceEvent.Sleep(1000);

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
      int iStreamId = m_activeChannelStream.GetStreamId(2);
      if (iStreamId == -1)
      {
        KODI->Log(LOG_DEBUG, "rds stream id not found");
        return NULL;
      }

      iSize   = m_UECPOutputBuffer.size();
      pPacket = PVR->AllocateDemuxPacket(iSize);
      if (!pPacket)
        return NULL;

      uint8_t *data = (uint8_t *)pPacket->pData;
      for (unsigned int i = 0; i < m_UECPOutputBuffer.size(); i++)
        data[i] = m_UECPOutputBuffer[i];

      pPacket->iStreamId  = iStreamId;
      pPacket->iSize      = iSize;

      m_UECPOutputBuffer.clear();
      return pPacket;
    }
  }

  /*!
   * Handle, process and return radio sound
   */
  {
    int iStreamId = m_activeChannelStream.GetStreamId(1);
    if (iStreamId == -1)
    {
      KODI->Log(LOG_DEBUG, "radio audio stream id not found");
      return NULL;
    }

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

    pPacket->iStreamId  = m_activeChannelStream.GetStreamId(1);
    pPacket->iSize      = iSize*sizeof(float);
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

  CStdString freq;
  freq.Format("Freq.=%8.4fMHz - %s - IF=%+5.1fdB  BB=%+5.1fdB  Audio=%+5.1fdB",
                m_FMDecoder->StereoDetected() ? "Stereo" : "Mono",
                (m_activeTunerFreq + m_FMDecoder->GetTuningOffset()) * 1.0e-6,
                interfaceLevel, 20*log10(m_FMDecoder->GetBasebandLevel()) + 3.01,
                audioLevel);

  strncpy(qualityinfo.strAdapterName, m_RtlSdrReceiver.GetDeviceName().c_str(), sizeof(qualityinfo.strAdapterName));
  strncpy(qualityinfo.strAdapterStatus, freq.c_str(), sizeof(qualityinfo.strAdapterStatus));
  if (!m_channelName.empty())
    strncpy(qualityinfo.strProviderName, m_channelName.c_str(), sizeof(qualityinfo.strProviderName));

  qualityinfo.iSignal       = 2.5*(interfaceLevel+40)*656;
  qualityinfo.iSNR          = (audioLevel+100)*656;
  qualityinfo.dVideoBitrate = 0;
  qualityinfo.dAudioBitrate = m_IfRate;
  qualityinfo.dDolbyBitrate = 0;

  return true;
}

void cRadioReceiver::SamplesMeanRMS(const float* samples, double& mean, double& rms, unsigned int n)
{
  float vsum = 0;
  float vsumsq = 0;

  for (unsigned int i = 0; i < n; i++)
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
  m_channelName = name;

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
  string settingFile = g_strClientPath;
  if (settingFile.at(settingFile.size() - 1) == '\\' ||
      settingFile.at(settingFile.size() - 1) == '/')
    settingFile.append("PVRFMRadioAddonSettings.xml");
  else
    settingFile.append("/PVRFMRadioAddonSettings.xml");
  return settingFile;
}

bool cRadioReceiver::LoadChannelData(void)
{
  TiXmlDocument xmlDoc;
  string strSettingsFile = GetSettingsFile();

  if (!xmlDoc.LoadFile(strSettingsFile))
  {
    KODI->Log(LOG_ERROR, "invalid demo data (no/invalid data file found at '%s')", strSettingsFile.c_str());
    return false;
  }

  TiXmlElement *pRootElement = xmlDoc.RootElement();
  if (strcmp(pRootElement->Value(), "radio") != 0)
  {
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
