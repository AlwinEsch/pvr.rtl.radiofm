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
 */

#include "client.h"
#include <stdint.h>
#include <vector>
#include <deque>
#include <string>

#include "xbmc_codec_descriptor.hpp"
#include "Definations.h"
#include "DownConvert.h"
#include "RTL_SDR_Source.h"

class cFmDecoder;
class cChannelSettings;

struct FMRadioChannel
{
  unsigned int  iUniqueId;
  float         fChannelFreq;
  std::string   strChannelName;
  std::string   strIconPath;
};

class cRadioReceiver
{
public:
  cRadioReceiver();
  virtual ~cRadioReceiver();

  /*!
   * From Kodi used functions
   */
  int GetChannelsAmount(void);
  PVR_ERROR GetChannels(ADDON_HANDLE handle);
  PVR_ERROR DeleteChannel(const PVR_CHANNEL &channel);

  bool OpenChannel(const PVR_CHANNEL &channel);
  bool GetStreamProperties(PVR_STREAM_PROPERTIES* props);
  void CloseChannel();
  bool SwitchChannel(const PVR_CHANNEL &channel);
  int CurrentChannel() { return m_activeChannelInfo.iChannelNumber; }
  DemuxPacket* Read(void);
  void Abort(void);

  bool GetSignalStatus(float &interfaceLevel, float &audioLevel, bool &stereo);
  bool GetSignalStatus(PVR_SIGNAL_STATUS &signalStatus);

  /*!
   * Intern of addon used functions
   */
  bool LoadChannelData(void);
  bool SaveChannelData(void);
  bool SetChannel(const PVR_CHANNEL &channel);
  unsigned int CreateNewUniqueId() { return m_UniqueIdNextNew++; }
  std::string CreateChannelName(FMRadioChannel &channel) const;
  std::vector<FMRadioChannel> *GetChannelData() { return &m_channels; }
  cRtlSdrSource *GetSource() { return &m_RtlSdrReceiver; }
  bool SetChannelName(std::string name);
  bool IsActive() { return m_StreamActive; }
  bool IsChannelActive(int index) { return m_StreamActive ? m_activeIndex == index : false; }
  bool IsSettingActive() { return m_SettingsDialog != NULL; }

  void RegisterDialog(cChannelSettings *dialog);
  void SetStreamChange() { m_StreamChange = true; }

private:
  std::string GetSettingsFile() const;

  inline void AdjustGain(float *samples, float gain, unsigned int n);
  inline void SamplesMeanRMS(const float* samples, double& mean, double& rms, unsigned int n);

  std::string                         m_strDefaultIcon;
  std::string                         m_channelName;
  int                                 m_DeviceIndex;
  std::vector<FMRadioChannel>         m_channels;
  ADDON::XbmcStreamProperties         m_activeChannelStream;
  PVR_CHANNEL                         m_activeChannelInfo;
  int                                 m_activeIndex;
  double                              m_activeChannelFrequency;
  double                              m_IfRate;
  double                              m_PCMRate;
  int                                 m_LnaGain;
  bool                                m_AgcMode;
  bool                                m_StreamChange;
  bool                                m_StreamActive;
  float                               m_AudioLevel;
  double                              m_activeTunerFreq;
  PVR_SIGNAL_STATUS                   m_activeSignalStatus;

  cChannelSettings                   *m_SettingsDialog;

/*!
 * Input stream related functions and data
 */
public:
  bool AddUECPDataFrame(uint8_t *UECPDataFrame);
  void WriteDataBuffer(std::vector<ComplexType> &iqsamples);
  void EndDataBuffer();

private:
  size_t SourceQueuedSamples();
  bool SourceGetSamples(std::vector<ComplexType> &samples);

  cRtlSdrSource                       m_RtlSdrReceiver;
  cFmDecoder                         *m_FMDecoder;

  PLATFORM::CMutex                    m_UECPMutex;
  std::vector<uint8_t>                m_UECPOutputBuffer;

  PLATFORM::CMutex                    m_AudioSignalMutex;

  size_t                              m_AudioSourceSize;
  std::deque<std::vector<ComplexType> >  m_AudioSourceBuffer;
  PLATFORM::CMutex                    m_AudioSourceMutex;
  PLATFORM::CEvent                    m_AudioSourceEvent;
  bool                                m_AudioSourceEndMarked;
  bool                                m_AudioSourceBufferWarning;

  unsigned int                        m_UniqueIdNextNew;
};
