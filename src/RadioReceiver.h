/*
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#pragma once

#include "Definitions.h"
#include "DownConvert.h"
#include "RTL_SDR_Source.h"

#include <condition_variable>
#include <deque>
#include <kodi/addon-instance/PVR.h>
#include <limits.h>
#include <mutex>
#include <stdint.h>
#include <string>
#include <vector>

class cFmDecoder;
class cChannelSettings;

struct FMRadioChannel
{
  unsigned int iUniqueId;
  float fChannelFreq;
  std::string strChannelName;
  std::string strIconPath;
};

class ATTRIBUTE_HIDDEN cRadioReceiver : public kodi::addon::CAddonBase,
                                        public kodi::addon::CInstancePVRClient
{
public:
  cRadioReceiver();
  virtual ~cRadioReceiver();

  PVR_ERROR GetCapabilities(kodi::addon::PVRCapabilities& capabilities) override;
  PVR_ERROR GetBackendName(std::string& name) override;
  PVR_ERROR GetBackendVersion(std::string& version) override;
  PVR_ERROR GetConnectionString(std::string& connection) override;

  /*!
   * From Kodi used functions
   */
  PVR_ERROR GetChannelsAmount(int& amount) override;
  PVR_ERROR GetChannels(bool radio, kodi::addon::PVRChannelsResultSet& results) override;
  PVR_ERROR DeleteChannel(const kodi::addon::PVRChannel& channel) override;
  PVR_ERROR RenameChannel(const kodi::addon::PVRChannel& channel) override;
  PVR_ERROR OpenDialogChannelSettings(const kodi::addon::PVRChannel& channel) override;
  PVR_ERROR OpenDialogChannelAdd(const kodi::addon::PVRChannel& channel) override;
  PVR_ERROR GetSignalStatus(int channelUid, kodi::addon::PVRSignalStatus& signalStatus) override;

  bool OpenLiveStream(const kodi::addon::PVRChannel& channel) override;
  void CloseLiveStream() override;
  PVR_ERROR GetStreamProperties(std::vector<kodi::addon::PVRStreamProperties>& properties) override;
  DemuxPacket* DemuxRead() override;
  void DemuxAbort() override;

  int CurrentChannel() { return m_activeChannelInfo.GetChannelNumber(); }

  bool GetSignalStatus(float& interfaceLevel, float& audioLevel, bool& stereo);

  /*!
   * Intern of addon used functions
   */
  bool LoadChannelData(bool initial);
  bool SaveChannelData(void);
  bool SetChannel(const kodi::addon::PVRChannel& channel);
  unsigned int CreateNewUniqueId() { return m_UniqueIdNextNew++; }
  std::string CreateChannelName(FMRadioChannel& channel) const;
  std::vector<FMRadioChannel>* GetChannelData() { return &m_channels; }
  cRtlSdrSource* GetSource() { return &m_RtlSdrReceiver; }
  bool SetChannelName(std::string name);
  bool IsActive() { return m_StreamActive; }
  bool IsChannelActive(int index) { return m_StreamActive ? m_activeIndex == index : false; }
  bool IsSettingActive() { return m_SettingsDialog != nullptr; }

  void RegisterDialog(cChannelSettings* dialog);
  void SetStreamChange() { m_StreamChange = true; }

private:
  std::string GetSettingsFile() const;

  inline void AdjustGain(float* samples, float gain, unsigned int n);
  inline void SamplesMeanRMS(const float* samples, double& mean, double& rms, unsigned int n);

  std::string m_strDefaultIcon;
  std::string m_channelName;
  int m_DeviceIndex = -1;
  std::vector<FMRadioChannel> m_channels;
  std::vector<kodi::addon::PVRStreamProperties> m_streams;
  kodi::addon::PVRChannel m_activeChannelInfo;
  int m_activeIndex = -1;
  double m_activeChannelFrequency;
  double m_IfRate = 1.0e6;
  double m_PCMRate = OUTPUT_SAMPLERATE;
  int m_LnaGain = INT_MIN;
  bool m_AgcMode = false;
  bool m_StreamChange = false;
  bool m_StreamActive = false;
  float m_AudioLevel = 0.0f;
  double m_activeTunerFreq;
  kodi::addon::PVRSignalStatus m_activeSignalStatus;

  cChannelSettings* m_SettingsDialog = nullptr;

  /*!
 * Input stream related functions and data
 */
public:
  bool AddUECPDataFrame(uint8_t* UECPDataFrame, unsigned int length);
  void WriteDataBuffer(std::vector<ComplexType>& iqsamples);
  void EndDataBuffer();

private:
  size_t SourceQueuedSamples();
  bool SourceGetSamples(std::vector<ComplexType>& samples);

  cRtlSdrSource m_RtlSdrReceiver;
  cFmDecoder* m_FMDecoder = nullptr;

  std::mutex m_UECPMutex;
  std::vector<uint8_t> m_UECPOutputBuffer;

  std::mutex m_AudioSignalMutex;
  double m_PTSNext;

  size_t m_AudioSourceSize;
  std::deque<std::vector<ComplexType>> m_AudioSourceBuffer;
  std::mutex m_AudioSourceMutex;
  std::condition_variable m_AudioSourceEvent;
  bool m_AudioSourceEndMarked;
  bool m_AudioSourceBufferWarning;

  unsigned int m_UniqueIdNextNew;
};
