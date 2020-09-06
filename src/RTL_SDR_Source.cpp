/*
 *  Copyright (C) 2013, Joris van Rantwijk.
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#include "RTL_SDR_Source.h"

#include "RadioReceiver.h"

#include <kodi/General.h>
#include <limits.h>
#include <rtl-sdr.h>
#include <string.h>

cRtlSdrSource::cRtlSdrSource(cRadioReceiver* proc)
  : m_DevicePtr(nullptr), m_Proc(proc), m_BlockLength(default_block_length)
{
}

cRtlSdrSource::~cRtlSdrSource()
{
  Close();
  if (m_DevicePtr)
    rtlsdr_close(m_DevicePtr);
  m_DevicePtr = nullptr;
}

bool cRtlSdrSource::OpenDevice(int dev_index)
{
  if (m_DevicePtr)
    rtlsdr_close(m_DevicePtr);
  m_DevicePtr = nullptr;

  const char* devname = rtlsdr_get_device_name(dev_index);
  if (devname != nullptr)
    m_DeviceName = devname;

  int ret = rtlsdr_open(&m_DevicePtr, dev_index);
  if (ret < 0)
  {
    m_error = "Failed to open RTL-SDR device (";
    m_error += strerror(-ret);
    m_error += ")";
    return false;
  }

  m_DeviceIndex = dev_index;
  return true;
}

void cRtlSdrSource::Close()
{
  m_Cancelled = true;

  if (m_DevicePtr)
    rtlsdr_cancel_async(m_DevicePtr);

  m_running = false;
  if (m_thread.joinable())
    m_thread.join();
}

// Configure RTL-SDR tuner and prepare for streaming.
bool cRtlSdrSource::Configure(
    uint32_t sample_rate, uint32_t frequency, int tuner_gain, int block_length, bool agcmode)
{
  int r;

  if (!m_DevicePtr)
    return false;

  r = rtlsdr_set_sample_rate(m_DevicePtr, sample_rate);
  if (r < 0)
  {
    m_error = "rtlsdr_set_sample_rate failed";
    return false;
  }

  r = rtlsdr_set_center_freq(m_DevicePtr, frequency);
  if (r < 0)
  {
    m_error = "rtlsdr_set_center_freq failed";
    return false;
  }

  if (tuner_gain == INT_MIN)
  {
    r = rtlsdr_set_tuner_gain_mode(m_DevicePtr, 0);
    if (r < 0)
    {
      m_error = "rtlsdr_set_tuner_gain_mode could not set automatic gain";
      return false;
    }
  }
  else
  {
    r = rtlsdr_set_tuner_gain_mode(m_DevicePtr, 1);
    if (r < 0)
    {
      m_error = "rtlsdr_set_tuner_gain_mode could not set manual gain";
      return false;
    }

    r = rtlsdr_set_tuner_gain(m_DevicePtr, tuner_gain);
    if (r < 0)
    {
      m_error = "rtlsdr_set_tuner_gain failed";
      return false;
    }
  }

  //! set RTL AGC mode
  r = rtlsdr_set_agc_mode(m_DevicePtr, int(agcmode));
  if (r < 0)
  {
    m_error = "rtlsdr_set_agc_mode failed";
    return false;
  }

  //! set block length
  m_BlockLength =
      (block_length < 4096) ? 4096 : (block_length > 1024 * 1024) ? 1024 * 1024 : block_length;
  m_BlockLength -= m_BlockLength % 4096;
  m_SampleRate = sample_rate;
  m_Frequency = frequency;
  m_TunerGain = tuner_gain;
  m_agcMode = agcmode;

  //! reset buffer to start streaming
  if (rtlsdr_reset_buffer(m_DevicePtr) < 0)
  {
    m_error = "rtlsdr_reset_buffer failed";
    return false;
  }

  m_Cancelled = false;
  m_running = true;
  m_thread = std::thread([&] { Process(); });

  return true;
}

uint32_t cRtlSdrSource::GetSampleRate()
{
  return rtlsdr_get_sample_rate(m_DevicePtr);
}

uint32_t cRtlSdrSource::GetFrequency()
{
  if (m_DevicePtr)
    return rtlsdr_get_center_freq(m_DevicePtr);
  else
    return 0;
}

void cRtlSdrSource::SetFrequency(uint32_t freq)
{
  if (m_DevicePtr)
    rtlsdr_set_center_freq(m_DevicePtr, freq);
}

int cRtlSdrSource::GetTunerGain()
{
  return rtlsdr_get_tuner_gain(m_DevicePtr);
}

std::vector<int> cRtlSdrSource::GetTunerGains()
{
  int num_gains = rtlsdr_get_tuner_gains(m_DevicePtr, nullptr);
  if (num_gains <= 0)
    return std::vector<int>();

  std::vector<int> gains(num_gains);
  if (rtlsdr_get_tuner_gains(m_DevicePtr, gains.data()) != num_gains)
    return std::vector<int>();

  return gains;
}

bool cRtlSdrSource::GetDeviceNames(std::vector<std::string>& result)
{
  int device_count = rtlsdr_get_device_count();
  if (device_count <= 0)
    return false;

  result.reserve(device_count);
  for (int i = 0; i < device_count; ++i)
    result.push_back(std::string(rtlsdr_get_device_name(i)));

  return true;
}

void cRtlSdrSource::ReadAsyncCB(unsigned char* buf, uint32_t len, void* ctx)
{
  cRtlSdrSource* thisClass = (cRtlSdrSource*)ctx;

  if (len != 2 * thisClass->m_BlockLength)
  {
    kodi::Log(ADDON_LOG_ERROR, "RtlSdr: short read, samples lost");
    return;
  }

  thisClass->m_Samples.resize(thisClass->m_BlockLength);
  for (unsigned int i = 0; i < thisClass->m_BlockLength; ++i)
  {
    thisClass->m_Samples[i] =
        ComplexType((buf[2 * i] / (255.0 / 2.0) - 1.0), (buf[2 * i + 1] / (255.0 / 2.0) - 1.0));
  }
  thisClass->m_Proc->WriteDataBuffer(thisClass->m_Samples);
}

void cRtlSdrSource::Process()
{
  unsigned int restartTry = 0;
  bool isOK = true;

  while (m_running && !m_Cancelled)
  {
    int readFail = rtlsdr_read_async(m_DevicePtr, ReadAsyncCB, this, 15, 2 * m_BlockLength);
    if (m_Cancelled)
      break;
    if (!isOK || readFail)
    {
      kodi::Log(ADDON_LOG_ERROR, "RtlSdr: %s", error().c_str());
      if (restartTry >= DEVICE_RESTART_TRIES)
      {
        kodi::Log(ADDON_LOG_ERROR, "RtlSdr: All restarts failed, exiting!");
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      restartTry++;
      kodi::Log(ADDON_LOG_INFO, "RtlSdr: %s trying restart %i", restartTry);
      isOK = Configure(m_SampleRate, m_Frequency, m_TunerGain, m_BlockLength, m_agcMode);
      continue;
    }
  }

  m_Proc->EndDataBuffer();
  return;
}
