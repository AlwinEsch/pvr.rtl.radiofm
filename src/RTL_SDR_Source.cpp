/*
 *      Copyright (C) 2013, Joris van Rantwijk.
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

#include <limits.h>
#include <string.h>
#include <rtl-sdr.h>

#include "RadioReceiver.h"
#include "RTL_SDR_Source.h"

using namespace ADDON;

cRtlSdrSource::cRtlSdrSource(cRadioReceiver *proc)
  : m_DevicePtr(NULL)
  , m_Proc(proc)
  , m_BlockLength(default_block_length)
{
}

cRtlSdrSource::~cRtlSdrSource()
{
  Close();
  if (m_DevicePtr)
    rtlsdr_close(m_DevicePtr);
  m_DevicePtr = NULL;
}

bool cRtlSdrSource::OpenDevice(int dev_index)
{
  if (m_DevicePtr)
    rtlsdr_close(m_DevicePtr);
  m_DevicePtr = NULL;

  const char *devname = rtlsdr_get_device_name(dev_index);
  if (devname != NULL)
    m_DeviceName = devname;

  int ret = rtlsdr_open(&m_DevicePtr, dev_index);
  if (ret < 0)
  {
    m_error =  "Failed to open RTL-SDR device (";
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

  StopThread();
}

// Configure RTL-SDR tuner and prepare for streaming.
bool cRtlSdrSource::Configure(uint32_t sample_rate,
                              uint32_t frequency,
                              int tuner_gain,
                              int block_length,
                              bool agcmode)
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
  m_BlockLength = (block_length < 4096) ? 4096 :
                  (block_length > 1024 * 1024) ? 1024 * 1024 :
                   block_length;
  m_BlockLength -= m_BlockLength % 4096;
  m_SampleRate  = sample_rate;
  m_Frequency   = frequency;
  m_TunerGain   = tuner_gain;
  m_agcMode     = agcmode;

  //! reset buffer to start streaming
  if (rtlsdr_reset_buffer(m_DevicePtr) < 0)
  {
    m_error = "rtlsdr_reset_buffer failed";
    return false;
  }

  m_Cancelled = false;
  CreateThread();

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

bool cRtlSdrSource::GetTunerGains(std::vector<int> &gains)
{
  int num_gains = rtlsdr_get_tuner_gains(m_DevicePtr, NULL);
  if (num_gains <= 0)
    return false;

  gains.resize(num_gains);
  if (rtlsdr_get_tuner_gains(m_DevicePtr, gains.data()) != num_gains)
    return false;

  return true;
}

bool cRtlSdrSource::GetDeviceNames(std::vector<std::string> &result)
{
  int device_count = rtlsdr_get_device_count();
  if (device_count <= 0)
    return false;

  result.reserve(device_count);
  for (int i = 0; i < device_count; i++)
    result.push_back(std::string(rtlsdr_get_device_name(i)));

  return true;
}

void cRtlSdrSource::ReadAsyncCB(unsigned char *buf, uint32_t len, void *ctx)
{
  cRtlSdrSource *thisClass = (cRtlSdrSource *)ctx;

  if (len != 2 * thisClass->m_BlockLength)
  {
    KODI->Log(LOG_ERROR, "RtlSdr: short read, samples lost");
    return;
  }

  thisClass->m_Samples.resize(thisClass->m_BlockLength);
  for (unsigned int i = 0; i < thisClass->m_BlockLength; i++)
  {
    thisClass->m_Samples[i] = ComplexType((buf [2 * i    ] / (255.0/2.0) - 1.0),
                                          (buf [2 * i + 1] / (255.0/2.0) - 1.0));
  }
  thisClass->m_Proc->WriteDataBuffer(thisClass->m_Samples);
}

void *cRtlSdrSource::Process(void)
{
  unsigned int restartTry = 0;
  bool isOK = true;

  while (!IsStopped() && !m_Cancelled)
  {
    int readFail = rtlsdr_read_async(m_DevicePtr, ReadAsyncCB, this, 15, 2 * m_BlockLength);
    if (m_Cancelled)
      break;
    if (!isOK || readFail)
    {
      KODI->Log(LOG_ERROR, "RtlSdr: %s", error().c_str());
      if (restartTry >= DEVICE_RESTART_TRIES)
      {
        KODI->Log(LOG_ERROR, "RtlSdr: All restarts failed, exiting!");
        break;
      }

      Sleep(1000);

      restartTry++;
      KODI->Log(LOG_INFO, "RtlSdr: %s trying restart %i", restartTry);
      isOK = Configure(m_SampleRate, m_Frequency, m_TunerGain, m_BlockLength, m_agcMode);
      continue;
    }
  }

  m_Proc->EndDataBuffer();
  return NULL;
}
