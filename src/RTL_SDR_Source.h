/*
 *  Copyright (C) 2013, Joris van Rantwijk.
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#pragma once

#include "Definitions.h"

#include <atomic>
#include <stdint.h>
#include <string>
#include <thread>
#include <vector>

class cRadioReceiver;

class ATTRIBUTE_HIDDEN cRtlSdrSource
{
public:
  //static const int default_block_length = 16 * 32 * 512;
  static const int default_block_length = 65536;

  cRtlSdrSource(cRadioReceiver* proc);
  virtual ~cRtlSdrSource();

  /*!>
   * Open the given rtl sdr device index number
   * @param dev_index Index to open
   * @return true if successed
   */
  bool OpenDevice(int dev_index);

  /*!>
   * Close the rtl sdr device
   */
  void Close();

  /*!>
   * Configure RTL-SDR tuner and prepare for streaming.
   *
   * sample_rate  :: desired sample rate in Hz.
   * frequency    :: desired center frequency in Hz.
   * tuner_gain   :: desired tuner gain in 0.1 dB, or INT_MIN for auto-gain.
   * block_length :: preferred number of samples per block.
   *
   * Return true for success, false if an error occurred.
   */
  bool Configure(uint32_t sample_rate,
                 uint32_t frequency,
                 int tuner_gain,
                 int block_length = default_block_length,
                 bool agcmode = false);

  /*!> Return current sample frequency in Hz. */
  uint32_t GetSampleRate();

  /*!> Return current center frequency in Hz. */
  uint32_t GetFrequency();

  void SetFrequency(uint32_t freq);

  /*!> Return current tuner gain in units of 0.1 dB. */
  int GetTunerGain();

  /*!> Return a list of supported tuner gain settings in units of 0.1 dB. */
  std::vector<int> GetTunerGains();

  /*!> Return name of opened RTL-SDR device. */
  std::string GetDeviceName() const { return m_DeviceName; }

  /*!> Return the last error, or return an empty string if there is no error. */
  std::string error()
  {
    std::string ret(m_error);
    m_error.clear();
    return ret;
  }

  /*!> Return true if the device is OK, return false if there is an error. */
  operator bool() const { return m_DevicePtr && m_error.empty(); }

  /*!> Return a list of supported devices. */
  static bool GetDeviceNames(std::vector<std::string>& result);

protected:
  void Process();

private:
  static void ReadAsyncCB(unsigned char* buf, uint32_t len, void* ctx);

  std::atomic<bool> m_running = {false};
  std::thread m_thread;

  struct rtlsdr_dev* m_DevicePtr;
  cRadioReceiver* m_Proc;
  unsigned int m_BlockLength;
  int m_DeviceIndex;
  uint32_t m_SampleRate;
  uint32_t m_Frequency;
  int m_TunerGain;
  bool m_agcMode;
  std::vector<ComplexType> m_Samples;
  bool m_Cancelled;

  std::string m_DeviceName;
  std::string m_error;
};
