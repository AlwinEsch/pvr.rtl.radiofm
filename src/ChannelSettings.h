/*
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#pragma once

#include <atomic>
#include <kodi/addon-instance/pvr/Channels.h>
#include <kodi/gui/Window.h>
#include <map>
#include <string>
#include <thread>

class cRadioReceiver;

class ATTRIBUTE_HIDDEN cChannelSettings : public kodi::gui::CWindow
{
public:
  cChannelSettings();
  virtual ~cChannelSettings();

  PVR_ERROR Open(const kodi::addon::PVRChannel& channel, cRadioReceiver* source, bool addAdjust);

  void UpdateName(std::string name);

  bool OnClick(int controlId) override;
  bool OnFocus(int controlId) override;
  bool OnInit() override;
  bool OnAction(int actionId, uint32_t buttoncode, wchar_t unicode) override;

protected:
  void Process();

private:
  void UpdateFreq(uint32_t freq);

  std::atomic<bool> m_running = {false};
  std::thread m_thread;

  int m_ChannelIndex;
  uint32_t m_CurrentFreq;
  uint32_t m_PrevFreq;
  cRadioReceiver* m_Source;
  bool m_addAdjust;
  bool m_AutoTuneIgnore;
  bool m_AutoTuneUp;
  bool m_AutoTuneDown;
  std::string m_Name;
  bool m_WasSaved;
};
