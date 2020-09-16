/*
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#include "ChannelSettings.h"

#include "RadioReceiver.h"
#include "Utils.h"

#include <limits.h>
#include <sstream>

#define BUTTON_OK 5
#define BUTTON_CANCEL 6

#define CONTROL_BUTTON_RUN_DOWN 30
#define CONTROL_BUTTON_DECREASE_1 31
#define CONTROL_BUTTON_DECREASE_2 32
#define CONTROL_LABEL_FREQ 33
#define CONTROL_BUTTON_INCREASE_2 34
#define CONTROL_BUTTON_INCREASE_1 35
#define CONTROL_BUTTON_RUN_UP 36
#define CONTROL_BUTTON_CHANNEL_NAME 37
#define CONTROL_LABEL_LEVEL 38


cChannelSettings::cChannelSettings()
  : kodi::gui::CWindow("ChannelTuner.xml", "skin.estuary", true, true)
{
}

cChannelSettings::~cChannelSettings()
{
  m_running = false;
  if (m_thread.joinable())
    m_thread.join();
}

PVR_ERROR cChannelSettings::Open(const kodi::addon::PVRChannel& channel,
                                 cRadioReceiver* source,
                                 bool addAdjust)
{
  m_Source = source;
  m_addAdjust = addAdjust;
  m_WasSaved = false;

  m_ChannelIndex = -1;
  if (m_addAdjust)
  {
    if (m_Source->IsActive())
      m_CurrentFreq =
          m_Source->GetSource()->GetFrequency() - 0.25 * m_Source->GetSource()->GetSampleRate();
    else
      m_CurrentFreq = 87500000;
  }
  else
  {
    std::vector<FMRadioChannel>* channels = m_Source->GetChannelData();
    for (unsigned int iChannelPtr = 0; iChannelPtr < channels->size(); iChannelPtr++)
    {
      if (channels->at(iChannelPtr).iUniqueId == channel.GetUniqueId())
      {
        m_ChannelIndex = iChannelPtr;
        m_CurrentFreq = channels->at(iChannelPtr).fChannelFreq;
        break;
      }
    }
    if (m_ChannelIndex == -1)
    {
      kodi::Log(ADDON_LOG_ERROR, "channel '%s' id %i unknown", channel.GetChannelName().c_str(),
                channel.GetUniqueId());
      return PVR_ERROR_INVALID_PARAMETERS;
    }
  }

  if (m_Source->IsActive())
    m_PrevFreq =
        m_Source->GetSource()->GetFrequency() - 0.25 * m_Source->GetSource()->GetSampleRate();
  else
    m_PrevFreq = m_CurrentFreq;

  DoModal();

  m_running = false;
  if (m_thread.joinable())
    m_thread.join();

  m_Source->RegisterDialog(nullptr);

  return PVR_ERROR_NO_ERROR;
}

void cChannelSettings::Process()
{
  m_AutoTuneIgnore = false;
  while (m_running)
  {
    if (m_AutoTuneUp)
    {
      m_CurrentFreq += 100000;
      if (m_CurrentFreq > 108000000)
        m_CurrentFreq = 87500000;

      UpdateFreq(m_CurrentFreq);
    }
    else if (m_AutoTuneDown)
    {
      m_CurrentFreq -= 100000;
      if (m_CurrentFreq < 87500000)
        m_CurrentFreq = 108000000;

      UpdateFreq(m_CurrentFreq);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1250));
    if (!m_Source->IsActive())
      break;

    float interfaceLevel;
    float audioLevel;
    bool stereo;
    if (m_Source->GetSignalStatus(interfaceLevel, audioLevel, stereo) == PVR_ERROR_NO_ERROR)
    {
      SetControlLabel(CONTROL_LABEL_LEVEL, StringUtils::Format("IF=%+5.1fdB Audio=%+5.1fdB",
                                                               interfaceLevel, audioLevel));

      if ((m_AutoTuneUp || m_AutoTuneDown) && !m_AutoTuneIgnore)
      {
        if (stereo)
        {
          m_AutoTuneUp = false;
          m_AutoTuneDown = false;
        }
      }
    }
    else
    {
      m_AutoTuneUp = false;
      m_AutoTuneDown = false;
    }
    m_AutoTuneIgnore = false;
  }
  return;
}

bool cChannelSettings::OnClick(int controlId)
{
  switch (controlId)
  {
    case CONTROL_BUTTON_RUN_DOWN:
      m_AutoTuneDown = !m_AutoTuneDown;
      m_AutoTuneUp = false;
      m_AutoTuneIgnore = true;
      break;
    case CONTROL_BUTTON_DECREASE_1:
      m_CurrentFreq -= 1000000;
      if (m_CurrentFreq < 87500000)
        m_CurrentFreq = 108000000;
      UpdateFreq(m_CurrentFreq);
      m_AutoTuneDown = false;
      m_AutoTuneUp = false;
      break;
    case CONTROL_BUTTON_DECREASE_2:
      m_CurrentFreq -= 100000;
      if (m_CurrentFreq < 87500000)
        m_CurrentFreq = 108000000;
      UpdateFreq(m_CurrentFreq);
      m_AutoTuneDown = false;
      m_AutoTuneUp = false;
      break;
    case CONTROL_BUTTON_INCREASE_2:
      m_CurrentFreq += 100000;
      if (m_CurrentFreq > 108000000)
        m_CurrentFreq = 87500000;
      UpdateFreq(m_CurrentFreq);
      m_AutoTuneDown = false;
      m_AutoTuneUp = false;
      break;
    case CONTROL_BUTTON_INCREASE_1:
      m_CurrentFreq += 1000000;
      if (m_CurrentFreq > 108000000)
        m_CurrentFreq = 87500000;
      UpdateFreq(m_CurrentFreq);
      m_AutoTuneDown = false;
      m_AutoTuneUp = false;
      break;
    case CONTROL_BUTTON_RUN_UP:
      m_AutoTuneUp = !m_AutoTuneUp;
      m_AutoTuneDown = false;
      m_AutoTuneIgnore = true;
      break;
    case CONTROL_BUTTON_CHANNEL_NAME:
    {
    }
    break;
    case BUTTON_OK:
      if (m_addAdjust)
      {
        FMRadioChannel channel;
        channel.iUniqueId = m_Source->CreateNewUniqueId();
        channel.fChannelFreq = m_CurrentFreq;
        channel.strChannelName = m_Name;
        channel.strChannelName = m_Source->CreateChannelName(channel);
        channel.strIconPath = "";
        m_Source->GetChannelData()->push_back(channel);
      }
      else
      {
        m_Source->GetChannelData()->at(m_ChannelIndex).fChannelFreq = m_CurrentFreq;

        if (m_Source->IsChannelActive(m_ChannelIndex))
          m_PrevFreq = m_CurrentFreq;
      }

      m_Source->SaveChannelData();
      UpdateFreq(m_PrevFreq);
      Close();
      m_WasSaved = true;
      break;
    case BUTTON_CANCEL:
      UpdateFreq(m_PrevFreq);
      Close();
      break;
    default:
      break;
  }

  return true;
}

bool cChannelSettings::OnFocus(int controlId)
{
  return true;
}

bool cChannelSettings::OnInit()
{
  UpdateFreq(m_CurrentFreq);
  UpdateName("-");

  m_Source->RegisterDialog(this);

  m_AutoTuneUp = false;
  m_AutoTuneDown = false;

  if (m_Source->IsActive())
  {
    m_running = true;
    m_thread = std::thread([&] { Process(); });
  }
  return true;
}

bool cChannelSettings::OnAction(ADDON_ACTION actionId)
{
  bool ret = false;

  switch (actionId)
  {
    case ADDON_ACTION_MOUSE_WHEEL_UP:
      ret = OnClick(CONTROL_BUTTON_INCREASE_2);
      break;
    case ADDON_ACTION_MOUSE_WHEEL_DOWN:
      ret = OnClick(CONTROL_BUTTON_DECREASE_2);
      break;
    case ADDON_ACTION_PREVIOUS_MENU:
    case ADDON_ACTION_NAV_BACK:
      ret = OnClick(BUTTON_CANCEL);
      break;
    default:
      break;
  }

  return ret;
}

void cChannelSettings::UpdateFreq(uint32_t freq)
{
  uint32_t newFreq = freq + 0.25 * m_Source->GetSource()->GetSampleRate();

  SetControlLabel(CONTROL_LABEL_FREQ, StringUtils::Format("%.01f MHz", (float)freq / 1000000));
  if (m_Source->IsActive())
    m_Source->GetSource()->SetFrequency(newFreq);

  UpdateName("-");
}

void cChannelSettings::UpdateName(std::string name)
{
  m_Name = name;
  SetControlLabel(CONTROL_BUTTON_CHANNEL_NAME, m_Name);
}
