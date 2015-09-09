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

#include <sstream>
#include <limits.h>
#include "platform/util/StringUtils.h"

#include "ChannelSettings.h"
#include "RadioReceiver.h"

#define ACTION_NAV_BACK                 92
#define ACTION_MOUSE_ROLL_UP            104
#define ACTION_MOUSE_ROLL_DOWN          105

#define BUTTON_OK                       5
#define BUTTON_CANCEL                   6

#define CONTROL_BUTTON_RUN_DOWN         30
#define CONTROL_BUTTON_DECREASE_1       31
#define CONTROL_BUTTON_DECREASE_2       32
#define CONTROL_LABEL_FREQ              33
#define CONTROL_BUTTON_INCREASE_2       34
#define CONTROL_BUTTON_INCREASE_1       35
#define CONTROL_BUTTON_RUN_UP           36
#define CONTROL_BUTTON_CHANNEL_NAME     37
#define CONTROL_LABEL_LEVEL             38

using namespace ADDON;

cChannelSettings::cChannelSettings()
 : m_window(NULL)
{
}

cChannelSettings::~cChannelSettings()
{
  StopThread();
}

PVR_ERROR cChannelSettings::Open(const PVR_CHANNEL &channel, cRadioReceiver *source, bool addAdjust)
{
  m_Source    = source;
  m_addAdjust = addAdjust;
  m_WasSaved  = false;

  m_ChannelIndex = -1;
  if (m_addAdjust)
  {
    if (m_Source->IsActive())
      m_CurrentFreq = m_Source->GetSource()->GetFrequency() - 0.25 * m_Source->GetSource()->GetSampleRate();
    else
      m_CurrentFreq = 87500000;
  }
  else
  {
    std::vector<FMRadioChannel> *channels = m_Source->GetChannelData();
    for (unsigned int iChannelPtr = 0; iChannelPtr < channels->size(); iChannelPtr++)
    {
      if (channels->at(iChannelPtr).iUniqueId == channel.iUniqueId)
      {
        m_ChannelIndex = iChannelPtr;
        m_CurrentFreq = channels->at(iChannelPtr).fChannelFreq;
        break;
      }
    }
    if (m_ChannelIndex == -1)
    {
      XBMC->Log(LOG_ERROR, "channel '%s' id %i unknown", channel.strChannelName, channel.iUniqueId);
      return PVR_ERROR_INVALID_PARAMETERS;
    }
  }

  if (m_Source->IsActive())
    m_PrevFreq = m_Source->GetSource()->GetFrequency() - 0.25 * m_Source->GetSource()->GetSampleRate();
  else
    m_PrevFreq = m_CurrentFreq;

  /* Load the Window as Dialog */
  m_window = GUI->Window_create("ChannelTuner.xml", "Confluence", false, true);
  m_window->m_cbhdl   = this;
  m_window->CBOnInit  = OnInitCB;
  m_window->CBOnFocus = OnFocusCB;
  m_window->CBOnClick = OnClickCB;
  m_window->CBOnAction= OnActionCB;
  m_window->DoModal();

  StopThread();
  m_Source->RegisterDialog(NULL);

  GUI->Window_destroy(m_window);

  return PVR_ERROR_NO_ERROR;
}

void *cChannelSettings::Process()
{
  m_AutoTuneIgnore = false;
  while (!IsStopped())
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

    Sleep(1250);
    if (!m_Source->IsActive())
      break;

    float interfaceLevel;
    float audioLevel;
    bool stereo;
    if (m_Source->GetSignalStatus(interfaceLevel, audioLevel, stereo))
    {
      m_window->SetControlLabel(CONTROL_LABEL_LEVEL, StringUtils::Format("IF=%+5.1fdB Audio=%+5.1fdB", interfaceLevel, audioLevel).c_str());

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
      m_AutoTuneUp   = false;
      m_AutoTuneDown = false;
    }
    m_AutoTuneIgnore = false;
  }
  return NULL;
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
      m_window->Close();
      m_WasSaved = true;
      break;
    case BUTTON_CANCEL:
      UpdateFreq(m_PrevFreq);
      m_window->Close();
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

  m_AutoTuneUp   = false;
  m_AutoTuneDown = false;

  if (m_Source->IsActive())
    CreateThread();
  return true;
}

bool cChannelSettings::OnAction(int actionId)
{
  bool ret = false;

  switch (actionId)
  {
    case ACTION_MOUSE_ROLL_UP:
      ret = OnClick(CONTROL_BUTTON_INCREASE_2);
      break;
    case ACTION_MOUSE_ROLL_DOWN:
      ret = OnClick(CONTROL_BUTTON_DECREASE_2);
      break;
    case ADDON_ACTION_CLOSE_DIALOG:
    case ADDON_ACTION_PREVIOUS_MENU:
    case ACTION_NAV_BACK:
      ret = OnClick(BUTTON_CANCEL);
      break;
    default:
      break;
  }

  return ret;
}

bool cChannelSettings::OnInitCB(GUIHANDLE cbhdl)
{
  cChannelSettings* scanner = static_cast<cChannelSettings*>(cbhdl);
  return scanner->OnInit();
}

bool cChannelSettings::OnClickCB(GUIHANDLE cbhdl, int controlId)
{
  cChannelSettings* scanner = static_cast<cChannelSettings*>(cbhdl);
  return scanner->OnClick(controlId);
}

bool cChannelSettings::OnFocusCB(GUIHANDLE cbhdl, int controlId)
{
  cChannelSettings* scanner = static_cast<cChannelSettings*>(cbhdl);
  return scanner->OnFocus(controlId);
}

bool cChannelSettings::OnActionCB(GUIHANDLE cbhdl, int actionId)
{
  cChannelSettings* scanner = static_cast<cChannelSettings*>(cbhdl);
  return scanner->OnAction(actionId);
}

void cChannelSettings::UpdateFreq(uint32_t freq)
{
  uint32_t newFreq = freq + 0.25 * m_Source->GetSource()->GetSampleRate();

  m_window->SetControlLabel(CONTROL_LABEL_FREQ, StringUtils::Format("%.01f MHz", (float)freq/1000000).c_str());
  if (m_Source->IsActive())
    m_Source->GetSource()->SetFrequency(newFreq);

  UpdateName("-");
}

void cChannelSettings::UpdateName(std::string name)
{
  m_Name = name;
  m_window->SetControlLabel(CONTROL_BUTTON_CHANNEL_NAME, m_Name.c_str());
}
