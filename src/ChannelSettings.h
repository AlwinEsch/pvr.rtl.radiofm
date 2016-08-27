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
#include <string>
#include <map>

class cRadioReceiver;

class cChannelSettings : public P8PLATFORM::CThread
{
public:
  cChannelSettings();
  virtual ~cChannelSettings();

  PVR_ERROR Open(const PVR_CHANNEL &channel, cRadioReceiver *source, bool addAdjust);

  void UpdateName(std::string name);

  bool OnClick(int controlId);
  bool OnFocus(int controlId);
  bool OnInit();
  bool OnAction(int actionId);

  static bool OnClickCB(GUIHANDLE cbhdl, int controlId);
  static bool OnFocusCB(GUIHANDLE cbhdl, int controlId);
  static bool OnInitCB(GUIHANDLE cbhdl);
  static bool OnActionCB(GUIHANDLE cbhdl, int actionId);

protected:
  virtual void *Process(void);

private:
  void UpdateFreq(uint32_t freq);

  CAddonGUIWindow  *m_window;
  int               m_ChannelIndex;
  uint32_t          m_CurrentFreq;
  uint32_t          m_PrevFreq;
  cRadioReceiver   *m_Source;
  bool              m_addAdjust;
  bool              m_AutoTuneIgnore;
  bool              m_AutoTuneUp;
  bool              m_AutoTuneDown;
  std::string       m_Name;
  bool              m_WasSaved;
};
