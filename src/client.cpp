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
#include "kodi/xbmc_pvr_dll.h"
#include "kodi/util/util.h"

#include "RadioReceiver.h"
#include "ChannelSettings.h"

#include <sstream>
#include <string>
#include <iostream>

using namespace std;
using namespace ADDON;

/*!
 * User adjustable settings are saved here.
 * Default values are defined inside client.h
 * and exported to the other source files.
 */
bool                m_bCreated       = false;
ADDON_STATUS        m_CurStatus      = ADDON_STATUS_UNKNOWN;
cRadioReceiver     *m_dataProc       = NULL;
bool                m_bIsPlaying     = false;

std::string         g_strUserPath    = "";
std::string         g_strClientPath  = "";

CHelper_libXBMC_addon *KODI   = NULL;
CHelper_libXBMC_codec *CODEC  = NULL;
CHelper_libXBMC_gui   *GUI    = NULL;
CHelper_libXBMC_pvr   *PVR    = NULL;

extern "C" {

/***********************************************************
 * Standart AddOn related public library functions
 ***********************************************************/

ADDON_STATUS ADDON_Create(void* hdl, void* props)
{
  if (!hdl || !props)
    return ADDON_STATUS_UNKNOWN;

  PVR_PROPERTIES* pvrprops = (PVR_PROPERTIES*)props;

  KODI = new CHelper_libXBMC_addon;
  if (!KODI->RegisterMe(hdl))
  {
    SAFE_DELETE(KODI);
    return ADDON_STATUS_PERMANENT_FAILURE;
  }

  GUI = new CHelper_libXBMC_gui;
  if (!GUI->RegisterMe(hdl))
  {
    SAFE_DELETE(GUI);
    SAFE_DELETE(KODI);
    return ADDON_STATUS_PERMANENT_FAILURE;
  }

  CODEC = new CHelper_libXBMC_codec;
  if (!CODEC->RegisterMe(hdl))
  {
    SAFE_DELETE(CODEC);
    SAFE_DELETE(GUI);
    SAFE_DELETE(KODI);
    return ADDON_STATUS_PERMANENT_FAILURE;
  }

  PVR = new CHelper_libXBMC_pvr;
  if (!PVR->RegisterMe(hdl))
  {
    SAFE_DELETE(PVR);
    SAFE_DELETE(CODEC);
    SAFE_DELETE(GUI);
    SAFE_DELETE(KODI);
    return ADDON_STATUS_PERMANENT_FAILURE;
  }

  KODI->Log(LOG_DEBUG, "Creating RTL-SDR FM Radio PVR-Addon");

  m_CurStatus     = ADDON_STATUS_UNKNOWN;
  g_strUserPath   = pvrprops->strUserPath;
  g_strClientPath = pvrprops->strClientPath;

  m_dataProc = new cRadioReceiver;
  m_CurStatus = ADDON_STATUS_OK;
  m_bCreated = true;

  return m_CurStatus;
}

ADDON_STATUS ADDON_GetStatus()
{
  return m_CurStatus;
}

void ADDON_Destroy()
{
  SAFE_DELETE(CODEC);

  if (m_dataProc)
    SAFE_DELETE(m_dataProc);

  if (PVR)
    SAFE_DELETE(PVR);

  if (GUI)
    SAFE_DELETE(GUI);

  if (KODI)
    SAFE_DELETE(KODI);

  m_bCreated = false;
  m_CurStatus = ADDON_STATUS_UNKNOWN;
}

bool ADDON_HasSettings()
{
  return true;
}

unsigned int ADDON_GetSettings(ADDON_StructSetting ***sSet)
{
  return 0;
}

ADDON_STATUS ADDON_SetSetting(const char *settingName, const void *settingValue)
{
  return ADDON_STATUS_OK;
}

void ADDON_Stop()
{
}

void ADDON_FreeSettings()
{

}

void ADDON_Announce(const char *flag, const char *sender, const char *message, const void *data)
{
}

/***********************************************************
 * PVR Client AddOn specific public library functions
 ***********************************************************/

const char* GetPVRAPIVersion(void)
{
  static const char *strApiVersion = XBMC_PVR_API_VERSION;
  return strApiVersion;
}

const char* GetMininumPVRAPIVersion(void)
{
  static const char *strMinApiVersion = XBMC_PVR_MIN_API_VERSION;
  return strMinApiVersion;
}

const char* GetGUIAPIVersion(void)
{
  static const char *strGuiApiVersion = XBMC_GUI_API_VERSION;
  return strGuiApiVersion;
}

const char* GetMininumGUIAPIVersion(void)
{
  static const char *strMinGuiApiVersion = XBMC_GUI_MIN_API_VERSION;
  return strMinGuiApiVersion;
}

PVR_ERROR GetAddonCapabilities(PVR_ADDON_CAPABILITIES* pCapabilities)
{
  pCapabilities->bSupportsEPG                = false;
  pCapabilities->bSupportsRecordings         = false;
  pCapabilities->bSupportsRecordingEdl       = false;
  pCapabilities->bSupportsTimers             = false;
  pCapabilities->bSupportsTV                 = false;
  pCapabilities->bSupportsRadio              = true;
  pCapabilities->bSupportsChannelGroups      = false;
  pCapabilities->bHandlesInputStream         = true;
  pCapabilities->bHandlesDemuxing            = true;
  pCapabilities->bSupportsChannelScan        = false;
  pCapabilities->bSupportsChannelSettings    = true;

  return PVR_ERROR_NO_ERROR;
}

const char *GetBackendName(void)
{
  static std::string BackendName = "RTL-SDR FM Radio receiver";
  return BackendName.c_str();
}

const char *GetBackendVersion(void)
{
  static std::string strBackendVersion = "0.1";
  return strBackendVersion.c_str();
}

const char *GetConnectionString(void)
{
  static std::string strConnectionString = "connected";
  return strConnectionString.c_str();
}

PVR_ERROR DialogChannelScan(void)
{
  return PVR_ERROR_NOT_IMPLEMENTED;
}

PVR_ERROR DialogChannelSettings(const PVR_CHANNEL &channel)
{
  if (m_dataProc)
  {
    cChannelSettings channels;
    return channels.Open(channel, m_dataProc, false);
  }
  return PVR_ERROR_SERVER_ERROR;
}

PVR_ERROR DialogAddChannel(const PVR_CHANNEL &channel)
{
  if (m_dataProc)
  {
    cChannelSettings channels;
    return channels.Open(channel, m_dataProc, true);
  }
  return PVR_ERROR_SERVER_ERROR;
}

PVR_ERROR DeleteChannel(const PVR_CHANNEL &channel)
{
  if (m_dataProc)
    return m_dataProc->DeleteChannel(channel);
  return PVR_ERROR_SERVER_ERROR;
}

PVR_ERROR RenameChannel(const PVR_CHANNEL &channel)
{
  return PVR_ERROR_NOT_IMPLEMENTED;
}

/*******************************************/
/** PVR Channel Functions                 **/

int GetChannelsAmount(void)
{
  if (m_dataProc)
    return m_dataProc->GetChannelsAmount();

  return -1;
}

PVR_ERROR GetChannels(ADDON_HANDLE handle, bool bRadio)
{
  if (bRadio && m_dataProc)
    return m_dataProc->GetChannels(handle);

  return PVR_ERROR_SERVER_ERROR;
}

/*******************************************/
/** PVR Live Stream Functions             **/

bool OpenLiveStream(const PVR_CHANNEL &channel)
{
  CloseLiveStream();

  if (m_dataProc)
    return m_dataProc->OpenChannel(channel);

  return false;
}

void CloseLiveStream(void)
{
  if (m_dataProc)
    m_dataProc->CloseChannel();
}

PVR_ERROR GetStreamProperties(PVR_STREAM_PROPERTIES* pProperties)
{
  if (!m_dataProc)
    return PVR_ERROR_SERVER_ERROR;

  return (m_dataProc->GetStreamProperties(pProperties) ? PVR_ERROR_NO_ERROR : PVR_ERROR_SERVER_ERROR);
}

void DemuxAbort(void)
{
  if (m_dataProc)
    m_dataProc->Abort();
}

DemuxPacket* DemuxRead(void)
{
  if (!m_dataProc)
    return NULL;

  return m_dataProc->Read();
}

int GetCurrentClientChannel(void)
{
  if (m_dataProc)
    return m_dataProc->CurrentChannel();

  return -1;
}

bool SwitchChannel(const PVR_CHANNEL &channel)
{
  if (m_dataProc)
    return m_dataProc->SwitchChannel(channel);

  return false;
}

PVR_ERROR SignalStatus(PVR_SIGNAL_STATUS &signalStatus)
{
  if (!m_dataProc)
    return PVR_ERROR_SERVER_ERROR;

  return (m_dataProc->GetSignalStatus(signalStatus) ? PVR_ERROR_NO_ERROR : PVR_ERROR_SERVER_ERROR);
}

/*******************************************/
/** PVR Menu Hook Functions               **/

PVR_ERROR CallMenuHook(const PVR_MENUHOOK &menuhook, const PVR_MENUHOOK_DATA &item)
{
  return PVR_ERROR_NO_ERROR;
}

/** UNUSED API FUNCTIONS */
PVR_ERROR GetDriveSpace(long long *iTotal, long long *iUsed) { return PVR_ERROR_NOT_IMPLEMENTED; }
PVR_ERROR GetEPGForChannel(ADDON_HANDLE handle, const PVR_CHANNEL &channel, time_t iStart, time_t iEnd) { return PVR_ERROR_NOT_IMPLEMENTED; }
PVR_ERROR MoveChannel(const PVR_CHANNEL &channel) { return PVR_ERROR_NOT_IMPLEMENTED; }
int GetChannelGroupsAmount() { return 0; }
PVR_ERROR GetChannelGroups(ADDON_HANDLE handle, bool bRadio) { return PVR_ERROR_NOT_IMPLEMENTED; }
PVR_ERROR GetChannelGroupMembers(ADDON_HANDLE handle, const PVR_CHANNEL_GROUP &group) { return PVR_ERROR_NOT_IMPLEMENTED; }
int GetTimersAmount(void) { return 0; }
PVR_ERROR GetTimers(ADDON_HANDLE handle) { return PVR_ERROR_NOT_IMPLEMENTED; }
PVR_ERROR AddTimer(const PVR_TIMER &timer) { return PVR_ERROR_NOT_IMPLEMENTED; }
PVR_ERROR DeleteTimer(const PVR_TIMER &timer, bool bForce) { return PVR_ERROR_NOT_IMPLEMENTED; }
PVR_ERROR UpdateTimer(const PVR_TIMER &timer) { return PVR_ERROR_NOT_IMPLEMENTED; }
int GetRecordingsAmount(void) { return 0; }
PVR_ERROR GetRecordings(ADDON_HANDLE handle) { return PVR_ERROR_NOT_IMPLEMENTED; }
PVR_ERROR RenameRecording(const PVR_RECORDING &recording) { return PVR_ERROR_NOT_IMPLEMENTED; }
PVR_ERROR DeleteRecording(const PVR_RECORDING &recording) { return PVR_ERROR_NOT_IMPLEMENTED; }
bool OpenRecordedStream(const PVR_RECORDING &recording) { return false; }
void CloseRecordedStream(void) { }
int ReadRecordedStream(unsigned char *pBuffer, unsigned int iBufferSize) { return -1; }
long long SeekRecordedStream(long long iPosition, int iWhence /* = SEEK_SET */) { return -1; }
long long PositionRecordedStream(void) { return 0; }
long long LengthRecordedStream(void) { return 0; }
bool CanPauseStream(void) { return false; }
bool CanSeekStream(void) { return false; }
bool SeekTime(int time, bool backwards, double *startpts) { return false; }
time_t GetPlayingTime() { return 0; }
time_t GetBufferTimeStart() { return 0; }
time_t GetBufferTimeEnd() { return 0; }
const char* GetBackendHostname() { return ""; }
void SetSpeed(int) {};
void PauseStream(bool bPaused) {}
void DemuxReset(void) {}
void DemuxFlush(void) {}
int ReadLiveStream(unsigned char *pBuffer, unsigned int iBufferSize) { return 0; }
long long SeekLiveStream(long long iPosition, int iWhence /* = SEEK_SET */) { return -1; }
long long PositionLiveStream(void) { return -1; }
long long LengthLiveStream(void) { return -1; }
const char * GetLiveStreamURL(const PVR_CHANNEL &channel) { return ""; }
PVR_ERROR SetRecordingPlayCount(const PVR_RECORDING &recording, int count) { return PVR_ERROR_NOT_IMPLEMENTED; }
PVR_ERROR SetRecordingLastPlayedPosition(const PVR_RECORDING &recording, int lastplayedposition) { return PVR_ERROR_NOT_IMPLEMENTED; }
int GetRecordingLastPlayedPosition(const PVR_RECORDING &recording) { return -1; }
PVR_ERROR GetRecordingEdl(const PVR_RECORDING& recinfo, PVR_EDL_ENTRY edl[], int *size) { return PVR_ERROR_NOT_IMPLEMENTED; }
unsigned int GetChannelSwitchDelay(void) { return 0; }
}
