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
 *  along with XBMC; see the file COPYING.  If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <string>

class StringUtils
{
public:
  static std::string& Trim(std::string &str);
  static std::string& Trim(std::string &str, const char* const chars);
  static std::string& TrimLeft(std::string &str);
  static std::string& TrimLeft(std::string &str, const char* const chars);
  static std::string& TrimRight(std::string &str);
  static std::string& TrimRight(std::string &str, const char* const chars);
};
