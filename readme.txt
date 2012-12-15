-----------------------------------------------------------------
 CV Drone (= OpenCV + AR.Drone)
 Copyright (C) 2012 puku0x
 https://github.com/puku0x/cvdrone
-----------------------------------------------------------------

INTRODUCTION
  This program is free software.

  You can redistribute it and/or modify it under the terms of the GNU General
  Public License as published by the Free Software Foundation; either version
  3 of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program. If not, please see http://www.gnu.org/licenses .

HOW TO INSTALL
  Please unzip "cvdrone-master.zip" in any directory.

HOW TO UNINSTALL
  Please delete the cvdrone folder.

BEFORE YOU BUILD
  You should install Visual Studio before you build CV Drone.
  CV Drone supports VC++2005/2008/2010/2012.
  To download VS, please see http://www.microsoft.com/visualstudio/eng/downloads .

HOW TO USE
  1. Open \build\vs20xx\test.sln
  2. Press F7 to build.
  3. Press F5 (or Ctrl+F5) to run.
  4. You can play around wiht OpenCV. See, sample codes in "src\samples".

FOR AR.DRONE 1.0 USERS
  Please update your AR.Drone's firmware to 1.10.14.

FOR AR.DRONE 2.0 USERS
  Please update your AR.Drone's firmware to 2.2.9.

FOR VS2010 USERS
  You can't build CV Drone by VS2010 after you installed VS2012.
  To build VS2010, you should uninstall ".Net Framework 4.5" and re-install "4.0".

LIBRARY DEPENDENCIES
  CV Drone uses following libraries.
  - OpenCV 2.4.3
    http://opencv.org/
  - FFmpeg 1.0 (git-11d695d)
    http://www.ffmpeg.org/
    http://ffmpeg.zeranoe.com/builds/
  - stdint.h for Visual C++
    http://www.kijineko.co.jp/node/63

  License files for each library can be found in the 'licenses' folder.

Thank you.