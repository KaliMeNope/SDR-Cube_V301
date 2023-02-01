SDR Cube flash tools
====================

This is a package for flash programming and inspecting
on the SDR Cube. Most of the code can be used on other
dsPIC projects. The tools are compatible with the tools
described in Microchip Application Note AN1094, and both
ends of the serial link can be used interchangeably with
the AN1094 modules.

The tools must be regarded to be in the testing stage,
anything unexpected may happen. If it does, please report
it to the author (contact info in HTML documents).

There are two directories in the ZIP file:

1. dspicp
2. cubeflash

There is a HTML formatted documentation set in both directories.
The documentation is accessed by opening manual.html with 
a Web browser.

The cubeflash directory contains the programming agent
code for the SDR Cube hardware. It resides in the same
locations as the AN1094 agent (0x400 - 0xbff), so it
cannot be programmed in the same way as the application
code. The preferred way is the Microchip ICD tool.

The dspicp directory contains the tool to be run at the
controlling computer (usually PC). There are subdirectories
for different run environments:
 - linux-intel  Linux on an Intel-based computer
 - linux-ppc    Linux on a PowerPC-based computer
 - osx-intel    Mac OS X on an Intel-based computer
 - osx-ppc      Mac OS X on a PowerPC-based computer
 - windows      MS Windows (Intel-based, of course).
 
The ZIP archive format does not preserve well the POSIX
(others than Windows) run file attributes, so there is a 
tar/gzip archive containing the run files for Linux and
Mac OS X (all flavors).
 
There is a HTML formatted documentation set in both directories.
The documentation is accessed by opening manual.html with 
a Web browser.

The src subdirectory contains the sources with the Make file.
The program should compile and run on nearly all UNIXish
systems.

Copyright (C) 2013 by Tauno Voipio

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
