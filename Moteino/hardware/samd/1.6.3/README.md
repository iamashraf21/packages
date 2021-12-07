# Arduino Core for SAMD21 and SAMD51 CPU

This repository contains the source code and configuration files of the Arduino Core
for Atmel's SAMD21 and SAMD51 processor (used on the Arduino/Genuino Zero, MKR1000 and MKRZero boards).

## Installation on Arduino IDE

This core is available as a package in the Arduino IDE cores manager.
Just open the "Boards Manager" and install the package called:

* AREF must be tied to 3.3V for dac to work. This is a bug in the SAMD51 silicon. The only alternative is to explicitly set AREF to the internal Bandgap 1.1V in the sketch before using the DAC (hence max output will be 1.1V).
* USB host mode doesn't work yet

## Support

Any bugs should be reported in the forum:
https://lowpowerlab.com/forum

There is also a dedicated section of the Arduino Forum for general discussion and project assistance:
http://forum.arduino.cc/index.php?board=98.0

## Bugs or Issues

If you find a bug you can submit an issue here on github:

https://github.com/arduino/ArduinoCore-samd/issues

Before posting a new issue, please check if the same problem has been already reported by someone else
to avoid duplicates.

## License and credits

This core has been developed by Arduino LLC in collaboration with Atmel.

```
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
```
