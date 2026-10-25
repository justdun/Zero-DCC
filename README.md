# Zero-DCC

(C) Justin M Dunlop (Zero1Guy) 2020, 2026

![ZeroDCC in use](/Photos/ZeroDCC_in_use.jpeg)

This project allows you to use your Hornby Zero 1 train control system from 1979 as a throttle for the DCC-EX platform.  It does not require any modifications to the Zero 1 control hardware.  You will need to remove any Zero 1 locomotive modules and fit DCC locomotive modules of your choosing.

The Zero DCC module provides a significant upgrade to Zero 1 users. To use this interface effectively, you must have a Zero 1 Master Controller and DCC equipped and programmed locomotives and optionally points/turnouts.  You can continue to use your Zero 1 accessory modules and MicroMimic console if desired.

The interface not only provides an upgrade allowing the use of your existing Hornby Zero 1 functions to command a modern DCC system, but it also provides additional DCC functions, not previously available using Hornby Zero 1.  For example control of DCC lights and sounds on an appropriately equipped locomotive.

Warning:  Locomotives equipped with Hornby Zero 1 Locomotive Modules will likely be damaged using this equipment. The only exception is ZTC decoders that are Hornby Zero 1, 12Volt DC and DCC compatible. 

This project includes:

* [Complete application manual](https://github.com/justdun/Zero-DCC/tree/main/Manual)
* [STL files for all components](https://github.com/justdun/Zero-DCC/tree/main/Case%20STL%20Files)
* [Decal masters for case](https://github.com/justdun/Zero-DCC/tree/main/Case%20Decal)
* [Files required as contents for the on-board SD Card](https://github.com/justdun/Zero-DCC/tree/main/SD%20Card%20Files)
* [Parts List](https://github.com/justdun/Zero-DCC/tree/main/Parts%20List)
* [Schematic](https://github.com/justdun/Zero-DCC/tree/main/Schematic)
* [Arduino INO file](https://github.com/justdun/Zero-DCC/tree/main/ZeroDCC%20code%20for%20Arduino%20Mega%20with%20WiFi)

For video demonstrations please visit:  https://www.youtube.com/@zero1guy

Special thanks to the team developing the DCC-EX platform without which this project would not be possible.

Thanks also to Geoff Bunza for his excellent, simple DCC sniffer circuit and PCB design.

![ZeroDCC Module Front](/Photos/ZeroDCC_front.jpg)
![ZeroDCC Module Back](/Photos/ZeroDCC_back.jpg)

Change notes:

Release Candidate 1.1
  - code improvement to catch potential corruption to incoming Zero 1 data
  - some arrays reduced in size to meet operational requirement and minimise memory use
