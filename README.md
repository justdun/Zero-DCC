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
    
Release Candidate 1.2
  - now broadcasts DCC-EX commands on Serial 0,1,2 & 3 to allow for wired connections plus serial out to WiFi on combo boards.

Release Candidate 2.0
  - screen update:
    - splash and loco screens are drawn in code (ZeroSplash.h, ZeroPanel.h) - no BMP files needed on the SD card
    - loading status (settings, other settings banks, MicroMimic map file, recordings) shown
      under the logo on the splash screen and on the serial monitor
  - track map screen: Accessory 93 ON shows the layout from the SD card file Z_DCCMAP.TXT (see Manual v8 for details),
    Accessory 93 OFF returns to the loco screen. 
    If Z_DCCMAP.TXT is missing or has an error at start-up, Accessory 93 is ignored.
  - change to some of the Accessory 95 recording function logic (see Manual v8 for details)
  - Zero 1 / MicroMimic feed (relay) is off at start-up and goes on and off with DCC track power.
    In Accessory 90, choosing 1 (ON) shows TRACK POWER ON on black; DCC power and the Zero 1
    feed are only turned on when Acc 90 is closed with 90<- (the status line then turns red).
    Closing Acc 90 without choosing ON leaves the power as it was (see Manual v8 for details)

Release Candidate 2.1
  - WiFi status added. Where a Combination Arduino Mega / ESP8266 WiFi board is being used for the ZeroDCC host, connection status will be displayed, see [WiFi Bridge](https://github.com/justdun/Zero-DCC/tree/main/WiFi%20Bridge).

Release Candidate 2.2
  - Accessory 92 added to allow use of DCC points numbered from 90-2040
