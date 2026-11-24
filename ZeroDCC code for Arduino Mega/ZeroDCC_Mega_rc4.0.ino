//  Zero DCC for Arduino Mega
//
//  Release Candidate 4.0
//
//  Hornby Zero 1 decoder and interface to a secondary DCC-EX device via serial connection
//
//  Written and Copyright (C) by Justin M Dunlop (Zero1Guy) 2020-2026
//
//  Please do not distrute without written permission from the author
//
//  NB:  Pin configurations below will need to be followed and/or edited to suit your set up
//
//  Refer to https://github.com/justdun/Zero-DCC for further details and instructons
//    and to https://www.youtube.com/@zero1guy for video demonstrations 
//
//  Update notes:
//  RC4.0 - WiFi app control, stage 2 (with ESP "WiFi 2.0 stage 2"): phone commands for a slot's
//          address (<t ...>), track power (<1>/<0>) and <!> come from the ESP on Serial3 and are handled
//          like Master keys. The Master's own changes are tracked separately (Z1_MasterSpeed/Direction),
//          so its 200 ms repeats don't undo a phone's change - last change wins. The slot list goes to
//          the ESP (@ZS line) every 10 s and when it changes; without DCC-EX, loco and power changes go
//          to the phones as @<l ...> / @<p..> lines.
//        - stage 2b: @<l ...> / @<p..> are sent whether or not DCC-EX is linked (with ESP "stage 2b")
//        - stage 3 (with ESP "WiFi 2.0 stage 3"): phone accessories 1-89 (<a n state>) are handled as a
//          Zero 1 point key (DCC-EX, Zero 1 bus, map, recording); phone functions (<F cab fn state>) go
//          through Loco_Fn_Send like Acc 98/99 and the touch screen. @<l ...> now carries the loco's
//          function bits, and is also sent when a slot loco's function changes.
//        - Zero 1 locos 1-16 are now DCC cab IDs 1-16: the Zero 1 bus drives Zero 1 loco n for whichever
//          slot holds address n (Acc 96/97), not for slot n. A slot holding any other address drives
//          no Zero 1 loco. A phone can also drive address 1-16 when no slot holds it.
//        - a loco leaving a slot (Acc 96, Acc 97, touch menu) is no longer stopped: it keeps running
//          until it's stopped from a phone, by emergency stop / track power off, or from a slot again.
//        - phone turnouts / accessories: the ESP sends every one to DCC-EX itself; for 1-89 Zero DCC does
//          the Zero 1 side only (Zero 1 bus, map, recording) - no second <a> to DCC-EX
//        - stage 4: recording cabs 9991-9999 = recordings 1-9, 9990 = recording 10: F1 Play, F2 Record
//          (press twice within 3 s), F0 Stop; the apps' buttons follow Acc 95 too. While recording (however
//          started), locos in no slot from the apps come through Zero DCC and are recorded (as slot 0).
//          A phone emergency stop also stops a playback.
//          Phone DCC accessories / turnouts 90-2040 now come to Zero DCC too (the ESP still sends them to
//          DCC-EX itself): they update the map, and are recorded while recording. Map taps on any point
//          1-2040 update the phones' turnout buttons.
//        - last roster kept in EEPROM: the 16 CAB IDs and the roster number (1-10) are saved to the Mega's
//          EEPROM whenever they change (Acc 96, Acc 97, menu). At power-up the remembered roster number's
//          file (Z_DCC_n.txt) is loaded from the SD card; if there is no card or that file is missing,
//          the CAB IDs saved in EEPROM are used. Blank / invalid EEPROM: Z_DCC_1.txt or defaults as before.
//        - Rail Drive module: the Zero 1 bus is made by a separate Arduino Nano (ZeroRailDrive.ino) on
//          Serial2, with D8 as its "track power on" line. Light pen stays on the Mega (D2).
//        - the Mega no longer makes the Zero 1 bus itself (H-bridge on pins 6/7, sense A8 removed):
//          ZDCC_Z1BUS 1 = Rail Drive Nano on Serial2, 0 = none. Light pen: ZDCC_LIGHTPEN.
//        - ZDCC_DCCEX_SERIAL 1: DCC-EX wired to Serial1 instead of over WiFi (needs ESP "WiFi 2.0 stage 4b").
//          Zero DCC's commands go to Serial1 only; phones' DCC-EX commands come from the ESP as "&<...>"
//          and go to Serial1; everything DCC-EX sends back is passed to the phones as "@<...>". The ESP is
//          told with "@DX S" and stays off DCC-EX's WiFi. The status dot shows the serial link: white =
//          waiting for DCC-EX, green = DCC-EX answering, yellow = DCC-EX not answering (pinged with <#>).
//        - every accessory 1-89 change goes to the ESP as "@A n state" (stage 3b), so a phone's turnout
//          on that address follows the Zero 1, the touch screen and the light pen
//        - stage 1 (with ESP sketch "WiFi 2.0"): the ESP runs Zero DCC's own access
//          point for phones and finds DCC-EX by itself. Status dot: new state 4 (blue) = DCC-EX not
//          found, Zero 1 only. The ESP's messages (#ZL lines) and the number of phones connected are
//          shown in the serial log. Works unchanged with bridge v1.1.
//
//  RC3.2 - Zero 1 input fix: stray loco speeds / directions in bursts (worse with the Zero 1 bus on).
//          The input ring buffer was emptied one edge per loop() with a 1 ms delay each, so with the
//          extra work of the Zero 1 bus it could fall behind the ~720 edges a second and drop messages.
//        - edges now timed on RISING (no pin read in the interrupt), 1024-entry ring, every waiting
//          edge processed each loop() with no delay, and an overrun drops the message being read
//        - stricter message check: loco speeds must be 1-15, accessory digits 0-9 and the unused
//          'F2 = 0, F3 not 0' pattern is ignored
//        - a loco message is used only when it arrives the same twice in a row (0.2 s apart)
//        - the Zero 1 bus interrupts (ZeroZ1Bus.h) can now be interrupted, so they no longer delay
//          the time-stamping of Zero 1 input edges
//        - current sense: Z1BUS_IS_OFFSET_V (A8 volts at no load) is subtracted, and the report shows the
//          average and peak current plus the A8 average voltage, for calibrating a new IBT-2
//        - each accessory command is sent 4 times on the Zero 1 bus (was 2: points sometimes didn't fire)
//
//  RC1.1 - code improvement to catch potential corruption to incoming Zero 1 data
//        - some arrays reduced in size to meet operational requirement and minimise memory use
//
//  RC1.2 - now broadcasts DCC-EX commands on Serial 0,1,2 & 3 to allow for wired connections plus serial out to WiFi on combo boards.
//
//  RC2.0 - Screen update:
//        - splash and loco screens are drawn in code (ZeroSplash.h, ZeroPanel.h) - no BMP files needed on the SD card
//        - loading status (settings, other settings banks, MicroMimic map file, recordings) shown
//          under the logo on the splash screen and on the serial monitor
//        - track map screen: Accessory 93 ON shows the layout from the SD card file Z_DCCMAP.TXT (DccMap.h),
//          Accessory 93 OFF returns to the loco screen. Points (Accessories 1-89) show green/red dots.
//          If Z_DCCMAP.TXT is missing or has an error at start-up, Accessory 93 is ignored.
//        - Zero 1 / MicroMimic feed (relay) is cut whenever DCC track power is off, and only reconnected
//          when track power is on and no Accessory 90-99 function is running
//        - while track power is off no loco, function or turnout commands are sent to DCC-EX (so it can't
//          get out of step with the layout or with JMRI etc.), and map points and signals don't change.
//          Still sent: power and track-mode commands (Acc 90), programming-track writes (Acc 91) and the
//          speed-0 stop sent when a cab slot is reassigned (Acc 96/97).
//        - change to some of the Accessory 95 logic
//        - while track power is off no loco or turnout commands are sent to DCC-EX (so it can't get out
//          of step with the layout or with JMRI etc.), and map points and signals don't change.
//          Still sent: power and track-mode commands (Acc 90), programming-track writes (Acc 91), the
//          speed-0 stop sent when a cab slot is reassigned (Acc 96/97) and loco functions (Acc 98/99).
//        - Zero 1 / MicroMimic feed (relay) is off at start-up and goes on and off with DCC track power.
//          In Accessory 90, choosing 1 (ON) shows TRACK POWER ON on black; DCC power and the Zero 1
//          feed are only turned on when Acc 90 is closed with 90<- (the status line then turns red).
//          Closing Acc 90 without choosing ON leaves the power as it was.
//          NB: ZeroDCC only knows about power switched with Accessory 90 (DCC-EX does not report back),
//          so these blocks follow the status line on the screen. Every power change is logged on the
//          serial monitor.
//
// RC 2.1 - WiFi combo boards: a coloured dot at the left of the status line shows the ESP8266 bridge's
//          status (white = looking for WiFi, yellow = no DCC-EX, green = connected, grey = stopped
//          reporting, magenta = data on RX3 but not from this bridge, brief black blink = commands
//          arriving from the Mega). No dot = nothing at all received from the ESP on RX3.
//
// RC 2.2 - Accessory 92: points / accessories numbered over 89.
//          92->            start (Zero 1 / MicroMimic feed cut, as for the other 90-99 functions)
//          n<-             enter the accessory number one digit at a time (10 = 0), like Acc 96
//          any key ->      accept the number and go to command mode (a number of 0 is ignored)
//          then any key <- sends <a n 1>, any key -> sends <a n 0> to the chosen accessory,
//          as many times as needed
//          92->            start again with a new number
//          92<-            end Accessory 92
//          Commands follow the same rules as normal points: nothing is sent while track power is off,
//          they are recorded during an Accessory 95 recording, and they update the map.
//        - points (Acc 1-89 and Acc 92) now send DCC accessory commands <a address 0/1> instead of
//          turnout commands <T id 0/1>, so they work with track-connected accessory decoders with
//          nothing to define in DCC-EX. Addresses are linear DCC accessory addresses 1-2040 (2041-2044 are reserved for broadcast).
//          Recordings made with earlier versions still contain <T ...> and are sent as recorded.
//        - track map (Z_DCCMAP.TXT): points and signals can use accessory numbers 1-2040
//          (e.g. HLT1050, VS210A). Numbers too wide for the space beside the track are drawn in a
//          smaller font, on two lines if needed. Requires the updated DccMap.h.
//        - fix: recorded point commands (Acc 1-89) no longer leave a blank line in the recording file,
//          which put playback out of step after the first point change
//        - Accessory 97 (load a settings bank) now works while a recording is playing back (Acc 95).
//          Before, 97-> was accepted but the bank number was ignored, leaving the Zero 1 feed cut
//          until 97<-. During playback, the speed-0 stop for the old bank's locos is skipped for any
//          loco the recording is currently driving, so the recording carries on undisturbed.
//        - fix: during playback the speed arrows are drawn on the row where each recorded loco
//          address is assigned now. Before, they went to the cab slot used when the recording was
//          made, so after an Acc 96/97 change they showed on a loco that wasn't moving.
//
// RC 2.3 - Touch screen (ZeroTouch.h): on the Accessory 93 map, tap a point or signal to flip it.
//          ON sends <a n 0>, OFF sends <a n 1>, and the dots update at once - the same as using the
//          Zero 1: nothing is sent while track power is off, and taps are recorded during an
//          Accessory 95 recording. One tap = one change (lift the pen before the next tap counts).
//          Taps on plain track, blank squares or the status strip are ignored; a near miss next to
//          a point or signal still finds it. Taps on the loco screen do nothing (yet).
//          Touch needs calibrating for each screen: run TouchCal_395 and copy its five TOUCH_...
//          lines into ZeroTouch.h. TOUCH_ENABLED 0 in ZeroTouch.h switches touch off.
//        - point / accessory commands from the Zero 1, Accessory 92 and the touch screen now all go
//          through one routine (Send_Accessory), so they behave identically
//        - runs on either screen: ZDCC_SPI_DISPLAY 0 = 3.95" parallel module (MCUFRIEND_kbv),
//          1 = 4.0" ST7796S SPI module (ZeroST7796.h, wired as ZeroDCC v6.96 SPI). The touch pins
//          and calibration in ZeroTouch.h follow the same setting.
//
// RC 2.4 - Touch screen, more controls (positions as the screen is viewed, status strip at the top):
//          Both screens
//        - TRACK POWER ON / OFF in the status strip is a button: tap to turn track power on or off
//          (same as Acc 90 ON / OFF - a quick way to stop everything; not recorded)
//        - red arrows at the left and right edges, half way down, switch between the loco screen and
//          the map (same as Acc 93 on / off). Not shown if there is no map file.
//          Map
//        - points: tap nearer the dot of the route you want (straight or diverging)
//        - signals: tap right of / below the dot for red, left of / above it for green
//          (horizontal track: left / right, vertical: above / below, diagonal: either)
//        - every tap is sent and recorded during an Acc 95 recording, even if the point is already set
//          that way, so a recording always sets the whole route
//          Loco screen
//        - tap the CAB ID: headlight F0 on / off (as Acc 98 <-); the CAB ID has a yellow background
//          while the light is on (also when it's switched with Acc 98 / 99)
//        - tap the slot number in the red strip: whistle F2 on / off (as Acc 98 ->)
//        - tap the bottom chevron: change direction
//        - tap chevrons 2-15: set the speed (chevron 2 = 9 ... chevron 15 = 126); tap the top lit
//          chevron again to stop. The Zero 1's own setting for that slot isn't changed, so the next
//          change on the Zero 1 for that slot takes over again.
//        - loco speed / direction taps follow the same rules as the Zero 1: not sent while track power
//          is off, recorded during Acc 95. Light / whistle taps are sent even with power off (as Acc 98).
//        - Acc 98 / 99 functions now go through the same routine as the touch screen (Loco_Function)
//        - fix: loco IDs could be drawn turned 90 degrees after Acc 97 / 96 - the loco screen routines
//          now set their own orientation
//        - fix: a settings bank file with more than 16 lines could overwrite memory; extra lines are ignored
//        - during playback, recorded loco functions update the function state of the row(s) where that loco
//          is assigned now, so the CAB ID turns yellow / back as the recording switches the headlight
//          (and the next Acc 98 / 99 / touch toggles from the recording's state)
//        - Accessory 96 (assign a loco to a cab slot) now works while recording or playing back (Acc 95).
//          Before, 96-> was accepted but the digits were ignored, leaving the Zero 1 feed cut until 96<-.
//          During playback, the speed-0 stop for the loco leaving the slot is skipped if the recording is
//          driving it. Recordings store loco addresses, so a slot change doesn't upset them.
//        - Accessory 97 (load a settings bank) now also works while recording, as it already did during playback.
//          The speed-0 stops sent when slots change (Acc 96 / 97) are now written to the recording, so
//          playback stops the same locos. After a bank change the rows show the new locos stopped.
//
// RC 2.5 / 3.0 - Menu screen (ZeroMenu.ino - a second tab, keep it in the same folder as this file)
//        - a third screen after the map (arrows: loco screen -> map -> menu -> loco screen; the map is
//          skipped if there is no map file). Grey background, status strip and arrows as the map.
//        - one button for each of Accessory 90-99. Choosing one does exactly what the Zero 1 key does
//          (starts the function and cuts the Zero 1 / MicroMimic feed); its Exit button ends it as 9x<-
//          (feed restored, Acc 90 power on applied). The Zero 1 keys keep working alongside, and using
//          the keys never opens the menu. Leave the menu with the arrows; anything still running keeps
//          running and the menu shows the same page when you come back.
//          90 Power - Power on / the three track modes / Exit (current mode and power shown in yellow)
//          91 DCC Programming - not finished yet (shown, but the two options do nothing)
//          92 Extended Point Setting - number pad with <- / ->; X deletes a digit, or the whole number
//             once a direction has been sent
//          93 eMicroMimic - goes to the map;  94 Reserved - does nothing
//          95 Record / Playback - playback list from the SD card (with Stop Playback), record list 1-10
//             showing which exist, "Are you sure?", then OK / End Recording
//          96 Assign CAB IDs - touch a CAB ID, enter the new one on the number pad, Select (saved at once)
//          97 Choose roster - rosters 1-10: an existing one is loaded as Acc 97, an empty one is created
//             from the locos in use now (no stops sent, the locos don't change)
//          98 Light / Whistle - a light bulb (F0) and music note (F2) under each CAB ID
//          99 DCC functions - choose a loco, then F0-F31 (yellow = on)
//        - loco functions now go up to F31 (F29-F31 from the menu only; the Zero 1 keys reach F28)
//        - Acc 93<- now returns to the loco screen from the menu as well as from the map
//        - the Accessory 90-99 start / end code is shared by the Zero 1 keys and the menu (Acc_Start /
//          Acc_End), as are recording / playback start (Start_Recording / Start_Playback) and slot
//          assignment (Assign_Slot / Save_Roster)
//
// RC 3.0 - fix: whistle sometimes silent in playback. Two causes:
//          1. Acc 98 -> toggled F2, so a whistle could be left ON; a recording then replayed "F2 on" to a
//             decoder that was already on, and nothing sounded. The whistle is now momentary: F2 on, then
//             off by itself after WHISTLE_MS (1 s); both are sent and recorded. (WHISTLE_MOMENTARY false
//             = the old toggle.)
//          2. function state was kept per cab SLOT, so after Acc 96 / 97 a slot carried the old loco's
//             state over to the new one. It is now kept per loco ADDRESS (up to 24 locos), so it follows
//             the loco to any slot or roster; playback updates it the same way.
//        - Acc 99 with the Zero 1 keys: 1-31 = F1-F31, 32 = F0 (was 29 = F0, F1-F28)
//        - Acc 98 / 99 menu pages and the loco screen show each loco's real light / function state
//          (as far as ZeroDCC knows - changes made from JMRI or another throttle aren't seen)
//        - menu: the accessory's name at the top of every page; number pads laid out like a calculator
//          (7 8 9 on top, as the Zero 1 keys)
//        - menu Acc 91 DCC Programming:
//          1-> loco CAB ID: warning, track power off + A: Main / B: Program, remove all locos, clean the
//              program track and place the loco, enter 1-99, confirm, <W n>, wait 15 s, done
//          2-> accessory module ID: the same steps for a module on the program track, 1-2040. Modules
//              answer to a block of 4 numbers: block (n-1)/4+1.
//          For both: <1 PROG> (program track power only), 1.5 s later the first write, then a
//          15 s wait after the last one, then <0 PROG>. If Acc 91 is ended (91<-) part way, <0 PROG> is sent then.
//          Accessory modules: Group = CV1 + 64 x CV9 (R8247 manual). Three writes 5 s apart, each one
//          valid whatever the module held before (the R8247 refuses a write that leaves CV1 = 0 with
//          CV9 = 0 - writing CV9 = 0 first failed when coming down from e.g. 253-256):
//          CV1 = new value (1 if it is 0), CV9 = new value, CV1 = new value.
//          Done page: numbers 90-99 can't be switched with the Zero 1 keys (they are ZeroDCC's own
//          functions) - a note says to test with Acc 92 or the map.
//          While the program track is on, the status strip shows TRACK POWER PROG (red on black).
//          After 2-> a question asks which module: Hornby R8216, Hornby R8247 or Other.
//          Hornby R8216: the Hornby Elite manual says it "must be programmed in Reg mode"; DCC-EX's <W>
//          is Direct mode, which it ignores. DCC-EX has no Register mode, so ZeroDCC sends the register
//          write as raw program-track packets: <P 0 78 gg> x3 (register 1 = group gg, in hex; DCC-EX adds
//          the checksum and sends each 4 times). Register 1 only, so IDs 1-252. Sequence: <= B MAIN>,
//          0.3 s <1 B>, charge 30 s (status strip: TRACK B CHARGING), <= B PROG>, 0.3 s <1 PROG>,
//          0.3 s write, 15 s, <0 PROG>. (MENU_R8216_TRIES > 1 repeats the write, with 2 s, Main + 30 s
//          recharge in between. First tried as 150 s charge and 3 writes, which worked.)
//          Ended part way: <0 B> + <= B PROG> if it was charging.
//          Hornby R8247: after the ID, an "R8247 Setup" page sets each channel (its 4 numbers):
//          Point - Short (default, CV3-6 = 1 = 0.1 s), Point - Long (5 = 0.5 s), Light - Continuous (0),
//          or Leave (no change). CV33 = 0 (Standard mode) is always written first, then the changed
//          channels, then the address. The confirm page lists the settings.
//          Block of 4: after an accessory module's number is keyed in, a page explains that modules use
//          fixed blocks of 4 and shows the block (e.g. 50 -> 49 - 52, Channel 1 = 49 ... Channel 4 = 52,
//          the number keyed in shown in red). OK goes on, Change goes back to the number. The confirm,
//          programming and done pages show the block ("49 - 52") instead of the number keyed in.
//        - fix: DCC-EX ignored "<= B PROG >" (B stayed DC 999, so there was no programming track and
//          Acc 91 failed): the track mode commands are now sent without the space before '>'
//        - start-up: track power off + A: Main / B: Program are sent to DCC-EX at start-up and again
//          3 s and 8 s later (DCC-EX may still be booting, or start with the tracks on or joined)
//        - Set_Track_Mode: track power off + track mode, shared by Acc 90 and the menu
//        - fix: boards with no WiFi bridge (e.g. Mega Pro) showed a magenta dot - an unconnected RX3 pin
//          picks up noise. RX3 now has its pull-up on, and magenta needs 3 complete lines of other data;
//          with nothing connected no dot is shown.
//        - touch screen check at power-up: the touch controller must answer correctly, and an untouched
//          panel must read as untouched. If there is no touch screen (or the panel is faulty) the arrows
//          and the menu screen don't appear and touch isn't read - the Zero 1 keys work as always and
//          Acc 93 still switches between the loco screen and the map. The splash screen says which.
//
// RC 3.1 - Zero 1 accessory bus and MicroMimic light pen (ZeroZ1Bus.h - keep it in the same folder)
//        - Zero DCC now drives the separate Zero 1 accessory wiring (and the MicroMimic's Track In)
//          itself, through an H-bridge module, with a full Zero 1 waveform (MERG TB CC01/1): 10 ms
//          slots, one data period then four power periods. Every point command for accessories 1-89 -
//          Zero 1 keys, touch screen, Accessory 92, and recordings played back with Accessory 95 - is
//          sent to the Zero 1 accessory modules and the MicroMimic as well as to DCC-EX (twice, 50 ms
//          apart). 90-99 are never sent (Zero DCC's own functions). The bus is on only while DCC
//          track power is on. The Zero 1 master's output must no longer be connected to that wiring.
//        - Zero 1 locos: the other data periods carry speeds and directions for Zero 1 locos 1-16
//          (Zero 1 loco n = cab slot n), so the same output can run a track with genuine Zero 1 locos.
//          Every loco command Zero DCC sends to DCC-EX for slot n - Zero 1 controls, touch screen,
//          slot changes (Acc 96 / 97) and recordings played back - is also sent to Zero 1 loco n.
//        - overload trip: the H-bridge's current sense (BTS7960 R_IS / L_IS) is watched by the ADC;
//          over Z1BUS_TRIP_AMPS for 2 ms switches the Zero 1 bus off at once, retried every 3 s
//          (like the Zero 1 master's cut-out). Settings at the top of ZeroZ1Bus.h.
//        - MicroMimic light pen: the MicroMimic sends standard Zero 1 accessory messages on its Rail
//          Drive socket (pin 3) when the pen hits an LED. They are read on RAILDRIVE_PIN (through a
//          6N137), checked (checksum, format), and handled exactly like the same Zero 1 point key:
//          DCC-EX, the map, recordings and the Zero 1 bus. A pen miss (accessory 0) is ignored, and
//          pen hits are ignored while an Accessory 90-99 function is waiting for keys.
//

#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>

// ---- Which screen? ----
//   0 = 3.95" LCD module for Mega2560 (parallel, plugs onto the Mega; touch on SPI CS 53 / IRQ 44)
//   1 = 4.0" ST7796S SPI module (Mega Pro test rig; display CS A5, DC A3, RESET A4, LED A0, SPI 51/52;
//       touch T_CLK 22, T_CS 24, T_DIN 26, T_DO 28, T_IRQ 30)
// The touch pins and calibration in ZeroTouch.h follow this setting.
#define ZDCC_SPI_DISPLAY 0

#if ZDCC_SPI_DISPLAY
#include "ZeroST7796.h"
ZeroST7796 tft(A5, A3, A4, A0);             // CS, DC/RS, RESET, LED - as ZeroDCC v6.96 SPI
#else
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
#endif

#include "ZeroPanel.h"                      // loco screen background (Zero 1 panel)
#include "ZeroSplash.h"                     // start-up splash screen
#include "DccMap.h"                         // track map screen (Accessory 93)
#include "ZeroTouch.h"                      // touch screen: tap points / signals on the map
#define ZDCC_Z1BUS 1                        // Zero 1 bus: 1 = Rail Drive Nano on Serial2 (D16 TX2 / D17 RX2),
                                            //                 D8 = its "track power on" line; 0 = none
#define ZDCC_LIGHTPEN 1                     // MicroMimic light pen (Rail Drive socket) on D2: 1 = on, 0 = off
#define ZDCC_DCCEX_SERIAL 1                 // DCC-EX link: 0 = over WiFi (the ESP joins DCC-EX's access point)
                                            //   1 = wired to DCC-EX on Serial1 (D18 TX1 / D19 RX1); the WiFi is
                                            //       then only Zero DCC's own access point for phones / JMRI.
                                            //       DCC-EX needs #define SERIAL3_COMMANDS (or SERIAL2_COMMANDS)
                                            //       in its config.h - its Serial1 is its own WiFi board
#include "ZeroZ1Bus.h"                      // Zero 1 accessory bus (H-bridge) + MicroMimic Rail Drive (light pen)
#include <Fonts/FreeSans9pt7b.h>            // loading status lines on the splash screen

const unsigned long Z1_SplashTime = 3000;   // minimum time the splash screen stays up (ms)
byte Z1_SplashLine = 0;                     // next loading-status line on the splash screen

byte Z1_Pin = 3;                            // NB: This is the pin that receives data from the optocoupler board
byte Z1_Relay_Pin = 5;                      // NB: This is the pin that sends a trigger to the dual relays to cut Zero 1 signal to Zero 1 accessories and MicroMimic
byte Z1_SD_Card_Pin = 48;                   // NB: For the documented TFT screen with built in SD Card Reader this is the default pin setting for the SD card reader, this will need to change if using a different SD card reader
String Z1_Settings_File = "Z_DCC_";
String Z1_Recording_File = "Z_DCCR";
File Z1_PlaybackFile;
bool Z1_PlaybackCommandLoaded = false;
int Z1_PlaybackLoco[17];                    // loco address the playback is running in each slot (0 = stopped / none)
byte Z1_Count = 0;
byte Z1_Data[34];
bool Z1_FrameBad = false;                    // a pulse of the wrong length arrived in this message: ignore it
uint16_t Z1_RejectCount = 0;                 // messages rejected (bad pulse or checksum) - noise on the Zero 1 input
byte Z1_Tag;
int Z1_LocoNo[ ] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
byte Z1_LocoSpeed[ ] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} ;
byte Z1_LocoDirection[ ] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} ;
// RC4.0: the last speed / direction the Zero 1 Master itself sent for each slot. Only a change from
// these counts as a Master change, so a phone's setting (Z1_LocoSpeed) isn't undone by the Master's repeats
byte Z1_MasterSpeed[17] = {0};
byte Z1_MasterDirection[17] = {0};
byte Z1_ShownSpeed[17];                     // speed / direction currently shown on the loco screen:
byte Z1_ShownDirection[17];                 // the Zero 1 controls, or a recording while it plays back
byte Z1_Accessory99Flag = 0;
byte Z1_Accessory99Function = 0;
byte Z1_Accessory98Flag = 0;
byte Z1_Accessory97Flag = 0;
byte Z1_Accessory97Bank = 1;

// ---- last roster in EEPROM (address 0): marker, roster number, 16 CAB IDs, checksum
struct RosterEE { uint16_t magic; uint8_t bank; int16_t ids[16]; uint8_t sum; };
const uint16_t ROSTER_EE_MAGIC = 0x5A31;
const int ROSTER_EE_ADDR = 0;
bool Roster_EE_Valid = false;               // a roster was found in EEPROM at power-up
RosterEE Roster_EE;

uint8_t Roster_EE_Sum(const uint8_t *p) {    // over everything before 'sum' (bytes, so no struct in the prototype)
  uint8_t x = 0x5A;
  for (size_t i = 0; i < offsetof(RosterEE, sum); i++) x = (x << 1 | x >> 7) ^ p[i];
  return x;
}

bool Roster_EE_Read() {
  EEPROM.get(ROSTER_EE_ADDR, Roster_EE);
  Roster_EE_Valid = Roster_EE.magic == ROSTER_EE_MAGIC && Roster_EE.bank >= 1 && Roster_EE.bank <= 99 &&
                    Roster_EE.sum == Roster_EE_Sum((const uint8_t *)&Roster_EE);
  return Roster_EE_Valid;
}

void Roster_EE_Use() {                      // the CAB IDs saved in EEPROM -> the 16 slots
  for (byte k = 1; k <= 16; k++) Z1_LocoNo[k] = Roster_EE.ids[k - 1];
  Z1_Accessory97Bank = Roster_EE.bank;
}

// called from loop(): save the roster to EEPROM 2 s after it last changed (only changed bytes are written)
void Roster_Watch() {
  static long saved = -1, pending = -1;
  static unsigned long changedAt = 0;
  long h = Z1_Accessory97Bank;
  for (byte k = 1; k <= 16; k++) h = h * 31 + Z1_LocoNo[k];
  if (h == saved) { pending = -1; return; }
  if (h != pending) { pending = h; changedAt = millis(); return; }
  if (millis() - changedAt < 2000) return;
  RosterEE r;
  r.magic = ROSTER_EE_MAGIC;
  r.bank = Z1_Accessory97Bank;
  for (byte k = 1; k <= 16; k++) r.ids[k - 1] = Z1_LocoNo[k];
  r.sum = Roster_EE_Sum((const uint8_t *)&r);
  EEPROM.put(ROSTER_EE_ADDR, r);            // put() only writes bytes that differ
  saved = h; pending = -1;
  Serial.print(F("Roster "));
  Serial.print(Z1_Accessory97Bank);
  Serial.println(F(" saved to EEPROM"));
}
byte Z1_Accessory96Flag = 0;
byte Z1_Accessory95Flag = 0;
String Z1_Accessory95File;
byte Z1_Accessory92Flag = 0;                // 0 off, 1 entering the accessory number, 2 command mode
unsigned int Z1_Accessory92Number = 0;      // accessory chosen in Accessory 92 (1-2040)
byte Z1_Accessory91Flag = 0;
byte Z1_Accessory90Flag = 0;
// Loco function state F0-F31, kept per loco ADDRESS (not per cab slot), so a loco's light etc. stays with it
// when it moves to another slot or roster (Acc 96 / 97). Up to 24 locos are remembered.
struct LocoFns { int addr; uint32_t fn; };
const byte FN_LOCOS = 24;
LocoFns Z1_Fn[FN_LOCOS];
byte Z1_FnNext = 0;

// Whistle (Acc 98 -> and the touch screen): sounds for WHISTLE_MS then switches itself off, so every
// press is heard, live and in playback (both the on and the off are recorded).
// false = the old behaviour: each press toggles F2 on / off.
const bool WHISTLE_MOMENTARY = true;
const unsigned int WHISTLE_MS = 1000;
struct WhistleOff { int addr; unsigned long at; };
WhistleOff Z1_Whistle[4];
int LocoNumber;
byte j;

String nextcommand;
unsigned long filesize;
unsigned long fileposition;
unsigned long commandtime;
unsigned long rightnow;
unsigned long Z1_PulseDuration;
unsigned long Z1_LastCommandTime;
int AccessoryNo;
byte Z1_TrackType = 2;                      // last track mode set by Accessory 90 (2, 3 or 4)
bool Z1_TrackPowerOn = false;
bool Z1_PowerPending = false;               // ON chosen inside Accessory 90 - applied when Acc 90 is closed with 90<-
bool Z1_SDOK = false;                       // SD card started at power-up
unsigned long Z1_SyncAt = 0;                // start-up: when to send the track mode to DCC-EX (see Startup_Sync)
byte Z1_SyncLeft = 0;
byte Z1_ProgPower = 0;                      // Acc 91: 1 = program track powered on its own (<1 PROG>), 2 = R8216 charging on TRACK B
int  Z1_RecordNumber = 0;                   // recording being made (Acc 95)
int  Z1_PlaybackNumber = 0;                 // recording being played back (Acc 95)

// Screens: the arrows step loco -> map -> menu -> loco (the map is skipped if there's no map file)
const byte SCREEN_LOCO = 0;
const byte SCREEN_MAP  = 1;
const byte SCREEN_MENU = 2;
byte Z1_Screen = SCREEN_LOCO;

// WiFi bridge status (combo boards: the ESP8266 reports "#ZB state count" on Serial3)
byte Z1_BridgeState = 0;                    // 0 never heard, 1 looking for DCC-EX, 2 can't reach DCC-EX,
                                            // 3 connected, 4 DCC-EX not found (Zero 1 only, WiFi 2.0),
                                            // 8 data on RX3 but no status lines, 9 stopped reporting
int Z1_BridgePhones = -1;                   // phones connected to Zero DCC's WiFi (WiFi 2.0), -1 unknown
int Z1_BridgeReported = 0;                  // the last state the ESP reported (5 = it knows DCC-EX is on serial)
bool Z1_WiFi2 = false;                      // the ESP runs WiFi 2.0 (takes @ lines, sends phone commands)
unsigned int Z1_BridgeCount = 0;            // commands the bridge has received from the Mega
bool Z1_BridgeCountKnown = false;
unsigned long Z1_BridgeHeard = 0;
unsigned long Z1_BridgeFlashUntil = 0;
bool Z1_BridgeFlashing = false;

// Zero 1 input: the interrupt stores the time of each rising edge; Data_Cycle() works through them
const uint16_t Z1_EDGES = 1024;             // ring size (power of 2): about 1.4 s of signal
volatile uint16_t Z1_EdgeTime[Z1_EDGES];
volatile uint16_t Z1_EdgeHead = 0;          // written by the interrupt
uint16_t Z1_EdgeTail = 0;                   // read by Data_Cycle()
bool Z1_EdgePrimed = false;                 // have a previous edge to measure from
uint16_t Z1_LastEdge = 0;
uint16_t Z1_OverrunCount = 0;

class MultiSerial : public Print {
  public:
    size_t write(uint8_t c) {
      Serial.write(c);
      Serial1.write(c);
#if !ZDCC_Z1BUS
      Serial2.write(c);                       // (Serial2 is the Rail Drive Nano's link when ZDCC_Z1BUS 1)
#endif
#if !ZDCC_DCCEX_SERIAL
      Serial3.write(c);                       // DCC-EX over WiFi: through the ESP
#endif
      return 1;
    }
    size_t write(const uint8_t *buffer, size_t size) {
      Serial.write(buffer, size);
      Serial1.write(buffer, size);
#if !ZDCC_Z1BUS
      Serial2.write(buffer, size);
#endif
#if !ZDCC_DCCEX_SERIAL
      Serial3.write(buffer, size);
#endif
      return size;
    }
};

MultiSerial DCC_Out;

Sd2Card card;
SdVolume volume;
SdFile root;
File myfile;

size_t readField(File* file, char* str, size_t size, char* delim) {
  char ch;
  size_t n = 0;
  while ((n + 1) < size && file->read(&ch, 1) == 1) {

    if (ch == '\r') {
      continue;
    }
    str[n++] = ch;
    if (strchr(delim, ch)) {
      break;
    }
  }
  str[n] = '\0';
  return n;
}

#define errorHalt(msg) {Serial.println(F(msg)); while(1);}

// One loading-status line: printed to the serial monitor and shown under the
// logo on the splash screen (landscape, left-aligned with the logo, one line per call)
void Splash_Status(const char *msg) {

  Serial.println(msg);

  int16_t x1, y1;
  uint16_t w, h;
  tft.setRotation(1);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextColor(0x0000);
  tft.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(ZS_Y0 - x1, 222 + Z1_SplashLine * 22);      // ZS_Y0 = left edge of the logo (up to 5 lines)
  tft.print(msg);
  tft.setFont();
  tft.setRotation(0);
  Z1_SplashLine++;

}

void Splash_Status(const __FlashStringHelper *msg) {

  char buf[60];
  strncpy_P(buf, (const char *)msg, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  Splash_Status(buf);

}

// Count the other settings banks (Z_DCC_2.TXT .. Z_DCC_10.TXT) and the
// session recordings (Z_DCCR<n>.TXT) in the root of the SD card
void Count_SD_Files(byte &otherSettings, byte &recordings) {

  otherSettings = 0;
  recordings = 0;
  File dir = SD.open("/");
  if (!dir) return;

  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    const char *n = entry.name();
    if (!entry.isDirectory() && strstr_P(n, PSTR(".TXT")) != NULL) {
      if (strncasecmp_P(n, PSTR("Z_DCCR"), 6) == 0 && isdigit(n[6])) {
        recordings++;
      }
      else if (strncasecmp_P(n, PSTR("Z_DCC_"), 6) == 0 && isdigit(n[6])) {
        int bank = atoi(n + 6);
        if (bank >= 2 && bank <= 10) otherSettings++;
      }
    }
    entry.close();
  }
  dir.close();

}

void setup()

{

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  pinMode(15, INPUT_PULLUP);                // RX3: holds the line idle when nothing is connected (no WiFi bridge)
  while (!Serial) {
    ;
  }

  Serial.println(F("Welcome to Zero DCC!"));
  Serial.println();
  Serial.println(F("You should only see DCC++EX commands here (or errors if things go wrong)."));
  Serial.println();

  pinMode (digitalPinToInterrupt(Z1_Pin), INPUT);

  pinMode(Z1_Relay_Pin, OUTPUT);
  digitalWrite(Z1_Relay_Pin, HIGH);         // Zero 1 / MicroMimic feed cut until DCC track power is turned on

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GRAY    0x8410
#define Z_DCC   ZP_WHITE                    // text background = label area of the panel
#define Z_DCC_RED ZP_RED                    // status line red = panel red

  pinMode(Z1_SD_Card_Pin, OUTPUT);          // SD card off the shared SPI bus while the display starts
  digitalWrite(Z1_SD_Card_Pin, HIGH);

  uint16_t ID;
  ID = tft.readID();
  if (ID == 0x0D3D3) ID = 0x7796;
  tft.begin(ID);

  drawZeroSplash(tft);
  unsigned long splashStart = millis();

  pinMode(Z1_SD_Card_Pin, OUTPUT);
  digitalWrite(Z1_SD_Card_Pin, HIGH);       // SD card deselected until SD.begin()
  touchBegin();                             // touch chip deselected before the SD card uses the shared SPI bus

 if (!SD.begin(Z1_SD_Card_Pin))

  {
    Serial.println(F("SD Card failed."));
    if (Roster_EE_Read()) {
      Roster_EE_Use();
      char msg[48];
      snprintf_P(msg, sizeof(msg), PSTR("No SD card - roster %d from memory"), Z1_Accessory97Bank);
      Splash_Status(msg);
    }
    else Splash_Status(F("Settings not found - using defaults"));
    Splash_Status(F("No additional settings files found"));
    Splash_Status(F("No MicroMimic file found - function not available"));
    Splash_Status(F("No recordings found"));
  }

  else {

    Z1_SDOK = true;
    // the roster in use when Zero DCC was last switched off (EEPROM), else roster 1
    if (Roster_EE_Read()) Z1_Accessory97Bank = Roster_EE.bank;
    String Filename = String (Z1_Settings_File + String(Z1_Accessory97Bank, DEC) + ".txt");

    if (SD.exists(Filename)) {
      char msg[48];
      snprintf_P(msg, sizeof(msg), PSTR("Loading roster %d"), Z1_Accessory97Bank);
      Splash_Status(msg);
      myfile = SD.open(Filename , FILE_WRITE);
    }
    else if (Roster_EE_Valid) {             // card fitted but that roster's file is missing: EEPROM copy
      Roster_EE_Use();
      char msg[48];
      snprintf_P(msg, sizeof(msg), PSTR("Roster %d file missing - from memory"), Z1_Accessory97Bank);
      Splash_Status(msg);
    }
    else {
      Splash_Status(F("Settings not found - using defaults"));
    }
    if (myfile) {

      myfile.seek(0);
      size_t n;
      char str[20];
      j = 1;

      while (true) {

        n = readField(&myfile, str, sizeof(str), ",\n");

        if (n == 0) {
          myfile.close();
          break;
        }

        if (str[n - 1] == ',' || str[n - 1] == '\n') {

          str[n - 1] = 0;
        }

        else {

          Serial.print(myfile.available() ? F("error: ") : F("eof:   "));
        }

        if (j > 16) break;                  // only 16 cab slots - ignore any extra lines
        Z1_LocoNo[j++] = atoi(str);

      }
      myfile.close();
    }

    byte otherSettings, recordings;
    Count_SD_Files(otherSettings, recordings);
    char msg[48];

    if (otherSettings == 0) {
      Splash_Status(F("No additional settings files found"));
    }
    else {
      snprintf_P(msg, sizeof(msg), PSTR("%d other settings file%s found"), otherSettings, otherSettings == 1 ? "" : "s");
      Splash_Status(msg);
    }

    if (!SD.exists(DCC_MAP_FILE)) {
      Splash_Status(F("No MicroMimic file found - function not available"));
    }
    else if (dccMapLoad() < 0) {            // track map for Accessory 93 (details on the serial monitor)
      Splash_Status(F("MicroMimic file has errors - function not available"));
    }
    else {
      Splash_Status(F("MicroMimic file found"));
    }

    if (recordings == 0) {
      Splash_Status(F("No recordings found"));
    }
    else {
      snprintf_P(msg, sizeof(msg), PSTR("%d session recording%s found"), recordings, recordings == 1 ? "" : "s");
      Splash_Status(msg);
    }

  }

  if (touchStatus == TOUCH_OK)         Splash_Status(F("Touch screen found"));
  else if (touchStatus == TOUCH_FAULT) Splash_Status(F("Touch screen fault - touch controls off"));
  else                                 Splash_Status(F("No touch screen - touch controls off"));

  while (millis() - splashStart < Z1_SplashTime) {
    ;
  }

  tft.setTextColor(BLACK, Z_DCC);
  tft.setTextSize(2);
  drawZeroPanel(tft);
  Update_Screen_Locos();

  Z1_TrackType = 2;
  Update_Screen_Track_Type();
  Track_Power_Off();
  Set_Track_Mode(Z1_TrackType);             // A: Main, B: Program - sent now, and again shortly (Startup_Sync)
  Draw_Screen_Arrows();
  Z1_SyncAt = millis() + 3000;
  Z1_SyncLeft = 2;

  Z1Bus_Begin();                            // Zero 1 accessory bus: waveform running, H-bridge off until track power is on
  Z1Rail_Begin();                           // MicroMimic Rail Drive (light pen)

  attachInterrupt (digitalPinToInterrupt(Z1_Pin), Z1_CALC_PULSE, RISING);

}

void loop()
{

  interrupts();
  Data_Cycle();
  Z1Bus_Poll();                             // light pen messages, Zero 1 bus overload trip
  Read_Bridge();
  Touch_Cycle();
  Menu_Cycle();
  Whistle_Cycle();
  Startup_Sync();

  if (Z1_Accessory95Flag == 3) {
    Playback_Cycle();
  }
  Rec_Watch();                              // recording cabs 9990-9999 / ESP follow Acc 95
  Roster_Watch();                           // last roster -> EEPROM

}

void Z1_CALC_PULSE()
{
  // rising edge on the Zero 1 input: just note the time (low 16 bits of micros() are enough)
  uint16_t h = Z1_EdgeHead;
  Z1_EdgeTime[h & (Z1_EDGES - 1)] = (uint16_t)micros();
  Z1_EdgeHead = h + 1;
}

void Data_Cycle()
{
  uint16_t head;
  noInterrupts();
  head = Z1_EdgeHead;
  interrupts();

  uint16_t waiting = head - Z1_EdgeTail;
  if (waiting > Z1_EDGES - 64) {            // fell too far behind: the oldest times are being overwritten
    Z1_EdgeTail = head;                     // start again from the newest edge
    Z1_EdgePrimed = false;
    Z1_Count = 0;
    Z1_FrameBad = false;
    Z1_OverrunCount++;
    static unsigned long said = 0;
    if (millis() - said >= 5000) {          // say so now and then
      said = millis();
      Serial.print(F("Zero 1 input: fell behind, messages dropped ("));
      Serial.print(Z1_OverrunCount);
      Serial.println(F(" times)"));
    }
    return;
  }

  for (uint8_t n = 0; n < 200 && Z1_EdgeTail != head; n++) {   // everything waiting (a bounded batch)
    uint16_t t = Z1_EdgeTime[Z1_EdgeTail & (Z1_EDGES - 1)];
    Z1_EdgeTail++;
    if (!Z1_EdgePrimed) { Z1_LastEdge = t; Z1_EdgePrimed = true; continue; }
    Z1_PulseDuration = (uint16_t)(t - Z1_LastEdge);
    Z1_LastEdge = t;
    Z1_Period();
  }
}

// One period (rising edge to rising edge) of the Zero 1 input
void Z1_Period()
{
    if (Z1_PulseDuration > 1000) {
      if (Z1_Count == 33) {
        if (!Z1_FrameBad && Z1_FrameValid()) { if (Z1_Confirmed()) decoder(); }
        else Z1_Rejected();
      }
      else if (Z1_Count > 0) Z1_Rejected();   // message cut short
      Z1_Count = 0;
      Z1_FrameBad = false;
      }

    else if (Z1_PulseDuration > 10 && Z1_PulseDuration < 150) {
      if (Z1_Count < 33) {
        Z1_Count = Z1_Count + 1;
        Z1_Data[Z1_Count] = 0;
      }
      }

    else if (Z1_PulseDuration > 160 && Z1_PulseDuration < 250) {
      if (Z1_Count < 33) {
        Z1_Count = Z1_Count + 1;
        Z1_Data[Z1_Count] = 1;
      }
      }

    else Z1_FrameBad = true;                  // neither a 0, a 1 nor the gap: noise - drop this message

}

// RC 3.1: check a received Zero 1 message before using it (MERG CC01/1): the eight 4-bit fields add up
// to 1111 (end-around carry), field 1 starts with the sync bits 0 1, field 8 is 0010, then the extra 0.
// A message garbled by noise (e.g. a point solenoid firing) fails this and is ignored, so it can't turn
// into stray loco or accessory commands.
bool Z1_FrameValid() {
  uint32_t msg = 0;
  for (uint8_t i = 0; i < 32; i++) if (Z1_Data[i + 1]) msg |= (uint32_t)1 << i;
  if (!(Z1_Data[33] == 0 && z1Field(msg, 8) == 0x2 && (z1Field(msg, 1) & 0x3) == 0x2 && z1ChecksumOK(msg))) return false;
  uint8_t f2 = z1Field(msg, 2), f3 = z1Field(msg, 3), f4 = z1Field(msg, 4), f5 = z1Field(msg, 5);
  if (f2 == 0 && f3 == 0)                   // accessory: two decimal digits, only bit 3 of F6 used
    return f4 <= 9 && f5 <= 9 && (z1Field(msg, 6) & 0x7) == 0;
  return f2 != 0 && f3 != 0 && f4 != 0 && f5 != 0;   // loco: speeds 1-15 (0 is never sent)
}

// RC 3.2: a loco message is acted on only when the same message for that group of four locos arrives
// twice in a row. The Master repeats each group every 200 ms, so a change takes effect 0.2 s later,
// and a one-off garbled message that still passes the checks can't change a loco. Accessory messages
// (sent only a few times) are used straight away.
uint32_t Z1_LastLocoMsg[4] = { 0, 0, 0, 0 };
bool Z1_Confirmed() {
  uint32_t msg = 0;
  for (uint8_t i = 0; i < 32; i++) if (Z1_Data[i + 1]) msg |= (uint32_t)1 << i;
  if (z1Field(msg, 2) == 0 && z1Field(msg, 3) == 0) return true;      // accessory
  uint8_t tag = (z1Field(msg, 1) >> 2) & 3;
  bool same = (msg == Z1_LastLocoMsg[tag]);
  Z1_LastLocoMsg[tag] = msg;
  return same;
}

void Z1_Rejected() {
  Z1_RejectCount++;
  static unsigned long last = 0;
  if (millis() - last >= 5000) {            // say so now and then, not for every message
    last = millis();
    Serial.print(F("Zero 1 input: ignored "));
    Serial.print(Z1_RejectCount);
    Serial.println(F(" garbled message(s)"));
    Z1_RejectCount = 0;
  }
}

void decoder() {

  if ((Z1_Data[8] + Z1_Data[7] + Z1_Data[6] + Z1_Data[5]) == 0) {

    AccessoryNo = (((Z1_Data[13] * 1) + (Z1_Data[14] * 2) + (Z1_Data[15] * 4) + (Z1_Data[16] * 8)) * 10) + ((Z1_Data[17] * 1) + (Z1_Data[18] * 2) + (Z1_Data[19] * 4) + (Z1_Data[20] * 8));

    if (Z1_Accessory99Flag == 1 && AccessoryNo != 99 ) {
      Accessory99();
    }

    else if (Z1_Accessory98Flag == 1 && AccessoryNo != 98 ) {
      Accessory98();
    }

    else if (Z1_Accessory92Flag != 0 && AccessoryNo != 92 ) {
      Accessory92();
    }

    else if (Z1_Accessory97Flag == 1 && AccessoryNo != 97 && Z1_Accessory95Flag != 1) {   // also during recording / playback
      Accessory97();
    }

    else if (Z1_Accessory96Flag == 1 && AccessoryNo != 96 && Z1_Accessory95Flag != 1)  {   // also during recording / playback
      Accessory96();
    }

    // Accessory 95 - record / play back. "95<-" always leaves the function.
    else if (AccessoryNo == 95 && Z1_Accessory95Flag == 3) {    // playing back: 95 either way stops it
      Playback_Stop();
    }

    else if (AccessoryNo == 95 && Z1_Accessory95Flag == 2) {    // recording: 95 either way ends and saves it
      Recording_Stop();
    }

    else if (Z1_Accessory95Flag == 1 ) {                         // waiting for a recording number
      if (AccessoryNo == 95) {
        if (Z1_Data[24] == 1) {                                  // 95<- : cancel
          Accessory95_Cancel();
        }                                                        // 95-> : ignored, still waiting
      }
      else {
        Accessory95();                                           // n-> record, n<- play back
      }
    }

    else if (Z1_Accessory91Flag == 1 && AccessoryNo != 91 && Z1_Accessory95Flag == 0 ) {
      Accessory91();
    }

    else if (Z1_Accessory90Flag == 1 && AccessoryNo != 90 && Z1_Accessory95Flag == 0 ) {
      Accessory90();
    }

    else if (AccessoryNo == 99 || AccessoryNo == 98 || AccessoryNo == 97) {
      if (Z1_Data[24] == 1) Acc_End(AccessoryNo);         // 9x<- : end the function
      else                  Acc_Start(AccessoryNo);       // 9x-> : start it
    }

    else if (AccessoryNo == 95 && Z1_Accessory95Flag == 0) {

      if (Z1_Data[24] == 0) {                                    // 95-> : wait for a recording number
        Z1_Accessory95Flag = 1;
        digitalWrite(Z1_Relay_Pin, HIGH);
        Serial.println(F("Acc 95: choose a recording number (forward = record, back = play) or 95 back to cancel"));
      }                                                          // 95<- : already off, ignored

    }

    else if (AccessoryNo == 96 || AccessoryNo == 92 || AccessoryNo == 91 || AccessoryNo == 90) {
      if (Z1_Data[24] == 1) Acc_End(AccessoryNo);
      else                  Acc_Start(AccessoryNo);
    }

    else if (AccessoryNo == MAP_SCREEN_ACC) {          // Accessory 93: 93-> map screen, 93<- loco screen
      if (Z1_Data[24] == 0) {
        if (dccMapLoaded) Show_Screen(SCREEN_MAP);
        else Serial.println(F("Acc 93 ignored: no valid map file loaded"));
      }
      else if (Z1_Screen != SCREEN_LOCO) {
        Show_Screen(SCREEN_LOCO);
      }
    }

    else if (AccessoryNo < 90 && Z1_Accessory91Flag == 0 && Z1_Accessory96Flag == 0 && Z1_Accessory97Flag == 0 && Z1_Accessory98Flag == 0 && Z1_Accessory99Flag == 0 ) {
      Point_Routine();
    }

  }

  else {
    Locomotive_Routine();
  }

  Z1_Count = 0;

}

// Start / end one of the Accessory 90-99 functions - the same for a Zero 1 key (9x-> / 9x<-)
// and the touch menu. Starting cuts the Zero 1 / MicroMimic feed; ending restores it (if track power is on).
void Acc_Start(byte n) {

  switch (n) {
    case 90: Z1_Accessory90Flag = 1; LocoNumber = 0; break;
    case 91: Z1_Accessory91Flag = 1; LocoNumber = 0; break;
    case 92:
      Z1_Accessory92Flag = 1;
      Z1_Accessory92Number = 0;
      Serial.println(F("Acc 92: enter the accessory number (digits with <-, then any key -> to accept)"));
      break;
    case 96: Z1_Accessory96Flag = 1; LocoNumber = 0; break;
    case 97: Z1_Accessory97Flag = 1; LocoNumber = 0; break;
    case 98: Z1_Accessory98Flag = 1; LocoNumber = 1; break;
    case 99: Z1_Accessory99Flag = 1; LocoNumber = 0; break;
    default: return;
  }
  digitalWrite(Z1_Relay_Pin, HIGH);

}

void Acc_End(byte n) {

  switch (n) {
    case 90:
      Z1_Accessory90Flag = 0;
      LocoNumber = 0;
      if (Z1_PowerPending) {                // ON was chosen: DCC power and Zero 1 feed on together
        Z1_PowerPending = false;
        Track_Power_On();
      }
      break;
    case 91: Z1_Accessory91Flag = 0; LocoNumber = 0; break;
    case 92:
      if (Z1_Accessory92Flag != 0) Serial.println(F("Acc 92 ended."));
      Z1_Accessory92Flag = 0;
      Z1_Accessory92Number = 0;
      break;
    case 96: Z1_Accessory96Flag = 0; LocoNumber = 0; break;
    case 97: Z1_Accessory97Flag = 0; LocoNumber = 0; break;
    case 98: Z1_Accessory98Flag = 0; LocoNumber = 0; break;
    case 99: Z1_Accessory99Flag = 0; LocoNumber = 0; Z1_Accessory99Function = 0; break;
    default: return;
  }
  Relay_Restore();

}

void Locomotive_Routine() {

  // Track power off: send nothing to DCC-EX and leave the stored speeds alone, so the
  // first Zero 1 update after power comes back on sends every loco's current setting
  if (!Z1_TrackPowerOn) return;

  Z1_Tag = (Z1_Data[3] * 1) + (Z1_Data[4] * 2);

  int LocoSpeed[5];
  int LocoDirection[5];
  j = 1;
  for (j = 1 ; j <= 4 ; j++) {
    int Change = 0;

    LocoSpeed[j] = ((Z1_Data[1 + (j * 4)] * 1) + (Z1_Data[2 + (j * 4)] * 2) + (Z1_Data[3 + (j * 4)] * 4) + (Z1_Data[4 + (j * 4)] * 8) - 1);

    LocoSpeed[j] = map(LocoSpeed[j], 0, 14, 0, 126);

    byte k = j + (Z1_Tag * 4);
    if (Z1_MasterSpeed[k] != LocoSpeed[j])  {   // a change on the Master (not just its repeat)
      Change = 1;
      Z1_MasterSpeed[k] = LocoSpeed[j];
      Z1_LocoSpeed[k] = LocoSpeed[j];
    }

    LocoDirection[j] = Z1_Data [20 + j];

    if (Z1_MasterDirection[k] != LocoDirection[j]) {
      Change = 1;
      Z1_MasterDirection[k] = LocoDirection[j];
      Z1_LocoDirection[k] = LocoDirection[j];
    }

    if (Change == 1) {
      Z1_LocoSpeed[k] = LocoSpeed[j];           // the Master's whole setting takes over
      Z1_LocoDirection[k] = LocoDirection[j];
      Slot_Send(k);
      Change = 0;
    }
  }

}

// Zero 1 locos 1-16 are DCC cab IDs 1-16: a command for address 1-16 drives that Zero 1 loco on the bus
void Z1Bus_Cab(int address, int speed, int direction) {
  if (address >= 1 && address <= 16) Z1Bus_Loco(address, speed, direction);
}

// Send slot k's speed and direction everywhere: DCC-EX, the Zero 1 bus, a recording, the screen,
// and (Zero 1 only, no DCC-EX) the phones. Used for Master keys and phone commands alike.
void Slot_Send(byte k) {

  DCC_Out.print("<t ");
  DCC_Out.print(k);
  DCC_Out.print(" ");
  DCC_Out.print(Z1_LocoNo[k]);
  DCC_Out.print(" ");
  DCC_Out.print(Z1_LocoSpeed[k]);
  DCC_Out.print(" ");
  DCC_Out.print(Z1_LocoDirection[k]);
  DCC_Out.println(">");
  Z1Bus_Cab(Z1_LocoNo[k], Z1_LocoSpeed[k], Z1_LocoDirection[k]);   // Zero 1 loco = the slot's address (1-16)

  if ( Z1_Accessory95Flag == 2) {
    rightnow = millis();
    myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

    Serial.print("Writing to file:");
    if (myfile) {
      myfile.print(rightnow - Z1_LastCommandTime);
      myfile.print ("\n");
      myfile.print("<t ");
      myfile.print(k);
      myfile.print(" ");
      myfile.print(Z1_LocoNo[k]);
      myfile.print(" ");
      myfile.print(Z1_LocoSpeed[k]);
      myfile.print(" ");
      myfile.print(Z1_LocoDirection[k]);
      myfile.print(">");
      myfile.print( "\n" );
      myfile.close();
    }
    else {

      Serial.println("error opening file");
    }
    Z1_LastCommandTime = rightnow;
  }
  Update_Screen_Speeds(int (k));

  Phone_Loco(k);

}

// Tell the phones (WiFi 2.0) about the loco in slot k: <l cab 0 speedbyte functions>, as DCC-EX would
void Phone_Loco(byte k) {

  if (!Z1_WiFi2 || k < 1 || k > 16 || Z1_LocoNo[k] <= 0) return;
  byte sb = Z1_LocoSpeed[k] == 0 ? 0 : Z1_LocoSpeed[k] + 1;    // DCC-EX speed byte
  if (Z1_LocoDirection[k]) sb |= 0x80;
  uint32_t *fn = Fn_Find(Z1_LocoNo[k], false);
  Serial3.print(F("@<l "));
  Serial3.print(Z1_LocoNo[k]);
  Serial3.print(F(" 0 "));
  Serial3.print(sb);
  Serial3.print(' ');
  Serial3.print(fn ? *fn : 0UL);
  Serial3.println('>');

}

// Zero 1 point key (accessories 1-89): <- = 1, -> = 0
void Point_Routine() {

  Send_Accessory(AccessoryNo, Z1_Data[24] == 1 ? 1 : 0);

}

// Every point / accessory command goes through here - from the Zero 1, Accessory 92 or the touch screen.
// Sends <a acc state> to DCC-EX, writes it to the recording during Acc 95 recording, and updates the map.
// Track power off: nothing is sent and the map is left as it is.
void Send_Accessory(unsigned int acc, byte state) {
  Accessory_Do(acc, state, true);
}

// sendDcc false: a phone's turnout / accessory that the ESP has already sent to DCC-EX itself -
// only the Zero 1 side here (Zero 1 bus, map, recording, the phones' turnout buttons)
void Accessory_Do(unsigned int acc, byte state, bool sendDcc) {

  if (!Z1_TrackPowerOn) return;             // track power off: no accessory commands, map unchanged

  if (sendDcc) {
    DCC_Out.print("<a ");
    DCC_Out.print(acc);
    DCC_Out.print(" ");
    DCC_Out.print(state);
    DCC_Out.println(">");
  }

  Z1Bus_Accessory(acc, state);              // and to the Zero 1 accessory modules / MicroMimic (1-89 only)
  if (Z1_WiFi2 && acc >= 1 && acc <= 2040) {   // the phones: their turnouts on this address (as <H id state>)
    Serial3.print(F("@A "));
    Serial3.print(acc);
    Serial3.print(' ');
    Serial3.println(state);
  }

  if (Z1_Accessory95Flag == 2) {
    rightnow = millis();
    myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

    Serial.print("Writing to file:");
    if (myfile) {
      myfile.print(rightnow - Z1_LastCommandTime);
      myfile.print("\n");
      myfile.print("<a ");
      myfile.print(acc);
      myfile.print(" ");
      myfile.print(state);
      myfile.print(">");
      myfile.print("\n");
      myfile.close();
    }
    else {
      Serial.println("error opening file");
    }
    Z1_LastCommandTime = rightnow;
  }

  dccMapSetPoint(tft, acc, state == 1);     // update the point's / signal's dots (93 is just a point here)
  if (Z1_Screen == SCREEN_MAP) Draw_Screen_Arrows(); // a dot in an edge square may have drawn over an arrow

}

// MicroMimic light pen hit, read from its Rail Drive socket (ZeroZ1Bus.h): handled exactly like the
// same point key on the Zero 1 - DCC-EX, the map, an Acc 95 recording and the Zero 1 accessory bus.
// Ignored while an Accessory 90-99 function is waiting for keys, as the Zero 1 point keys are.
void Z1Rail_OnAccessory(uint8_t acc, uint8_t state) {

  if (acc < 1 || acc > 89) return;
  if (Z1_Accessory90Flag || Z1_Accessory91Flag || Z1_Accessory92Flag || Z1_Accessory96Flag ||
      Z1_Accessory97Flag || Z1_Accessory98Flag || Z1_Accessory99Flag || Z1_Accessory95Flag == 1) {
    Serial.println(F("MicroMimic light pen ignored: an Accessory 90-99 function is running"));
    return;
  }
  Serial.print(F("MicroMimic light pen: accessory "));
  Serial.print(acc);
  Serial.println(state ? F(" <-") : F(" ->"));
  Send_Accessory(acc, state);

}

// ================= Touch screen =================
// All positions are landscape pixels (480 x 320, the way the screen is viewed, status strip at the top).
//
//   Both screens:  TRACK POWER ON / OFF in the status strip - tap to turn track power on or off
//                  arrows at the left and right edges, half way down - switch loco screen <-> map
//   Map:           point - tap nearer the dot of the route you want; signal - right/below = red,
//                  left/above = green. Every tap is sent and recorded, even if nothing changes.
//   Loco screen:   CAB ID - headlight F0 on/off (as Acc 98 <-), yellow background while it's on
//                  slot number (red strip) - whistle F2 on/off (as Acc 98 ->)
//                  bottom chevron - change direction
//                  chevrons 2-15 - set speed (tap the top lit chevron again to stop)

const int16_t TOUCH_POWER_X = 280;          // power button: status strip, right-hand part
const int16_t TOUCH_STRIP_H = 28;           // status strip height
const int16_t ARROW_Y       = 160;          // arrows: centre height
const int16_t ARROW_HALF    = 14;           // arrows: half height of the triangle
const int16_t ARROW_DEPTH   = 11;           // arrows: how far the triangle comes in from the edge
const int16_t ARROW_TOUCH_W = 20;           // arrows: touch zone width from the edge
const int16_t ARROW_TOUCH_H = 26;           // arrows: touch zone half height

// The screen after (forward) or before the current one; the map is skipped if there's no map file
byte Next_Screen(byte s, bool forward) {
  do {
    s = forward ? (s + 1) % 3 : (s + 2) % 3;
  } while (s == SCREEN_MAP && !dccMapLoaded);
  return s;
}

void Show_Screen(byte s) {

  if (s == SCREEN_MAP && !dccMapLoaded) return;
  Z1_Screen = s;
  dccMapShown = (s == SCREEN_MAP);           // DccMap.h only draws dots while the map is showing
  if (s == SCREEN_MAP) {
    dccMapDraw(tft);                         // ends with showMapStatus(): status strip + arrows
  }
  else if (s == SCREEN_MENU) {
    Menu_Show_Screen();
  }
  else {
    tft.setRotation(0);
    showLocoScreen();
  }

}

void Touch_Cycle() {

  int16_t x, y;
  if (!touchTap(x, y)) return;

  // power button (both screens)
  if (y < TOUCH_STRIP_H) {
    if (x >= TOUCH_POWER_X) Touch_Power();
    return;                                 // rest of the status strip: nothing
  }

  // screen arrows (every screen): right = next screen, left = previous
  if (y >= ARROW_Y - ARROW_TOUCH_H && y <= ARROW_Y + ARROW_TOUCH_H) {
    if (x >= TOUCH_W - ARROW_TOUCH_W) { Show_Screen(Next_Screen(Z1_Screen, true));  return; }
    if (x < ARROW_TOUCH_W)            { Show_Screen(Next_Screen(Z1_Screen, false)); return; }
  }

  if (Z1_Screen == SCREEN_MAP)       Touch_Map(x, y);
  else if (Z1_Screen == SCREEN_MENU) Menu_Touch(x, y);
  else                               Touch_Loco(x, y);

}

// true while one of the Accessory 90-99 functions is running (Zero 1 feed cut)
bool Any_Function_Active() {
  return Z1_Accessory90Flag || Z1_Accessory91Flag || Z1_Accessory92Flag || Z1_Accessory95Flag == 1 ||
         Z1_Accessory96Flag || Z1_Accessory97Flag || Z1_Accessory98Flag || Z1_Accessory99Flag;
}

// power button: same as Acc 90 ON / OFF, not recorded
void Touch_Power() {

  Z1_PowerPending = false;
  if (Z1_TrackPowerOn) {
    Serial.println(F("Touch: track power off"));
    Track_Power_Off();
  }
  else {
    Serial.println(F("Touch: track power on"));
    Track_Power_On();
    if (!Any_Function_Active()) Relay_Restore();   // Zero 1 feed back on, unless a 90-99 function is open
  }

}

void Touch_Map(int16_t x, int16_t y) {

  uint16_t acc;
  uint8_t state;
  if (!dccMapTap(x, y, acc, state)) return;  // plain track, blank square

  if (!Z1_TrackPowerOn) {
    Serial.println(F("Touch ignored: track power is off"));
    return;
  }
  Send_Accessory(acc, state);                // sent and recorded even if it's already set this way

}

// Loco screen: the panel is drawn in portrait (ZeroPanel.h), so turn the landscape tap into
// portrait coordinates, then find the sloped row and the part of the row that was tapped
void Touch_Loco(int16_t x, int16_t y) {

  int16_t px = ZP_W - 1 - y;                 // across the row: CAB ID (0) ... red strip (319)
  int16_t py = x;                            // down the panel: row 1 ... row 16

  byte k = 0;
  for (byte r = 1; r <= 16; r++) {
    if (py >= zpLineYf(r - 1, px) && py < zpLineYf(r, px)) { k = r; break; }
  }
  if (k == 0) return;                        // above row 1, row 17 or below

  if (px < 62) {                             // CAB ID: headlight
    Loco_Function(k, 0);
  }
  else if (px >= ZP_RED_X) {                 // slot number in the red strip: whistle
    Whistle(k);
  }
  else if (px >= 64 && px < 220) {           // chevrons
    int l = constrain((px - 58) / 10, 1, 15);
    if (!Z1_TrackPowerOn) {
      Serial.println(F("Touch ignored: track power is off"));
      return;
    }
    if (l == 1) {                            // bottom chevron: change direction, same speed
      Send_Loco(k, Z1_ShownSpeed[k], Z1_ShownDirection[k] ? 0 : 1);
    }
    else {
      byte speed = (l - 1) * 9;              // chevron 2 = 9 ... chevron 15 = 126
      if (speed > 126) speed = 126;
      if ((Z1_ShownSpeed[k] / 9) + 1 == l) speed = 0;   // top lit chevron tapped again: stop
      Send_Loco(k, speed, Z1_ShownDirection[k]);
    }
  }

}

// Speed / direction for cab slot k from the touch screen. Sent and recorded like a Zero 1 change.
// The Zero 1's own setting for the slot isn't touched, so the next change on the Zero 1 for this
// slot takes over again.
void Send_Loco(byte k, byte speed, byte direction) {

  if (!Z1_TrackPowerOn) return;

  DCC_Out.print("<t ");
  DCC_Out.print(k);
  DCC_Out.print(" ");
  DCC_Out.print(Z1_LocoNo[k]);
  DCC_Out.print(" ");
  DCC_Out.print(speed);
  DCC_Out.print(" ");
  DCC_Out.print(direction);
  DCC_Out.println(">");
  Z1Bus_Cab(Z1_LocoNo[k], speed, direction);   // Zero 1 loco = the slot's address (1-16)

  if (Z1_Accessory95Flag == 2) {
    rightnow = millis();
    myfile = SD.open(Z1_Accessory95File , FILE_WRITE);
    Serial.print("Writing to file:");
    if (myfile) {
      myfile.print(rightnow - Z1_LastCommandTime);
      myfile.print("\n");
      myfile.print("<t ");
      myfile.print(k);
      myfile.print(" ");
      myfile.print(Z1_LocoNo[k]);
      myfile.print(" ");
      myfile.print(speed);
      myfile.print(" ");
      myfile.print(direction);
      myfile.print(">");
      myfile.print("\n");
      myfile.close();
    }
    else {
      Serial.println("error opening file");
    }
    Z1_LastCommandTime = rightnow;
  }

  Z1_ShownSpeed[k] = speed;
  Z1_ShownDirection[k] = direction;
  Draw_Speed(k);

}

// ---- loco function state, by address ----
bool Loco_In_Slot(int addr) {
  for (byte k = 1; k <= 16; k++) if (Z1_LocoNo[k] == addr) return true;
  return false;
}

// the function bits for this loco address (NULL if not known and create is false)
uint32_t *Fn_Find(int addr, bool create) {
  if (addr <= 0) return NULL;
  for (byte i = 0; i < FN_LOCOS; i++) if (Z1_Fn[i].addr == addr) return &Z1_Fn[i].fn;
  if (!create) return NULL;
  // a free entry, or the oldest one whose loco isn't in a cab slot any more
  byte use = Z1_FnNext;
  for (byte t = 0; t < FN_LOCOS; t++) {
    byte i = (Z1_FnNext + t) % FN_LOCOS;
    if (Z1_Fn[i].addr == 0 || !Loco_In_Slot(Z1_Fn[i].addr)) { use = i; break; }
  }
  Z1_FnNext = (use + 1) % FN_LOCOS;
  Z1_Fn[use].addr = addr;
  Z1_Fn[use].fn = 0;
  return &Z1_Fn[use].fn;
}

bool Fn_Get(int addr, byte fn) {
  uint32_t *p = Fn_Find(addr, false);
  return p && ((*p >> fn) & 1);
}

void Fn_Set(int addr, byte fn, bool on) {
  uint32_t *p = Fn_Find(addr, true);
  if (!p) return;
  if (on) *p |= (1UL << fn); else *p &= ~(1UL << fn);
}

// function fn of the loco in cab slot k
bool Slot_Fn(byte k, byte fn) { return Fn_Get(Z1_LocoNo[k], fn); }

// show a function change on every row that has this loco (yellow CAB ID for the light, menu icons)
void Loco_Fn_Shown(int addr, byte fn) {
  for (byte k = 1; k <= 16; k++) {
    if (Z1_LocoNo[k] != addr) continue;
    if (fn == 0) Draw_Loco_ID(k);
    Menu_Loco_Changed(k, fn);
  }
}

// Set function fn of loco 'addr' on or off: sent to DCC-EX (even with track power off, as Acc 98 / 99),
// recorded during an Acc 95 recording, and shown on screen
void Loco_Fn_Send(int addr, byte fn, bool on) {

  if (addr <= 0 || fn > 31) return;
  Fn_Set(addr, fn, on);

  DCC_Out.print("<F ");
  DCC_Out.print(addr);
  DCC_Out.print(" ");
  DCC_Out.print(fn);
  DCC_Out.print(" ");
  DCC_Out.print(on ? 1 : 0);
  DCC_Out.println(">");

  if (Z1_Accessory95Flag == 2) {
    rightnow = millis();
    myfile = SD.open(Z1_Accessory95File , FILE_WRITE);
    Serial.print("Writing to file:");
    if (myfile) {
      myfile.print(rightnow - Z1_LastCommandTime);
      myfile.print("\n");
      myfile.print("<F ");
      myfile.print(addr);
      myfile.print(" ");
      myfile.print(fn);
      myfile.print(" ");
      myfile.print(on ? 1 : 0);
      myfile.print(">");
      myfile.print("\n");
      myfile.close();
    }
    else {
      Serial.println("error opening file");
    }
    Z1_LastCommandTime = rightnow;
  }

  Loco_Fn_Shown(addr, fn);
  for (byte k = 1; k <= 16; k++) if (Z1_LocoNo[k] == addr) { Phone_Loco(k); break; }   // the phones too

}

// Toggle function fn of the loco in cab slot k (1-16) - Acc 98 <-, Acc 99, the touch screen and the menu
void Loco_Function(byte k, byte fn) {

  if (k < 1 || k > 16 || fn > 31) return;
  int addr = Z1_LocoNo[k];
  Loco_Fn_Send(addr, fn, !Fn_Get(addr, fn));

}

// Whistle (F2) for the loco in cab slot k - Acc 98 ->, the slot number on the loco screen, the menu
void Whistle(byte k) {

  if (k < 1 || k > 16) return;
  if (!WHISTLE_MOMENTARY) { Loco_Function(k, 2); return; }
  int addr = Z1_LocoNo[k];
  if (addr <= 0) return;
  byte freeSlot = 255;
  for (byte i = 0; i < 4; i++) {
    if (Z1_Whistle[i].addr == addr) {          // already sounding: just keep it going a bit longer
      Z1_Whistle[i].at = millis() + WHISTLE_MS;
      return;
    }
    if (Z1_Whistle[i].addr == 0 && freeSlot == 255) freeSlot = i;
  }
  if (freeSlot == 255) {                         // four whistles at once: finish the first one now
    Loco_Fn_Send(Z1_Whistle[0].addr, 2, false);
    freeSlot = 0;
  }
  Loco_Fn_Send(addr, 2, true);
  Z1_Whistle[freeSlot].addr = addr;
  Z1_Whistle[freeSlot].at = millis() + WHISTLE_MS;

}

// called from loop(): switch each whistle off when its time is up
void Whistle_Cycle() {
  for (byte i = 0; i < 4; i++) {
    if (Z1_Whistle[i].addr && (long)(millis() - Z1_Whistle[i].at) >= 0) {
      int addr = Z1_Whistle[i].addr;
      Z1_Whistle[i].addr = 0;
      Loco_Fn_Send(addr, 2, false);
    }
  }
}

// Screen-switch arrows at the left and right edges, half way down (landscape)
void Draw_Screen_Arrows() {

  if (!touchFound()) return;                 // no touch screen: no arrows (the menu can't be reached)

  tft.setRotation(1);
  int16_t l = 2, r = TOUCH_W - 3;
  tft.fillTriangle(l, ARROW_Y, l + ARROW_DEPTH, ARROW_Y - ARROW_HALF, l + ARROW_DEPTH, ARROW_Y + ARROW_HALF, Z_DCC_RED);
  tft.drawTriangle(l, ARROW_Y, l + ARROW_DEPTH, ARROW_Y - ARROW_HALF, l + ARROW_DEPTH, ARROW_Y + ARROW_HALF, BLACK);
  tft.fillTriangle(r, ARROW_Y, r - ARROW_DEPTH, ARROW_Y - ARROW_HALF, r - ARROW_DEPTH, ARROW_Y + ARROW_HALF, Z_DCC_RED);
  tft.drawTriangle(r, ARROW_Y, r - ARROW_DEPTH, ARROW_Y - ARROW_HALF, r - ARROW_DEPTH, ARROW_Y + ARROW_HALF, BLACK);
  tft.setRotation(0);

}

void Accessory90() {

  Serial.print(F("Acc 90 key "));
  Serial.println(AccessoryNo);

  if (AccessoryNo == 1) {                   // ON: chosen now, sent when Acc 90 is closed with 90<-
     Z1_PowerPending = true;
     Serial.println(F("Track power ON chosen - applied when Acc 90 is closed"));
     Draw_Track_Power();
     return;
  }

  Z1_PowerPending = false;                  // any other choice turns power off
  Set_Track_Mode(AccessoryNo);

}

// DCC-EX may still be starting when ZeroDCC sends the start-up power off / track mode, or its own
// configuration may switch the tracks on or join them at start-up. So the power off and A: Main /
// B: Program are sent again 3 s and 8 s after start-up, to make DCC-EX match the status strip.
// Skipped if track power has already been turned on, or the track mode changed, in the meantime.
void Startup_Sync() {

  if (!Z1_SyncLeft || (long)(millis() - Z1_SyncAt) < 0) return;
  Z1_SyncLeft--;
  Z1_SyncAt = millis() + 5000;
  if (Z1_TrackPowerOn || Z1_PowerPending || Z1_ProgPower || Z1_TrackType != 2) { Z1_SyncLeft = 0; return; }
  Serial.println(F("Start-up: setting DCC-EX track power off, A: Main, B: Program"));
  Set_Track_Mode(2);

}

// Track power off, then (for 2-4) set the track mode:
//   2 = A: DCC Main, B: DCC Program   3 = A: DCC Main, B: DC 999   4 = A: DC 999, B: DC 998
void Set_Track_Mode(byte mode) {

  Track_Power_Off();
  if (mode == 2) {
    DCC_Out.println("<= A MAIN>");
    DCC_Out.println("<= B PROG>");
  }
  else if (mode == 3) {
    DCC_Out.println("<= A MAIN>");
    DCC_Out.println("<= B DC 999>");
  }
  else if (mode == 4) {
    DCC_Out.println("<= A DC 999>");
    DCC_Out.println("<= B DC 998>");
  }
  else return;
  Z1_TrackType = mode;
  Update_Screen_Track_Type();

}

void Accessory91() {

  if (Z1_Data[24] == 1) {
    if ( AccessoryNo == 10 ) {
      AccessoryNo = 0;
    }
    if ( LocoNumber == 0 ) {
      LocoNumber = AccessoryNo;
    }
    else {
      LocoNumber = (LocoNumber * 10) + AccessoryNo;
    }
  }

  else if (Z1_Data[24] == 0) {

    if ( LocoNumber > 0 && LocoNumber <= 9999) {

        DCC_Out.print("<W ");
        DCC_Out.print(LocoNumber);
        DCC_Out.println(">");
        delay(500);
      }
      Z1_Accessory91Flag = 0;
      LocoNumber = 0;
      Relay_Restore();
    }

    else {

      Z1_Accessory91Flag = 0;
      LocoNumber = 0;
      Relay_Restore();
    }

}

// Accessory 92 - points / accessories numbered over 89
//   number entry: n<- adds a digit (10 = 0), any key -> accepts the number
//   command mode: any key <- sends <a n 1>, any key -> sends <a n 0>, until 92<-
void Accessory92() {

  if (Z1_Accessory92Flag == 1) {

    if (Z1_Data[24] == 1) {                                     // <- : add a digit
      if (AccessoryNo < 1 || AccessoryNo > 10) return;          // only keys 1-10 are digits
      byte digit = (AccessoryNo == 10) ? 0 : AccessoryNo;
      unsigned long next = (unsigned long)Z1_Accessory92Number * 10 + digit;
      if (next > 2040) {                                       // DCC accessory addresses stop at 2040 (2041-2044 = broadcast)
        Serial.println(F("Acc 92: number too large, digit ignored"));
        return;
      }
      Z1_Accessory92Number = (unsigned int)next;
      Serial.print(F("Acc 92 number: "));
      Serial.println(Z1_Accessory92Number);
    }

    else {                                                      // -> : accept the number
      if (Z1_Accessory92Number == 0) {
        Serial.println(F("Acc 92: no number entered yet"));
        return;
      }
      Z1_Accessory92Flag = 2;
      Serial.print(F("Acc 92: controlling accessory "));
      Serial.println(Z1_Accessory92Number);
    }
    return;
  }

  // command mode: the key pressed only gives the direction
  Send_Accessory(Z1_Accessory92Number, (Z1_Data[24] == 1) ? 1 : 0);

}

// Recording number chosen: n-> records to Z_DCCR<n>.txt, n<- plays it back
void Accessory95() {

  if (Z1_Data[24] == 0) Start_Recording(AccessoryNo);
  else                  Start_Playback(AccessoryNo);

}

// Start recording to Z_DCCR<n>.txt (an existing recording n is replaced)
void Start_Recording(int n) {

  Relay_Restore();
  Z1_Accessory95File = String (Z1_Recording_File + String(n, DEC) + ".txt");
  Z1_LastCommandTime = millis();

  if (SD.exists(Z1_Accessory95File)) {
    SD.remove(Z1_Accessory95File);
  }

  myfile = SD.open(Z1_Accessory95File , FILE_WRITE);

  if (myfile) {
    myfile.close();
    Z1_Accessory95Flag = 2;
    Z1_RecordNumber = n;
    Serial.print(F("Recording to "));
    Serial.println(Z1_Accessory95File);
  }
  else {
    Z1_Accessory95Flag = 0;
    Serial.println(F("Recording failed: error opening file"));
  }

}

// Play back Z_DCCR<n>.txt
void Start_Playback(int n) {

  Relay_Restore();
  Z1_Accessory95File = String (Z1_Recording_File + String(n, DEC) + ".txt");
  Z1_Accessory95Flag = 0;
  if (SD.exists(Z1_Accessory95File)) {
    Z1_PlaybackFile = SD.open(Z1_Accessory95File, FILE_READ);
    if (Z1_PlaybackFile) {
      filesize = Z1_PlaybackFile.size();
      fileposition = 0;
      Z1_PlaybackCommandLoaded = false;
      memset(Z1_PlaybackLoco, 0, sizeof(Z1_PlaybackLoco));
      Z1_Accessory95Flag = 3;
      Z1_PlaybackNumber = n;
      Serial.print(F("Playing "));
      Serial.println(Z1_Accessory95File);
    }
  }
  if (Z1_Accessory95Flag != 3) {
    Serial.print(F("Recording not found: "));
    Serial.println(Z1_Accessory95File);
  }

}

void Recording_Stop() {

  Z1_Accessory95Flag = 0;
  Serial.println(F("Recording stopped."));

}

void Accessory95_Cancel() {

  Z1_Accessory95Flag = 0;
  Relay_Restore();
  Serial.println(F("Acc 95 cancelled."));

}

void Playback_Cycle() {
  if (!Z1_PlaybackCommandLoaded) {
    if (fileposition >= filesize) {
      Playback_Stop();
      return;
    }
    readnextcommand();
    Z1_PlaybackCommandLoaded = true;
  }

  if (millis() >= commandtime) {
    if (Z1_TrackPowerOn) {                  // track power off: the recording keeps time but sends nothing
      DCC_Out.println(nextcommand);
      Playback_Show_Command(nextcommand);
      Playback_Z1Bus(nextcommand);
    }
    Z1_PlaybackCommandLoaded = false;
  }
}

// Keep the screens in step with a recording during playback:
//   "<t slot address speed direction>"  -> speed arrows on the row(s) where that loco ADDRESS is
//                                          assigned now (Acc 96/97 may have moved it since the
//                                          recording was made); not shown if it isn't assigned
//   "<a accessory 0/1>"                 -> the point / signal dots on the map
//                                          (<T ...> from recordings made before RC2.2 too)
//   "<F address function 0/1>"          -> that loco's function state; headlight (F0) shown as the
//                                          yellow CAB ID on the row(s) where that address is assigned
// The recorded speeds are only shown, not stored as the Zero 1 control settings, so
// the next change on the Zero 1 controller still takes over as normal.
void Playback_Show_Command(const String &cmd) {
  if (cmd.startsWith("<t ")) {
    int slot, address, speed, direction;
    if (sscanf(cmd.c_str(), "<t %d %d %d %d", &slot, &address, &speed, &direction) == 4 && slot >= 1 && slot <= 16) {
      Z1_PlaybackLoco[slot] = (speed > 0) ? address : 0;   // remembered so Acc 97 doesn't stop it
      for (byte k = 1; k <= 16; k++) {
        if (Z1_LocoNo[k] == address) {      // the row this loco is on now, not the recorded slot
          Z1_ShownSpeed[k] = speed;
          Z1_ShownDirection[k] = direction;
          Draw_Speed(k);
        }
      }
    }
    return;
  }
  if (cmd.startsWith("<F ")) {              // loco function: keep each slot that has this loco in step
    int address, fn, state;
    if (sscanf(cmd.c_str(), "<F %d %d %d", &address, &fn, &state) == 3 && fn >= 0 && fn <= 31) {
      Fn_Set(address, fn, state != 0);      // stored, so the next Acc 98 / 99 / touch toggles from here
      Loco_Fn_Shown(address, fn);           // yellow CAB ID / menu icons on the row(s) with this loco
    }
    return;
  }
  if (!cmd.startsWith("<a ") && !cmd.startsWith("<T ")) return;
  int sp = cmd.indexOf(' ', 3);
  if (sp < 0) return;
  long acc = cmd.substring(3, sp).toInt();
  if (acc < MAP_FIRST_ACC || acc > MAP_LAST_ACC) return;
  if (!Z1_TrackPowerOn) return;             // map only changes while track power is on
  dccMapSetPoint(tft, (uint16_t)acc, cmd.charAt(sp + 1) == '1');   // a recorded <a 93 x> is a point, not the map screen
}

// A recorded command also goes to the Zero 1 bus:
//   "<t slot address speed direction>"  -> Zero 1 loco 'slot' (the recorded cab slot, 1-16)
//   "<a accessory 0/1>" (or "<T ...>" from recordings made before RC2.2) -> accessory 1-89
void Playback_Z1Bus(const String &cmd) {
  if (cmd.startsWith("<t ")) {
    int slot, address, speed, direction;
    if (sscanf(cmd.c_str(), "<t %d %d %d %d", &slot, &address, &speed, &direction) == 4) {
      Z1Bus_Cab(address, speed, direction);   // the recorded loco ADDRESS (1-16 = Zero 1 loco)
    }
    return;
  }
  if (!cmd.startsWith("<a ") && !cmd.startsWith("<T ")) return;
  int acc, state;
  if (sscanf(cmd.c_str() + 3, "%d %d", &acc, &state) == 2 && acc >= 1) {
    Z1Bus_Accessory((unsigned int)acc, state ? 1 : 0);
  }
}

void readnextcommand() {
  Z1_PlaybackFile.seek(fileposition);

  size_t n;
  char str[20];

  n = readField(&Z1_PlaybackFile, str, sizeof(str), ",\n");
  if (n == 0) { fileposition = filesize; return; }
  if (str[n - 1] == ',' || str[n - 1] == '\n') str[n - 1] = 0;
  rightnow = millis();
  commandtime = rightnow + strtol(str, NULL, 10);

  n = readField(&Z1_PlaybackFile, str, sizeof(str), ",\n");
  if (n == 0) { fileposition = filesize; return; }
  if (str[n - 1] == ',' || str[n - 1] == '\n') str[n - 1] = 0;
  nextcommand = str;

  fileposition = Z1_PlaybackFile.position();
}

void Playback_Stop() {
  if (Z1_PlaybackFile) {
    Z1_PlaybackFile.close();
  }
  Z1_PlaybackCommandLoaded = false;
  memset(Z1_PlaybackLoco, 0, sizeof(Z1_PlaybackLoco));
  Z1_Accessory95Flag = 0;
  Serial.println("Playback stopped.");
}

// During an Acc 95 recording, write the speed-0 stop sent when a cab slot is reassigned (Acc 96 / 97),
// so playback stops the same locos
void Record_Stop(byte slot, int address) {

  if (Z1_Accessory95Flag != 2) return;
  rightnow = millis();
  myfile = SD.open(Z1_Accessory95File , FILE_WRITE);
  Serial.print("Writing to file:");
  if (myfile) {
    myfile.print(rightnow - Z1_LastCommandTime);
    myfile.print("\n");
    myfile.print("<t ");
    myfile.print(slot);
    myfile.print(" ");
    myfile.print(address);
    myfile.print(" 0 0>");
    myfile.print("\n");
    myfile.close();
  }
  else {
    Serial.println("error opening file");
  }
  Z1_LastCommandTime = rightnow;

}

void Accessory96() {

  if (Z1_Data[24] == 0 && AccessoryNo >= 1 && AccessoryNo < 17) {
    Assign_Slot(AccessoryNo, LocoNumber);
    LocoNumber = 0;
  }

  else if (Z1_Data[24] == 1)  {
    if ( AccessoryNo == 10 ) {
      AccessoryNo = 0;
    }
    if ( LocoNumber == 0 ) {
      LocoNumber = AccessoryNo;
    }
    else {
      LocoNumber = (LocoNumber * 10) + AccessoryNo;
    }
  }

}

// Put loco address 'address' in cab slot k (1-16) and save the roster - Acc 96 and the touch menu.
// The loco leaving the slot keeps running as it was (DCC or Zero 1) - stop it from a phone, with an
// emergency stop or track power off, or by putting it back in a slot.
void Assign_Slot(byte k, int address) {

  if (k < 1 || k > 16) return;
  Z1_LocoSpeed[k] = 0;
  Z1_LocoDirection[k] = 0;

  Z1_LocoNo[k] = address;
  Z1_ShownSpeed[k] = 0;                      // the row now shows the new loco, stopped until it's driven
  Z1_ShownDirection[k] = 0;
  Update_Screen_Locos();
  Draw_Speed(k);
  Menu_Loco_Changed(k, -1);

  Save_Roster(Z1_Accessory97Bank);

}

// Write the 16 cab slot addresses to roster (settings bank) file Z_DCC_<bank>.txt
void Save_Roster(byte bank) {

  String Filename = String (Z1_Settings_File + String(bank, DEC) + ".txt");
  if (SD.exists(Filename)) {
    SD.remove(Filename);
  }
  myfile = SD.open(Filename , FILE_WRITE);

  Serial.print("Writing to file : ");
  if (myfile) {
    for (j = 1; j <= 16; j++) {
      myfile.print(Z1_LocoNo[j]);
      myfile.print( "\n" );
    }
    myfile.close();
    Serial.println("done.");
  }
  else {
    Serial.println("error opening file");
  }

}

// true if the recording being played back currently has this loco address moving
bool Playback_Is_Driving(int address) {
  if (address <= 0) return false;
  for (byte s = 1; s <= 16; s++) {
    if (Z1_PlaybackLoco[s] == address) return true;
  }
  return false;
}

void Accessory97() {

  String Filename;

  if ( AccessoryNo <= 10 ) {
    int k;
    Z1_Accessory97Bank = byte(AccessoryNo);

    for (k = 1 ; k <= 16 ; k++) {           // locos leaving their slots keep running as they were
      Z1_LocoSpeed[k] = 0;
      Z1_LocoDirection[k] = 0;
    }

    Filename = String (Z1_Settings_File + String(Z1_Accessory97Bank, DEC) + ".txt");
    if (SD.exists(Filename)) {
      myfile = SD.open(Filename , FILE_WRITE);
      if (myfile) {

        myfile.seek(0);
        size_t n;
        char str[20];
        j = 1;

        while (true) {
          n = readField(&myfile, str, sizeof(str), ",\n");

          if (n == 0) break;

          if (str[n - 1] == ',' || str[n - 1] == '\n') {
            Serial.print(str[n - 1] == ',' ? F("comma: ") : F("endl:  "));

            str[n - 1] = 0;
          }

          else {

            Serial.print(myfile.available() ? F("error: ") : F("eof:   "));
          }

          if (j > 16) break;                  // only 16 cab slots - ignore any extra lines
          Z1_LocoNo[j++] = atoi(str);

        }
      }

      myfile.close();
    }

    Update_Screen_Locos();
    for (k = 1; k <= 16; k++) {              // rows now show the new bank's locos, stopped until driven
      Z1_ShownSpeed[k] = 0;
      Z1_ShownDirection[k] = 0;
      Draw_Speed(k);
    }
    Menu_Loco_Changed(0, -1);
  }

  Z1_Accessory97Flag = 0;
  LocoNumber = 0;
  Relay_Restore();

}

// Accessory 98: n<- headlight (F0), n-> whistle (F2) for cab slot n
void Accessory98() {

  LocoNumber = AccessoryNo;
  if (Z1_Data[24] == 1) Loco_Function(LocoNumber, 0);   // n<- headlight on / off
  else                  Whistle(LocoNumber);            // n-> whistle

}

// Accessory 99: first key picks the cab slot, then each key toggles that function: 1-31 = F1-F31, 32 = F0
void Accessory99() {

  if ( LocoNumber == 0 ) {
    LocoNumber = AccessoryNo;
  }

  else  {
    Z1_Accessory99Function = AccessoryNo;

    if ( Z1_Accessory99Function == 32 )  {
      Z1_Accessory99Function = 0;
    }

    if (Z1_Accessory99Function <= 31) Loco_Function(LocoNumber, Z1_Accessory99Function);
  }

}

// Status line: same position on the loco screen and the map screen
int Status_Line_Y() {
  return MAP_STATUS_Y;
}

void Update_Screen_Track_Type() {

  tft.setRotation(1);
  tft.setTextColor(BLACK, Z_DCC_RED);
  tft.setTextSize(2);
  tft.setCursor(40, Status_Line_Y());

  if (Z1_TrackType == 2)       { tft.print("A:MAIN    B:PROG    "); }
  else if (Z1_TrackType == 3)  { tft.print("A:MAIN    B:DC (999)"); }
  else if (Z1_TrackType == 4)  { tft.print("A:DC(999) B:DCX(998)"); }
  tft.setRotation(0);

}

void Draw_Track_Power() {

  tft.setRotation(1);
  tft.setTextSize(2);
  tft.setCursor(282, Status_Line_Y());       // 16 characters: 282-474
  if (Z1_ProgPower == 2) {                  // Acc 91: R8216 charging on TRACK B (as Main)
    tft.setTextColor(Z_DCC_RED, BLACK);
    tft.print("TRACK B CHARGING");
  }
  else if (Z1_ProgPower) {                  // Acc 91 programming: only the program track is powered
    tft.setTextColor(Z_DCC_RED, BLACK);
    tft.print("TRACK POWER PROG");
  }
  else if (Z1_TrackPowerOn) {
    tft.setTextColor(BLACK, Z_DCC_RED);
    tft.print(" TRACK POWER ON ");
  }
  else if (Z1_PowerPending) {               // chosen in Acc 90, not on yet
    tft.setTextColor(Z_DCC_RED, BLACK);
    tft.print(" TRACK POWER ON ");
  }
  else {
    tft.setTextColor(Z_DCC_RED, BLACK);
    tft.print("TRACK POWER OFF ");
  }
  tft.setRotation(0);

}

// Reconnect the Zero 1 / MicroMimic feed at the end of an Accessory 90-99 function,
// but only while DCC track power is on - Zero 1 power goes on and off with DCC power
void Relay_Restore() {

  digitalWrite(Z1_Relay_Pin, Z1_TrackPowerOn ? LOW : HIGH);

}

void Track_Power_On() {

  Z1_TrackPowerOn = true;
  Z1Bus_Power(true);                        // Zero 1 accessory bus on with DCC power
  Serial.println(F("Track power ON"));
  Draw_Track_Power();
  DCC_Out.println("<1>");
  if (Z1_WiFi2) Serial3.println(F("@<p1>"));   // the phones

}

void Track_Power_Off() {

  Z1_TrackPowerOn = false;
  Z1Bus_Power(false);                       // Zero 1 accessory bus off with DCC power
  Serial.println(F("Track power OFF"));
  digitalWrite(Z1_Relay_Pin, HIGH);         // Zero 1 / MicroMimic feed off with DCC power
  Draw_Track_Power();
  DCC_Out.println("<0>");
  if (Z1_WiFi2) Serial3.println(F("@<p0>"));

}

// Text sits centred in the sloped panel rows (ZeroPanel.h): zpRowCentreY(row, x) gives the row's
// centre at column x, so each character follows the slope of its row
void Update_Screen_Locos() {
  int k;

  if (Z1_Screen != SCREEN_LOCO) return;

  tft.setTextColor(BLACK, Z_DCC);
  tft.setTextSize(2);

  for (k = 1; k <= 16; k++)   {
    Draw_Loco_ID(k);
  }

}

// CAB ID for slot k: yellow background while its headlight (F0) is on
void Draw_Loco_ID(int k) {

  if (Z1_Screen != SCREEN_LOCO) return;

  tft.setRotation(ZP_BASE_ROT);              // loco screen is drawn in portrait, whatever was drawn last
  tft.setFont();
  tft.setTextColor(BLACK, Slot_Fn(k, 0) ? YELLOW : Z_DCC);
  tft.setTextSize(2);
  tft.setCursor(10, zpRowCentreY(k, 34) - 7);         // 4 characters wide, centred at x = 34
  tft.print("    ");
  tft.setCursor(10, zpRowCentreY(k, 34) - 7);
  tft.print(String(Z1_LocoNo[k], DEC));

}

// Zero 1 control change: show the cab's new speed and direction
void Update_Screen_Speeds(int k) {

  Z1_ShownSpeed[k] = Z1_LocoSpeed[k];
  Z1_ShownDirection[k] = Z1_LocoDirection[k];
  Draw_Speed(k);

}

// Draw the speed arrows for cab k from what is currently shown
void Draw_Speed(int k) {
  int l;

  if (Z1_Screen != SCREEN_LOCO) return;

  tft.setRotation(ZP_BASE_ROT);              // loco screen is drawn in portrait, whatever was drawn last
  tft.setFont();
  tft.setTextColor(BLUE, Z_DCC);
  tft.setTextSize(2);

  for (l = 1; l <= 15; l++) {
    tft.setCursor(((l * 10) + 58), zpRowCentreY(k, (l * 10) + 64) - 7);
    tft.print(" ");
  }

  for (l = 1; l <= ((Z1_ShownSpeed[k] / 9) + 1); l++) {
    tft.setCursor(((l * 10) + 58), zpRowCentreY(k, (l * 10) + 64) - 7);

    if (Z1_ShownDirection[k] == 1) {
      tft.print(">");
    }
    else {
      tft.print("<");
    }
  }

}

// ---- WiFi bridge status dot (left end of the status line) ----
//   no dot  - nothing heard from the ESP8266 (bridge not running, or no link from the ESP back to RX3)
//   white   - bridge running, looking for the WiFi network
//   yellow  - on the WiFi, but can't reach DCC-EX
//   green   - connected to DCC-EX
//   grey    - the bridge has stopped reporting
//   magenta - something is arriving on RX3 but it isn't the bridge's status lines
//             (the ESP is running other firmware or an older bridge, or the baud rate is wrong)
//   blinks black briefly - the bridge has just received commands from the Mega (TX3 -> ESP works)
void Read_Bridge() {

  static char line[128];                   // status and log lines (longer ones, e.g. DCC-EX replies, skipped)
  static byte len = 0;
  static bool skip = false;
  static byte otherLines = 0;               // complete lines on RX3 that weren't bridge status lines

  while (Serial3.available()) {
    char c = Serial3.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = 0;
#if ZDCC_DCCEX_SERIAL
      if (!skip && line[0] == '&' && line[1] == '<') { Serial1.println(line + 1); len = 0; continue; }   // a phone's DCC-EX command
#endif
      if (!skip && Z1_WiFi2 && line[0] == '<') { App_Command(line); len = 0; continue; }   // a phone command
      bool status = !skip && Bridge_Line(line);
      // Only call it "data from something else" (magenta) after 3 real lines of text - stray bytes
      // or noise on an unconnected RX3 pin (no WiFi bridge, e.g. a Mega Pro) show no dot at all
      if (!status && len > 0 && Z1_BridgeState == 0 && ++otherLines >= 3) {
        Z1_BridgeState = 8;
        Serial.println(F("WiFi bridge: data on RX3, but no status lines"));
        Draw_Bridge();
      }
      len = 0;
      skip = false;
    }
    else if (len < sizeof(line) - 1) line[len++] = c;
    else skip = true;                        // too long: a DCC-EX reply, not a status line
  }

  unsigned long now = millis();
  // the slot list for the ESP: every 10 s, and straight away when a slot's address changes
  if (Z1_WiFi2) {
    static unsigned long lastList = 0;
    static long lastSum = -1;
    long sum = 0;
    for (byte k = 1; k <= 16; k++) sum = sum * 31 + Z1_LocoNo[k];
    if (sum != lastSum || now - lastList >= 10000) {
      lastSum = sum; lastList = now;
      Serial3.print(F("@ZS"));
      for (byte k = 1; k <= 16; k++) { Serial3.print(' '); Serial3.print(Z1_LocoNo[k]); }
      Serial3.println();
      Serial3.print(F("@R "));                // recording or not (non-slot locos come here while recording)
      Serial3.println(Z1_Accessory95Flag == 2 ? 1 : 0);
    }
  }
  // how DCC-EX is reached - every 2 s until the ESP shows it knows (serial), then every 10 s
  {
    static unsigned long lastDX = 0;
    bool known = !ZDCC_DCCEX_SERIAL || Z1_BridgeReported == 5;
    if (now - lastDX >= (known ? 10000UL : 2000UL)) {
      lastDX = now;
      Serial3.println(ZDCC_DCCEX_SERIAL ? F("@DX S") : F("@DX W"));
    }
  }
#if ZDCC_DCCEX_SERIAL
  Read_DCCEX();                             // DCC-EX on Serial1: its replies, and the status dot
#else
  if (Z1_BridgeState >= 1 && Z1_BridgeState <= 4 && now - Z1_BridgeHeard > 6000) {
    Z1_BridgeState = 9;
    Serial.println(F("WiFi bridge: stopped reporting"));
    Draw_Bridge();
  }
#endif
  if (Z1_BridgeFlashing && (long)(now - Z1_BridgeFlashUntil) >= 0) {
    Z1_BridgeFlashing = false;
    Draw_Bridge();
  }

}

#if ZDCC_DCCEX_SERIAL
// DCC-EX wired to Serial1: pass what it sends to the phones (through the ESP), ping it, and show the
// link on the status dot (white = waiting, green = answering, yellow = not answering)
void Read_DCCEX() {
  static char buf[128];
  static byte len = 0;
  static bool in = false;
  static unsigned long heard = 0, lastPing = 0;
  static bool everHeard = false;
  unsigned long now = millis();

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '<') { in = true; len = 0; }
    if (!in) continue;
    if (len < sizeof(buf) - 1) buf[len++] = c; else { in = false; continue; }   // too long: dropped
    if (c == '>') {
      in = false;
      buf[len] = 0;
      heard = now; everHeard = true;
      // to the phones - not DCC-EX's diagnostics (<* ... *>) or the answers to the ping (<# ...>)
      if (Z1_WiFi2 && buf[1] != '*' && buf[1] != '#') { Serial3.print('@'); Serial3.println(buf); }
    }
  }

  if (now - lastPing >= 10000) {            // DCC-EX answers <#> with <# n>
    lastPing = now;
    Serial1.println(F("<#>"));
  }

  byte want = !everHeard ? (now < 20000 ? 1 : 2) : (now - heard < 25000 ? 3 : 2);
  if (want != Z1_BridgeState) {
    Z1_BridgeState = want;
    if (want == 3) Serial.println(F("DCC-EX: answering on Serial1"));
    else if (want == 2) Serial.println(F("DCC-EX: not answering on Serial1 - check the wiring and SERIALn_COMMANDS in its config.h"));
    Draw_Bridge();
  }
}
#endif

// true if 'line' is a bridge status line ("#ZB state count [phones]") or a log line ("#ZL text")
bool Bridge_Line(const char *line) {

  if (strncmp_P(line, PSTR("#ZL "), 4) == 0) {   // WiFi 2.0: a line for the serial log
    Serial.print(F("WiFi: "));
    Serial.println(line + 4);
    Z1_BridgeHeard = millis();
    return true;
  }

  int state, phones = -1;
  unsigned int count;
  if (strncmp_P(line, PSTR("#ZB "), 4) != 0) return false;
  int n = sscanf_P(line + 4, PSTR("%d %u %d"), &state, &count, &phones);
  if (n < 2 || state < 1 || state > 5) return false;
  Z1_BridgeReported = state;
  if (n == 3 && !Z1_WiFi2) {
    Z1_WiFi2 = true;                          // a WiFi 2.0 ESP: phone commands and @ lines from now on
    Serial.println(F("WiFi: Zero DCC WiFi 2.0 found"));
  }
  if (n == 3 && phones != Z1_BridgePhones) {
    Z1_BridgePhones = phones;
    Serial.print(F("WiFi: phones connected: "));
    Serial.println(phones);
  }

  Z1_BridgeHeard = millis();
#if ZDCC_DCCEX_SERIAL
  return true;                              // the dot shows the serial link to DCC-EX (Read_DCCEX)
#endif
  if (state != Z1_BridgeState) {
    Z1_BridgeState = state;
    if (state == 1) Serial.println(F("WiFi bridge: looking for DCC-EX's WiFi"));
    else if (state == 2) Serial.println(F("WiFi bridge: on DCC-EX's WiFi, can't reach DCC-EX"));
    else if (state == 3) Serial.println(F("WiFi bridge: connected to DCC-EX"));
    else Serial.println(F("WiFi bridge: DCC-EX not found - Zero 1 only"));
    Draw_Bridge();
  }
  if (Z1_BridgeCountKnown && count != Z1_BridgeCount) {
    Z1_BridgeFlashing = true;
    Z1_BridgeFlashUntil = millis() + 150;
    Draw_Bridge();
  }
  Z1_BridgeCount = count;
  Z1_BridgeCountKnown = true;
  return true;

}

// ---- recording cabs (stage 4): cab 9991-9999 = recording 1-9, cab 9990 = recording 10
//   F1 Play, F2 Record (press twice within 3 s - it replaces the recording), F0 Stop.
//   Only a press (function on) counts; afterwards the cab's buttons are set to what is really happening:
//   F1 lit while it plays, F2 lit while it records (and briefly after the first Record press).
const unsigned long REC_ARM_MS = 3000;
byte Rec_Armed = 0;                          // recording whose Record was pressed once (0 = none)
unsigned long Rec_ArmedAt = 0;

int Rec_Cab(long cab) {                      // recording number for a cab, or 0
  if (cab == 9990) return 10;
  if (cab >= 9991 && cab <= 9999) return (int)(cab - 9990);
  return 0;
}

// set recording n's cab buttons in the apps to what's happening now
void Rec_Lights(int n) {
  if (!Z1_WiFi2 || n < 1 || n > 10) return;
  unsigned long bits = 0;
  if (Z1_Accessory95Flag == 3 && Z1_PlaybackNumber == n) bits |= 1UL << 1;   // F1 playing
  if (Z1_Accessory95Flag == 2 && Z1_RecordNumber == n) bits |= 1UL << 2;     // F2 recording
  if (Rec_Armed == n) bits |= 1UL << 2;                                      // F2 armed
  Serial3.print(F("@<l "));
  Serial3.print(n == 10 ? 9990 : 9990 + n);
  Serial3.print(F(" 0 0 "));
  Serial3.print(bits);
  Serial3.println('>');
}

void Rec_Function(int n, int fn, bool on) {
  if (on) {
    if (fn == 1) {                           // Play
      Rec_Armed = 0;
      if (!Z1_TrackPowerOn) Serial.println(F("WiFi app: play ignored - track power is off"));
      else if (Z1_Accessory95Flag == 2) Serial.println(F("WiFi app: play ignored - stop the recording first"));
      else {
        if (Z1_Accessory95Flag == 3) Playback_Stop();       // last press wins
        if (Z1_Accessory95Flag == 1) Accessory95_Cancel();  // Acc 95 was waiting for a number
        Serial.print(F("WiFi app: play recording "));
        Serial.println(n);
        Start_Playback(n);
      }
    }
    else if (fn == 2) {                      // Record (twice)
      if (!Z1_TrackPowerOn) { Rec_Armed = 0; Serial.println(F("WiFi app: record ignored - track power is off")); }
      else if (Z1_Accessory95Flag == 2 || Z1_Accessory95Flag == 3) {
        Rec_Armed = 0;
        Serial.println(F("WiFi app: record ignored - a recording is already recording or playing"));
      }
      else if (Rec_Armed == n && millis() - Rec_ArmedAt <= REC_ARM_MS) {
        Rec_Armed = 0;
        if (Z1_Accessory95Flag == 1) Accessory95_Cancel();
        Serial.print(F("WiFi app: record recording "));
        Serial.println(n);
        Start_Recording(n);
      }
      else {
        int old = Rec_Armed;
        Rec_Armed = n; Rec_ArmedAt = millis();
        if (old && old != n) Rec_Lights(old);
        Serial.print(F("WiFi app: press Record again within 3 s to record recording "));
        Serial.print(n);
        Serial.println(F(" (it replaces what's there)"));
      }
    }
    else if (fn == 0) {                      // Stop
      if (Rec_Armed == n) Rec_Armed = 0;
      if (Z1_Accessory95Flag == 3 && Z1_PlaybackNumber == n) Playback_Stop();
      else if (Z1_Accessory95Flag == 2 && Z1_RecordNumber == n) Recording_Stop();
    }
  }
  Rec_Lights(n);
}

// called from loop(): the apps' buttons and the ESP follow recording / playback however it starts or
// ends (Acc 95, the menu, a phone, the end of the file); Record arming runs out after 3 s
void Rec_Watch() {
  static byte lastFlag = 0;
  static int lastNum = 0;
  byte f = (Z1_Accessory95Flag == 2 || Z1_Accessory95Flag == 3) ? Z1_Accessory95Flag : 0;
  int num = f == 2 ? Z1_RecordNumber : (f == 3 ? Z1_PlaybackNumber : 0);
  if (f != lastFlag || num != lastNum) {
    if (lastNum && lastNum != num) Rec_Lights(lastNum);
    if (num) Rec_Lights(num);
    if ((f == 2) != (lastFlag == 2) && Z1_WiFi2) { Serial3.print(F("@R ")); Serial3.println(f == 2 ? 1 : 0); }
    lastFlag = f; lastNum = num;
  }
  if (Rec_Armed && millis() - Rec_ArmedAt > REC_ARM_MS) {
    byte n = Rec_Armed;
    Rec_Armed = 0;
    Rec_Lights(n);
  }
}

// write one command line to the recording being made (with the time since the last one)
void Record_Write(const String &cmd) {
  if (Z1_Accessory95Flag != 2) return;
  rightnow = millis();
  myfile = SD.open(Z1_Accessory95File , FILE_WRITE);
  if (myfile) {
    myfile.print(rightnow - Z1_LastCommandTime);
    myfile.print("\n");
    myfile.print(cmd);
    myfile.print("\n");
    myfile.close();
  }
  else Serial.println(F("error opening file"));
  Z1_LastCommandTime = rightnow;
}

// A phone command passed on by the ESP (WiFi 2.0): <t [reg] cab speed dir>, <1>, <0>, <!>, <a n state>, <F cab fn state>
void App_Command(const char *line) {

  if (line[1] == 't') {
    long v[4]; int n = sscanf_P(line + 2, PSTR("%ld %ld %ld %ld"), &v[0], &v[1], &v[2], &v[3]);
    long cab, speed, dir;
    if (n == 3) { cab = v[0]; speed = v[1]; dir = v[2]; }
    else if (n == 4) { cab = v[1]; speed = v[2]; dir = v[3]; }
    else return;
    if (Rec_Cab(cab)) { Rec_Lights(Rec_Cab(cab)); return; }   // a recording cab: no speed, slider back to 0
    if (speed < 0) speed = 0;
    if (speed > 126) speed = 126;
    if (!Z1_TrackPowerOn) { Serial.println(F("WiFi app: loco ignored - track power is off")); return; }
    bool found = false;
    for (byte k = 1; k <= 16; k++) {
      if (Z1_LocoNo[k] != cab) continue;
      found = true;
      Z1_LocoSpeed[k] = speed;
      Z1_LocoDirection[k] = dir ? 1 : 0;
      Slot_Send(k);
    }
    if (!found && ((cab >= 1 && cab <= 16) || Z1_Accessory95Flag == 2)) {
      // a Zero 1 loco in no slot (driven by its cab ID), or any loco in no slot while recording:
      // to DCC-EX and the Zero 1 bus, and into the recording as slot 0 (playback drives it, not shown)
      DCC_Out.print(F("<t "));
      DCC_Out.print(cab);
      DCC_Out.print(' ');
      DCC_Out.print(speed);
      DCC_Out.print(' ');
      DCC_Out.print(dir ? 1 : 0);
      DCC_Out.println('>');
      Z1Bus_Cab(cab, speed, dir);
      if (Z1_Accessory95Flag == 2)
        Record_Write("<t 0 " + String(cab) + " " + String(speed) + " " + String(dir ? 1 : 0) + ">");
      if (Z1_WiFi2) {
        byte sb = speed == 0 ? 0 : speed + 1;
        if (dir) sb |= 0x80;
        uint32_t *fn = Fn_Find(cab, false);
        Serial3.print(F("@<l "));
        Serial3.print(cab);
        Serial3.print(F(" 0 "));
        Serial3.print(sb);
        Serial3.print(' ');
        Serial3.print(fn ? *fn : 0UL);
        Serial3.println('>');
      }
    }
    else if (!found) { Serial.print(F("WiFi app: no slot has loco ")); Serial.println(cab); }
  }
  else if (line[1] == '1') {
    Serial.println(F("WiFi app: track power on"));
    Z1_PowerPending = false;
    Track_Power_On();
    if (!Any_Function_Active()) Relay_Restore();   // Zero 1 feed back on, unless a 90-99 function is open
  }
  else if (line[1] == '0') {
    Serial.println(F("WiFi app: track power off"));
    Z1_PowerPending = false;
    Track_Power_Off();
  }
  else if (line[1] == 'a') {                  // a phone's DCC accessory / turnout (the ESP sent it to DCC-EX)
    int acc, state;
    if (sscanf_P(line + 2, PSTR("%d %d"), &acc, &state) != 2 || acc < 1 || acc > 2040) return;
    if (!Z1_TrackPowerOn) { Serial.println(F("WiFi app: accessory ignored - track power is off")); return; }
    if (acc > 89) {                           // not a Zero 1 point: the map, and the recording if recording
      dccMapSetPoint(tft, acc, state != 0);
      if (Z1_Accessory95Flag == 2) {
        Record_Write("<a " + String(acc) + " " + String(state ? 1 : 0) + ">");
        Serial.print(F("WiFi app: accessory "));
        Serial.print(acc);
        Serial.println(F(" recorded"));
      }
      return;
    }
    Serial.print(F("WiFi app: accessory "));
    Serial.print(acc);
    Serial.println(state ? F(" <-") : F(" ->"));
    Accessory_Do(acc, state ? 1 : 0, false);  // the ESP has sent it to DCC-EX already
  }
  else if (line[1] == 'F') {                  // loco function
    int addr, fn, state;
    if (sscanf_P(line + 2, PSTR("%d %d %d"), &addr, &fn, &state) != 3) return;
    if (Rec_Cab(addr)) { Rec_Function(Rec_Cab(addr), fn, state != 0); return; }
    Serial.print(F("WiFi app: loco "));
    Serial.print(addr);
    Serial.print(F(" F"));
    Serial.print(fn);
    Serial.println(state ? F(" on") : F(" off"));
    Loco_Fn_Send(addr, fn, state != 0);
  }
  else if (line[1] == '!') {
    Serial.println(F("WiFi app: emergency stop - all slots to 0"));
    if (Z1_Accessory95Flag == 3) Playback_Stop();   // and a recording playing back
    for (byte k = 1; k <= 16; k++) {
      if (Z1_LocoSpeed[k] == 0) continue;
      Z1_LocoSpeed[k] = 0;
      if (Z1_TrackPowerOn) Slot_Send(k); else Update_Screen_Speeds(int (k));
    }
  }

}

void Draw_Bridge() {

  if (Z1_BridgeState == 0) return;
  uint16_t colour;
  if (Z1_BridgeFlashing)          colour = BLACK;
  else if (Z1_BridgeState == 1)   colour = WHITE;
  else if (Z1_BridgeState == 2)   colour = YELLOW;
  else if (Z1_BridgeState == 3)   colour = GREEN;
  else if (Z1_BridgeState == 4)   colour = BLUE;
  else if (Z1_BridgeState == 8)   colour = MAGENTA;
  else                            colour = GRAY;
  tft.setRotation(1);
  tft.fillCircle(14, Status_Line_Y() + 8, 7, BLACK);
  tft.fillCircle(14, Status_Line_Y() + 8, 5, colour);
  tft.setRotation(0);

}

// Called by the map module after it draws the map screen:
// red band right across the top of the screen (from the top edge to a few pixels
// below the text), then the usual status text on top of it
void showMapStatus() {

  tft.setRotation(1);
  tft.fillRect(0, 0, tft.width(), MAP_STATUS_Y + 16 + 4, Z_DCC_RED);
  tft.setRotation(0);
  Update_Screen_Track_Type();
  Draw_Track_Power();
  Draw_Bridge();
  Draw_Screen_Arrows();

}

// Called by the map module when Accessory 93 is turned off
void showLocoScreen() {
  int k;

  drawZeroPanel(tft);
  Update_Screen_Locos();
  for (k = 1; k <= 16; k++) {
    Draw_Speed(k);
  }
  Update_Screen_Track_Type();
  Draw_Track_Power();
  Draw_Bridge();
  Draw_Screen_Arrows();

}
