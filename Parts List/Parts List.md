<H1>ZeroDCC - Parts List</H1>


Here is the parts list for my build.  I have included some more expensive sources for some important items, which are of high quality and are known to work.  There are cheaper options available for most items listed, but do your research and be prepared that some of the every cheap options may not work (the DCC-EX team have documented this as part of their project).

<h2>The List:</h2>


1.   Arduino Mega (eg: https://www.jaycar.com.au/duinotech-mega-2560-r3-main-board/p/XC4420) 

	 This is the main unit for the ZeroDCC code and display.


2.   4” TFT screen sheild for Arduino Mega with SD Card Reader (eg: https://www.aliexpress.com/item/1005004542426818.html )

	 This is to display the current ZeroDCC configuration.


3.   DCC Sniffer (eg: https://forum.mrhmag.com/post/sma37-simple-dcc-connections-for-wide-decoder-adaptation-12217074)

	 I used Geoff Bunza’s PCB. Interestingly, this simple DCC sniffer can also sniff the Zero 1 signal.


4.   Relay module, 2 channel relay 5V (eg: https://www.aliexpress.com/item/1005002617199944.html)

	 If you are still using a MicroMimic and/or Zero 1 accessory modules (which you certainly can), these relays are used to disconnect the Zero 1 feed to those circuits when Zero 1 Functions 90-99 are being accessed by the ZeroDCC code.  Without this, your genuine Zero 1 accessories and MicroMimic will falsely change in response to ZeroDCC relayed keyboard strokes.


5.    DCC-EX system

	  I used an Arduino Mega with WiFi and a genuine Arduino MotorShield (see DCC-EX pages for details https://dcc-ex.com/) 

      (eg: https://www.jaycar.com.au/duinotech-mega-2560-r3-main-board-with-wi-fi/p/XC4421 ; and
           https://www.jaycar.com.au/arduino-motor-shield-rev3/p/XC9207)

      A few notes about installing the DCC-EX software.  The new DCC-EX installer works very well - just follow the instructions.  I've tested against Command Station v5.6.0.
	 
      In the configuration page, you have a number of options.  
	 
      Only select "I have a display" if you're going to have a screen on your DCC-EX board. This does NOT related to the screen that attaches to the ZeroDCC board. Feel free to have two, I just don't use it that way.

      Do not disable (ie keep) programming support.  If you ever want to use ZeroDCC to program locomotive or accessory modules, you'll need this enabled.

      I highly recommend selecting (ie turn on) "Advanced config" - that will allow you to configure your serial ports which is the best way to connect ZeroDCC to DCC-EX.

      Turn on WiFi support with "I have WiFi" if you have a WiFi module for your DCC-EX and you wish to connect with your phone, or use it to connect ZeroDCC to DCC-EX wirelessly.

      Don't forget to select your motor drive board from the drop down list, or you won't be able to continue with the installer!

      <B>Advanced Configuration</B>

      In this screen, I recommend leaving everything intact, and add a single new line at the bottom with the following text:

          #define SERIAL1_COMMANDS


      That will enable your serial 1 port on DCC-EX so it can connect to your ZeroDCC board directly with a single wire.

6. 	DCC-EX power supply (12-24V DC 5Amp)

  	 This is to provide power to your DCC system. The motor shield on the DCC-EX system will regulate the voltage for the track.  I used an old electronics power supply.  Using an optional buck converter (below) you can also use this supply to power the Arduino's.

7.    Cables and connectors

      In the main, I used Dupont single wire connectors.
      The biggest problem was making a cable to separate the TFT shield from the Mega and hang it over the Zero 1 locomotive list. I couldn't find any existing cables for this, so I made a custom ribbon cable.

  


<h2>Optional:</h2>

8.    Buck convertor (eg: https://www.aliexpress.com/item/1005012618202087.html)
	
      	I used this to convert the incoming 12-24V DC required for the DCC-EX motor shield to allow for a 5V DC feed to power the Arduino and components.  This results in one power source for the entire system.
  		BEWARE:  You will need to use a multimeter and adjust the buck converter when in place to ensure only 5V DC in being supplied BEFORE attaching to the Arduino units (or risk damaging them).
