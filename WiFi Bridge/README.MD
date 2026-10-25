# Notes on how to set up an Arduino Mega / ESP8266 Combo board for WiFi connection to DCC-EX

On an Arduino Mega/ESP8266 WiFi combo board, the ESP8266 is a separate chip with its own program. You flash a small “bridge” sketch onto it once. After that the Mega just sends its commands out on Serial3, and the ESP passes them over WiFi to DCC-EX.


1. Know the DIP switches. 

The board has an 8-way DIP switch that decides what’s connected to what. The usual combo board (the RobotDyn style) works like this, but check the table printed on your board or its listing:

|What you’re doing                                             |Switches ON |
|--------------------------------------------------------------|------------|
|Upload a sketch to the ESP8266	                               | 5, 6, 7    |
|USB serial monitor to the ESP8266	                           | 5, 6       |
|Upload a sketch to the Mega	                                 | 3, 4       |
|Normal running: Mega ↔ ESP, with USB to the Mega for debug	   | 1, 2, 3, 4 |
|Mega ↔ ESP only	                                             | 1, 2       |

There’s often also a small slide switch that picks which Mega serial port connects to the ESP. Set it to RXD3/TXD3.


2. Edit the settings at the top of the bridge sketch.

If DCC-EX runs its own WiFi access point: the network name looks like DCCEX_ followed by six characters, and the password is PASS_ followed by the same six characters. DCC-EX prints both on its serial monitor at startup. Its address is normally 192.168.4.1, which is already filled in.

If DCC-EX joins your home network: use your home WiFi details instead, and give the command station a fixed IP address in your router.

Baud rate: the sketch is set to 115200. That has to match the Mega side.


3. Flash the bridge onto the ESP.

Install the ESP8266 board package in the Arduino IDE.

Select “Generic ESP8266 Module”.

Set the DIP switches to 5, 6, 7 ON and upload.

Some boards also need the reset button pressed when the upload starts.


4. What the bridge does.

It joins the WiFi and connects to DCC-EX on port 2560.

It forwards everything it receives from the Mega, and passes DCC-EX’s replies back.

If the link drops, it retries every 2 seconds.

While there’s no connection it throws incoming data away, so nothing old gets sent later.

The ESP’s blue LED is on while it’s connected to DCC-EX.


5. Change DIP switches for operations:

Set the DIP switches to 1, 2, 3, 4 ON for normal running.
