# OpenWindowAlarm

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://travis-ci.org/ArminJo/Arduino-OpenWindowAlarm.svg?branch=master)](https://travis-ci.org/ArminJo/Arduino-OpenWindowAlarm)

[Youtube video](https://youtu.be/6l_QOM59nyc)

[Instructable](https://www.instructables.com/id/Arduino-Open-Window-Detector-for-Winter/)

Place this on a windowsill and you will be alarmed if you leave the window open longer than five minutes.
It senses the falling temperature and thus works best in winter. It requires only 0.006 milliampere. This means one battery will last the whole winter.
![OpenWindowAlarm on a windowsill](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/OpenWindowAlarm.jpg)
OpenWindowAlarm on a windowsill

# Function
Every 24 seconds a reading is taken of the ATtiny internal temperature sensor which has a resolution of 1 degree.
If temperature is lower than the "old" temperature value an alarm is issued five minutes later if by then the condition still holds true.
A detection of an open window is indicated by a longer 20ms blink and a short click every 24 seconds.
A low battery is indicated by beeping and flashing the LED every 24 seconds. Only the beep (not the flash) is significantly longer than the beep for an open window detection.

# How to make your own
The parts you need
![Parts](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Parts.jpg)
![Battery Case](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/BatteryCase.jpg)


## Programming the Digispark board
### Installation of Arduino IDE
Install the Digispark board for the Arduino IDE as described in http://digistump.com/wiki/digispark/tutorials/connecting
Since we want to save power, the board clock is switched to 1MHz in our setup() so please choose **Digispark (1mhz - No USB)** 
as board in the *Tools* menu in order to set the right timing.

### Compile and upload the program to the board
In the Arduino IDE create a new sketch with *File/New* and name it e.g. "`OpenWindowAlarm`".
Copy the code from [OpenWindowAlarm.ino](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/OpenWindowAlarm.ino)

**OR**

Download and extract the repository. Open the sketch with File -> Open... and select the "`OpenWindowAlarm`" folder. 

Compile and upload it. Keep in mind, that upload will not work if the speaker is connected.
If everything worked, the build in LED of the Digispark will blink 5 times and then start flashing after 8 seconds with an interval of 24 sconds.

## Power reduction
Before power reduction changes
![Final power reduction](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Digispark.jpg)

We now have a Digispark board that uses 6/9.5 mA at 3,7/5 Volt. With a battery of 2000mAh it will run for 9 days. But it is possible to reduce power consumption to 6-9 µA in 3 Steps.
1. Disabling the power LED by breaking the copper wire that connects the power LED to the diode with a knife or removing / disabling the 102 resistor saves 2/2.2 mA.
2. Removing the VIN voltage regulator saves 1.5/3.8 mA. We now use 2.5/3.5 mA at 3.7/5 Volt. The 2000mAh battery now lasts for 23 days.
3. Disconnecting the USB Pullup resistor (marked 152) from 5 Volt (VCC) saves the 2.5/3.5 mA. Disconnect it by breaking the copper wire on the side of the resistor that points to the ATTiny.<br/>**BUT** this also disables the USB interface and in turn the possibility to program the Digispark board via USB.

   **There is a solution:**<br/>
   If you use a battery, then just connect the resistor directly to the USB 5 Volt that is easily available at one side of the diode. 
   The correct side of the diode can be found by using a continuity tester. One side of this diode is connected to pin 8 of the ATtiny (VCC).
   The other side is connected to the USB 5V. 
   
Now the USB pullup resistor is only activated if the Digispark board is connected to USB e.g. during programming and the board consumes 5-9 µA during regular operation.
   
After power reduction changes
![Final power reduction](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-Version-Detail.jpg)

## Reset button
If you do not want to remove power to reset the alarm connect a reset button between PB5 and ground. 
I did this by connecting the unconnected VIN copper surface to PB5 and soldering the reset button directly to the VIN pin hole and the big ground surface of the removed VIN voltage regulator.

## Loudspeaker
Loudspeaker disassembly part 1
![Loudspeaker disassembly](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Loudspeaker1.jpg)
Loudspeaker disassembly part 2
![Loudspeaker disassembly](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Loudspeaker2.jpg)

OpenWindowAlarm circuit with AAA batteries
![OpenWindowAlarm circuit with battery](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-Version.jpg)

OpenWindowAlarm circuit with LiPo battery
![OpenWindowAlarm circuit with battery](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-VersionLiPo.jpg)
# Placement
To use the board place it on a windowsill and connect it to a battery or USB power bank.
If the temperature on the sill is lower than the temperature where the board was originally located it will take an additional 5 Minutes to intelligently adopt to the new start value.

# Internal Operation
* An open window is detected after `TEMPERATURE_COMPARE_AMOUNT * TEMPERATURE_SAMPLE_SECONDS` (48) seconds of reading a temperature with a value of `TEMPERATURE_DELTA_THRESHOLD_DEGREE` (2) lower than the temperature `TEMPERATURE_COMPARE_DISTANCE * TEMPERATURE_SAMPLE_SECONDS` (192 -> 3 minutes and 12 seconds) seconds before.
* The delay is implemented by sleeping 3 times at `SLEEP_MODE_PWR_DOWN` for a period of 8 seconds to reduce power consumption.
* A detection of an open window is indicated by a longer 20ms blink and a short click every 24 seconds.
   Therefore, the internal sensor has a time of 3 minutes to adjust to the outer temperature in order to capture even small changes in temperature.
   The greater the temperature change the earlier the sensor value will change and detect an open window.
* After open window detection Alarm is activated after `OPEN_WINDOW_ALARM_DELAY_MINUTES` (5).
    The alarm will not sound if `OPEN_WINDOW_ALARM_DELAY_MINUTES` (5) after detecting an open window the current temperature is greater than the minimum measured temperature (+ 1) i.e. the window has been closed already.
* Every `VCC_MONITORING_DELAY_MIN` (60) minutes the battery voltage is measured. A battery voltage below `VCC_VOLTAGE_LOWER_LIMIT_MILLIVOLT` (3550) Millivolt is indicated by beeping and flashing the LED every 24 seconds. Only the beep (not the flash) is significantly longer than the beep for an open window detection.
* The initial alarm lasts for 10 minutes. After this it is activated for a period 10 seconds with a increasing break from 24 seconds up to 5 minutes.
* After power up or reset, the inactive settling time is 5 minutes. 4:15 (or 8:30) minutes are added if the board is getting colder during the settling time, to avoid false alarms after boot.


## Travis CI
The FrequencyDetector library examples are built on Travis CI for the following boards:

- Digispark ATTiny85
