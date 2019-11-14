# OpenWindowAlarm

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://travis-ci.org/ArminJo/Arduino-OpenWindowAlarm.svg?branch=master)](https://travis-ci.org/ArminJo/Arduino-OpenWindowAlarm)
[![Hit Counter](https://hitcounter.pythonanywhere.com/count/tag.svg?url=https%3A%2F%2Fgithub.com%2FArminJo%2FArduino-OpenWindowAlarm)](https://github.com/brentvollebregt/hit-counter)

Place this on a windowsill and you will be alarmed if you leave the window open longer than five minutes.
It senses the falling temperature and thus works best in winter. It requires only 0.006 milliampere. This means one battery will last the whole winter.

Der **Sourcecode** kann von [hier](https://raw.githubusercontent.com/ArminJo/Arduino-OpenWindowAlarm/master/OpenWindowAlarm/OpenWindowAlarm.ino) kopiert werden.<br/>
Das Programm ist auch als Beispiel der Arduino "ATtinySerialOut" Bibliothek - unter *Datei -> Beispiele -> Beispiele aus eigenen Bibliotheken* verf�gbar. Die Bilbliothek kann mit *Werkzeuge -> Bibliotheken verwalten...* oder *Strg+Umschalt+I* geladen werden. Dabei "SerialOut" als Suchstring benutzen.

The **sourcecode** can be copied from [here](https://raw.githubusercontent.com/ArminJo/Arduino-OpenWindowAlarm/master/OpenWindowAlarm/OpenWindowAlarm.ino).<br/>
The application is also available as an example of the Arduino "ATtinySerialOut" library - use *File -> Examples -> Examples from Custom Libraries*. You can load the library with *Tools -> Manage Libraries...* or *Ctrl+Shift+I*. Use "SerialOut" as filter string.

| YouTube video | Instructable |
|---------|------|
| [![YouTube video](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/OpenWindowAlarm1.jpg)](https://youtu.be/6l_QOM59nyc)<br/> OpenWindowAlarm on a windowsill | [![Instructable](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/instructables-logo-v2.png)](https://www.instructables.com/id/Arduino-Open-Window-Detector-for-Winter/) |

# Function
Every 24 seconds a reading is taken of the ATtiny internal temperature sensor which has a resolution of 1 degree.
If temperature is lower than the "old" temperature value, an alarm is issued five minutes later if by then the condition still holds true.<br/>
A detection of an open window is indicated by a longer 20 ms blink and a short click every 24 seconds.
A low battery is indicated by beeping and flashing the LED every 24 seconds. Only the beep (not the flash) is significantly longer than the beep for an open window detection.

# How to make your own
### The parts you need:
![Parts](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Parts.jpg) 
### Add one of the power supplies
| AAA battery case  | CR2032 case  | LiPo battery |
|---|---|---|
| ![AAA battery case](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/BatteryCase.jpg) | ![CR2032 holder](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/CR2032Holder.jpg) | ![LiPo battery](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/LiPo.jpg) |


## Programming the Digispark board
### Installation of Digispark for the Arduino IDE
Install the Digispark board for the Arduino IDE as described in http://digistump.com/wiki/digispark/tutorials/connecting
Since we want to save power, the board clock is switched to 1 MHz in our setup() so please choose **Digispark (1mhz - No USB)**
as board in the `Tools` menu in order to set the right timing.

### Compile and upload the program to the board
In the Arduino IDE create a new sketch with `File/New` and name it e.g. "`OpenWindowAlarm`".
Copy the code from [OpenWindowAlarm.ino](https://raw.githubusercontent.com/ArminJo/Arduino-OpenWindowAlarm/master/OpenWindowAlarm/OpenWindowAlarm.ino)

**OR**

Download and extract the repository. Open the sketch with `File/Open`... and select the "`OpenWindowAlarm`" folder.

Compile and upload it. Keep in mind, that upload will not work if the speaker is connected.
If everything works well, the built-in LED of the Digispark will blink 5 times (for the 5 minutes alarm delay) and then start flashing after 8 seconds with an interval of 24 seconds to signal each temperature reading.

## Power reduction
Before power reduction changes
![Final power reduction](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Digispark.jpg)

We now have a Digispark board that consumes 6/9.5 mA at 3,7/5 volt. With a battery of **2000 mAh** it will run for **9 days**. But it is possible to reduce power consumption to **6-9 µA** in 3 Steps.
1. **Disabling the power LED** by breaking the copper wire that connects the power LED to the diode with a knife or removing / disabling the 102 resistor saves 2/2.2 mA.
2. **Removing the VIN voltage regulator** saves 1.5/3.8 mA.<br/>
The board now needs 2.5/3.5 mA at 3.7/5 volt and the 2000mAh battery will last for 23 days.
3. **Disconnecting the USB Pullup resistor** (marked 152) from 5 volt (VCC) saves the remaining 2.5/3.5 mA. Disconnect it by breaking the copper wire on the side of the resistor that points to the ATtiny.<br/>
**This disables the USB interface** and in turn the possibility to program the Digispark board via USB. To **enable it again**, but still save power, **connect the resistor (marked 152) directly to the USB 5 volt** that is easily available at one side of the diode.<br/>
The correct side of the diode can be found by using a continuity tester. One side of this diode is connected to pin 8 of the ATtiny (VCC). The other side is connected to the USB 5 volt.

Now the USB pullup resistor is only activated if the Digispark board is connected to USB e.g. during programming and the board consumes 5-9 µA during battery operation.


## Reset button
**If you do not want to remove power to reset the alarm**, connect a reset button between PB5 and ground.
I did this by connecting the unconnected VIN copper surface to PB5 and soldering the reset button directly to the VIN pin hole and the big ground surface of the removed VIN voltage regulator.<br/><br/>
If you want to **get rid of the 5 seconds wait** for USB connection **after reset**, you can change the micronucleus kernel on the ATtiny85.
Adapt and run the `Burn_upgrade_micronucleus-t85_no_pullup.cmd` script and then reload the OpenWindowAlarm application again with the Arduino IDE.

### After power reduction changes and reset button assembly
| Both patches on front  | Reset connection on back |
|---|---|
| ![Final power reduction](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-Version-Detail_annotated.jpg) | ![Patch front](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/PatchWithResetOnBack.jpg)  |
| ![Final power reduction](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Patch1.jpg)  | ![Reset connection on back](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/ResetConnectionBack.jpg) |
|   | ![Mini-USB module](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/MiniUSBModule.jpg) |

## Loudspeaker disassembly
| Part 1 | Part 2 | 
|---|---|
| ![Loudspeaker disassembly](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Loudspeaker1.jpg) | ![Loudspeaker disassembly](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Loudspeaker2.jpg) |


## Module Samples
| | |
|---|---|
| ![OpenWindowAlarm circuit with AAA batteries](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-VersionAAA.jpg)<br/>Powered by 2 AAA batteries | ![OpenWindowAlarm circuit by CR2032](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/CR2032Front.jpg)<br/>Powered by CR2032 coin cell |
| ![OpenWindowAlarm circuit with LiPo battery](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-VersionLiPo.jpg)<br/>Powered by LiPo battery | ![OpenWindowAlarm circuit by CR2032](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/CR2032Back.jpg)<br/>Back viev with  CR2032 coin cell |

Different reset buttons and connectors
![OpenWindowAlarm circuit by CR2032](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/4Modules.jpg)

# Placement
Place the board on a windowsill and connect it to the supply.
If the temperature on the sill is lower than the temperature where the board was originally located, it will take additional 5 minutes to adopt to the new start value to avoid false alarm.
![OpenWindowAlarm circuit with LiPo battery](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/OpenWindowAlarmLiPo_2.jpg)

# Internal Operation
* An open window is detected after `TEMPERATURE_COMPARE_AMOUNT * TEMPERATURE_SAMPLE_SECONDS` (48) seconds of reading a temperature with a value of `TEMPERATURE_DELTA_THRESHOLD_DEGREE` (2) lower than the temperature `TEMPERATURE_COMPARE_DISTANCE * TEMPERATURE_SAMPLE_SECONDS` (192 -> 3 minutes and 12 seconds) seconds before.
* The delay is implemented by sleeping 3 times at `SLEEP_MODE_PWR_DOWN` for a period of 8 seconds to reduce power consumption.

* A detection of an open window is indicated by a longer 20 ms blink and a short click every 24 seconds.
   Therefore, the internal sensor has a time of 3 minutes to adjust to the outer temperature in order to capture even small changes in temperature.
   The greater the temperature change the earlier the sensor value will change and detect an open window.
   
* `OPEN_WINDOW_ALARM_DELAY_MINUTES` (5) minutes after open window detection the alarm is activated.<br/>
    The alarm will not start or an activated alarm will stop if the current temperature is greater than the minimum measured temperature (+ 1) i.e. the window has been closed already.
    
* The initial alarm lasts for 10 minutes. After this, it is activated for a period of 10 seconds with a increasing break from 24 seconds up to 5 minutes.

* Every `VCC_MONITORING_DELAY_MIN` (60) minutes the battery voltage is measured. Depending on the detected battery type at power-on (see `VCC_VOLTAGE_LIPO_DETECTION` (3.6 volt)), a battery voltage below `VCC_VOLTAGE_LOWER_LIMIT_MILLIVOLT_LIPO` (3550) or `VCC_VOLTAGE_LOWER_LIMIT_MILLIVOLT_STANDARD` Millivolt is indicated by beeping and flashing the LED every 24 seconds. Only the beep (not the flash) is significantly longer than the beep for an open window detection.

* After power-on, the inactive settling time is 5 minutes. If the board is getting colder during the settling time, 4:15 (or 8:30) minutes are added to avoid false alarms after power-on.

* If you enable DEBUG by commenting out line 49, you can monitor the serial output with 115200 baud at P2 to see what is happening.

# Revision History
### Version 1.2.2
- Converted to Serial.print.
- New PWMTone() without tone().
### Version 1.2.1
- Fixed bug in check for temperature rising after each alarm.
### Version 1.2
- Improved sleep, detecting closed window also after start of alarm, reset behavior.
- Changed LIPO detection threshold.
- Fixed analog reference bug.

### Sample TRACE output after reset
```
Changed OSCCAL from 0x52 to 0x4e
START ../src/OpenWindowAlarm.cpp
Version 1.2.1 from Nov  5 2019
Alarm delay = 5 minutes
MCUSR=0x2 LFuse=0x225 WDTCR=0x0 OSCCAL=0x78
Booting from reset 
VCC=4022mV - LIPO detected
Temp=300 Old=0 New=300
Temp=298 Old=0 New=598
Temp=298 Old=0 New=596
Temp=297 Old=0 New=595
Temp=296 Old=0 New=593
Temp=296 Old=0 New=592
Temp=296 Old=0 New=592
Temp=296 Old=0 New=592
Temp=296 Old=0 New=592
Temp=296 Old=0 New=592
Temp=296 Old=300 New=592
Detected porting to a colder place -> reset
Temp=296 Old=0 New=296
Temp=296 Old=0 New=592
...
Temp=296 Old=0 New=592
Temp=296 Old=296 New=592
Temp=296 Old=592 New=592
Temp=296 Old=592 New=592
```

