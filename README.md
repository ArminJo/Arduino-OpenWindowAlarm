# [OpenWindowAlarm](https://github.com/ArminJo/Arduino-OpenWindowAlarm)
Available as "OpenWindowAlarm" example of Arduino library "ATtinySerialOut"

### [Version 1.3.1](https://github.com/ArminJo/ATtinySerialOut/releases)

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
 [![Commits since latest](https://img.shields.io/github/commits-since/ArminJo/Arduino-OpenWindowAlarm/latest)](https://github.com/ArminJo/Arduino-OpenWindowAlarm/commits/master) 
[![Build Status](https://github.com/ArminJo/Arduino-OpenWindowAlarm/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Arduino-OpenWindowAlarm/actions)
[![Hit Counter](https://hitcounter.pythonanywhere.com/count/tag.svg?url=https%3A%2F%2Fgithub.com%2FArminJo%2FArduino-OpenWindowAlarm)](https://github.com/brentvollebregt/hit-counter)

Place this on a windowsill and you will be alarmed if you leave the window open longer than five minutes.
It senses the falling temperature and thus works best in winter. It requires only 0.026 milliampere. This means one battery will last the whole winter.

### [Driver installation ->](https://github.com/ArminJo/Arduino-OpenWindowAlarm#driver-installation)

### Sourcecode
Der **Sourcecode** kann von [hier](https://raw.githubusercontent.com/ArminJo/Arduino-OpenWindowAlarm/master/OpenWindowAlarm/OpenWindowAlarm.ino) kopiert werden.<br/>
Das Programm ist auch als Beispiel der Arduino "ATtinySerialOut" Bibliothek - unter *Datei -> Beispiele -> Beispiele aus eigenen Bibliotheken* verf�gbar. Die Bibliothek kann mit *Werkzeuge -> Bibliotheken verwalten...* oder *Strg+Umschalt+I* geladen werden. Dabei "SerialOut" als Suchstring benutzen.

The **sourcecode** can be copied from [here](https://raw.githubusercontent.com/ArminJo/Arduino-OpenWindowAlarm/master/OpenWindowAlarm/OpenWindowAlarm.ino).<br/>
The application is also available as an example of the [Arduino ATtinySerialOut library](https://github.com/ArminJo/ATtinySerialOut) - use *File -> Examples -> Examples from Custom Libraries*.
You can load the library with *Tools -> Manage Libraries...* or *Ctrl+Shift+I*. Use "SerialOut" as filter string.

| YouTube video | Instructable |
|---------|------|
| [![YouTube video](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/OpenWindowAlarm1.jpg)](https://youtu.be/6l_QOM59nyc)<br/> OpenWindowAlarm on a windowsill | [![Instructable](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/instructables-logo-v2.png)](https://www.instructables.com/id/Arduino-Open-Window-Detector-for-Winter/) |

# Functional overview
Every 24 seconds a reading is taken of the ATtiny internal temperature sensor which has a resolution of 1 degree.
If temperature is lower than the "old" temperature value, an **alarm is issued five minutes later** if by then the condition still holds true.<br/>
**Detection of an open window** is indicated by a longer 20 ms blink and a short click every 24 seconds.
**Low battery** is indicated by beeping and flashing the LED every 24 seconds. The beep and the flash are longer than for an open window detection.

# How to make your own
### The parts you need:
![Parts](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Parts.jpg) 
### Add one of the power supplies
| AAA battery case  | CR2032 case  | LiPo battery |
|---|---|---|
| ![AAA battery case](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/BatteryCase.jpg) | ![CR2032 holder](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/CR2032Holder.jpg) | ![LiPo battery](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/LiPo.jpg) |

## Programming the Digispark board
### Installation of Digispark for the Arduino IDE
Install the Digispark board for the Arduino IDE as described in http://digistump.com/wiki/digispark/tutorials/connecting. I recommend to use as Digispark board URL in Arduino *File/Preferences* the new  **https://raw.githubusercontent.com/ArminJo/DigistumpArduino/master/package_digistump_index.json** instead of **http://digistump.com/package_digistump_index.json** and install the latest **Digistump AVR Boards** version.<br/>
Since we want to save power, the board clock is switched to 1 MHz in our setup() so you can choose **Digispark (1mhz - No USB)**
as board in the *Tools* menu.

## Driver installation
For Windows you must install the **Digispark driver** before you can program the board.<br/>
if you have the *Diigistump AVR Boards* already installed, then the driver is located in `%UserProfile%\AppData\Local\Arduino15\packages\digistump\tools\micronucleus\2.0a4`. Just execute the `Install_Digistump_Drivers.bat` file.<br/>
**Or** download it [here](https://github.com/digistump/DigistumpArduino/releases/download/1.6.7/Digistump.Drivers.zip), open it and run `InstallDrivers.exe`. 

### German instructions
Leider muss der Treiber f�r das Digispark Board manuell installiert werden. Der **Digispark Treiber** kann von [hier](https://github.com/digistump/DigistumpArduino/releases/download/1.6.7/Digistump.Drivers.zip) heruntergeladen werden. Dann die Datei �ffnen und `InstallDrivers.exe` ausf�hren.<br/>
Wenn die Digispark Boards in der Arduino IDE schon installiert sind, ist der Treiber bereits auf der Platte unter `%UserProfile%\AppData\Local\Arduino15\packages\digistump\tools\micronucleus\2.0a4`. Am einfachsten installiert man ihn, wenn man das Board einsteckt und wenn das unbekannte Ger�t im Ger�te-Manager auftaucht, *Treiber aktualisieren* ausw�hlt. Dann *Auf dem Computer nach Treibersoftware suchen* w�hlen, `C:\Users\<username>` w�hlen und *Weiter* klicken.<br/>
Bei der Nachfrage *M�chten sie diese Ger�tesoftware installieren* auf *installieren* klicken.

Wenn das **Board nicht erkannt** wird (kein Ger�usch beim Einstecken) kann es daran liegen, dass die Buchse zu tief ist, dann eine ander Buchse oder ein USB Verl�ngerungskabel benutzen.

### Compile and upload the program to the board
In the Arduino IDE create a new sketch with *File -> New* and name it e.g. `OpenWindowAlarm`.
Copy the code from [OpenWindowAlarm.ino](https://raw.githubusercontent.com/ArminJo/Arduino-OpenWindowAlarm/master/OpenWindowAlarm/OpenWindowAlarm.ino)<br/>
**OR**<br/>
Download and extract the repository. Open the sketch with *File -> Open...* and select the `OpenWindowAlarm` folder.

Compile and upload it. Keep in mind, that upload will not work if the speaker is connected.
If everything works well, the built-in LED of the Digispark will blink 5 times (for the 5 minutes alarm delay) and then start flashing after 8 seconds with an interval of 24 seconds to signal each temperature reading.

## Power reduction
Before power reduction changes
![Final power reduction](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Digispark.jpg)

We now have a Digispark board that [consumes 6 mA at 1MHz and 3,7 volt](https://github.com/ArminJo/micronucleus-firmware#measured-digispark-pll-clock-supply-current). With a battery of **2000 mAh** it will run for **14 days**. But it is possible to reduce power consumption to **27 �A** in 3 Steps.
1. **Disabling the power LED** by breaking the copper wire that connects the power LED to the diode with a knife or removing / disabling the 102 resistor saves 2/2.2 mA.
2. **Removing the VIN voltage regulator** saves 1.5/3.0 mA.<br/>
The board now needs 3/4.3 mA at 3.7/5 volt and the 2000mAh battery will last for 28 days.
3. **Disconnecting the USB D- Pullup resistor** (marked 152) from 5 volt (VCC). Disconnect it by breaking the copper wire on the side of the resistor that points to the ATtiny.<br/>
**This disables the USB interface** and in turn the possibility to program the Digispark board via USB. To **enable it again**, but still save power, **connect the resistor (marked 152) directly to the USB V+** that is easily available at the outer side of the diode.<br/>
The correct side of the diode can be found by using a continuity tester. One side of this diode is connected to pin 8 of the ATtiny (VCC) and Digispark 5V. The other side is connected to the USB V+.

Now the USB pullup resistor is only activated if the Digispark board is connected to USB e.g. during programming.<br/>
The board now consumes **27 �A** during sleep.

The software loop needs 2.1 ms (plus 3 times 64 ms startup time) => active time is around 1/125 of total time.
During the loop the power consumption is 100 times the sleep current => Loop adds **80%** to total power consumption.<br/>
We now have an average current consumption of **75 �A** and the 2000mAh battery will last for **3 years**.

The BOD current of 20 �A can only be disabled by setting fuses via ISP programmer](https://www.google.de/search?q=arduino+as+isp) and a connecting adapter. We can also reduce the start-up time from sleep from 64 to to 5 ms.
For reprogramming the fuses, you can use [this script](https://github.com/ArminJo/micronucleus-firmware/blob/master/utils/Write%2085%20Fuses%20E1%20DF%20FE%20-%20Digispark%20default%20without%20BOD%20and%20Pin5%20and%20fast%20startup.cmd).<br/>
Without BOD and with fast startup we have an average current consumption of **9 �A** and are still able to program the ATtiny by USB.

## Reset button
**If you do not want to remove power to reset the alarm**, connect a reset button between PB5 and ground.
I did this by connecting the unconnected VIN copper surface to PB5 and soldering the reset button directly to the VIN pin hole and the big ground surface of the removed VIN voltage regulator.<br/><br/>
If you want to **get rid of the 5 seconds wait** for USB connection **after reset**, you can [change the micronucleus kernel on the ATtiny85](https://github.com/ArminJo/DigistumpArduino#update-the-bootloader).

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


## Module samples
| | |
|---|---|
| ![OpenWindowAlarm circuit with AAA batteries](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-VersionAAA.jpg)<br/>Powered by 2 AAA batteries | ![OpenWindowAlarm circuit by CR2032](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/CR2032Front.jpg)<br/>Powered by CR2032 coin cell |
| ![OpenWindowAlarm circuit with LiPo battery](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-VersionLiPo.jpg)<br/>Powered by LiPo battery | ![OpenWindowAlarm circuit by CR2032](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/CR2032Back.jpg)<br/>Back viev with CR2032 coin cell |
| ![OpenWindowAlarm circuit with buzzer](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/OpenWindowAlarm2AAA.jpg)<br/>With 16 Ohm buzzer from an old Pc | ![OpenWindowAlarm circuit compact version](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-Version-Compact.jpg)<br/>Compact version |

Different reset buttons and connectors
![OpenWindowAlarm circuit by CR2032](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/4Modules.jpg)

# Placement
Place the board on a windowsill and connect it to the supply.
If the temperature on the sill is lower than the temperature where the board was originally located, it will take additional 5 minutes to adopt to the new start value to avoid false alarm.
![OpenWindowAlarm circuit with LiPo battery](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/OpenWindowAlarmLiPo_2.jpg)

# Internal operation
* An open window is detected after `TEMPERATURE_COMPARE_AMOUNT * TEMPERATURE_SAMPLE_SECONDS` (48) seconds of reading a temperature with a value of `TEMPERATURE_DELTA_THRESHOLD_DEGREE` (2) lower 
    than the temperature `TEMPERATURE_COMPARE_DISTANCE * TEMPERATURE_SAMPLE_SECONDS` (192 seconds-> 3 minutes and 12 seconds) before.
* The delay is implemented by sleeping 3 times at `SLEEP_MODE_PWR_DOWN` for a period of 8 seconds -the maximum hardware sleep time- to reduce power consumption.

* If an **open window is detected**, this is indicated by a longer **20 ms blink** and a **short click** every 24 seconds.
   Therefore, the internal sensor has a time of 3 minutes to adjust to the outer temperature in order to capture even small changes in temperature.
   The greater the temperature change the earlier the sensor value will change and detect an open window.
   
* `OPEN_WINDOW_ALARM_DELAY_MINUTES` (5) minutes after open window detection the **alarm is activated**.<br/>
    The alarm will not start or an activated alarm will stop if the current temperature is greater than the minimum measured temperature (+ 1) i.e. the window has been closed already.
    
* The **initial alarm** lasts for 10 minutes. After this, it is activated for a period of 10 seconds with a increasing break time from 24 seconds up to 5 minutes.

* At **power-on** the VCC voltage is measured used to **determine the type of battery**  using `VCC_VOLTAGE_LIPO_DETECTION` (3.6 volt).

* Every `VCC_MONITORING_DELAY_MIN` (60) minutes the battery voltage is measured. Depending on the detected battery type, **low battery voltage** is indicated by **beeping and flashing the LED every 24 seconds**.
    Only the beep (not the flash) is significantly longer than the beep for an open window detection.<br/>
    Low battery voltage is defined by `VCC_VOLTAGE_LOWER_LIMIT_MILLIVOLT_LIPO` (3550 Millivolt) or `VCC_VOLTAGE_LOWER_LIMIT_MILLIVOLT_STANDARD` (2350 Millivolt).

* After power-on, the **inactive settling time** is 5 minutes. If the board is getting colder during the settling time, 4:15 (or 8:30) minutes are added to avoid false alarms after power-on.

* If you enable `DEBUG` by activating line 62, you can monitor the serial output with 115200 baud at P2 to see what is happening.

# Revision History
### Version 1.3.1
- Check for closed window happens only the first 10 minutes of alarm.
### Version 1.3.0
- Changed voltage low detection.
- Improved DEBUG output.
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
#### If you find this program useful, please give it a star.
