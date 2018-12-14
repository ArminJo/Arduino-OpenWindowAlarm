# OpenWindowAlarm
Place this on a windowsill and you will be alarmed if you opened the window too long, which means more than 5 minutes.
It senses the falling temperature, so it works best in winter. It needs only 0.006 milliampere, so one battery or power bank will last the whole winter.
![OpenWindowAlarm on a windowsill](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/OpenWindowAlarm.jpg)
OpenWindowAlarm on a windowsill

# Function
Every 24 seconds a sample is taken of the ATtiny internal temperature sensor which has a resolution of 1 degree.
If temperature is lower than "old" temperature value, then an alarm is issued 5 minutes later, if the condition still holds.
Detection of an open window is indicated by a longer 20ms blink and a short click every 24 seconds.
A low battery (below 3.55 Volt) is indicated by beeping and flashing the LED every 24 seconds. Only the beep (not the flash) is significantly longer than at open window detected.

# How to make your own
The parts you need. On the bottom you see a battery charger and 2 different batteries. One is from a tiny quadrocopter, the other from an old mobile phone. You can get the Li-Ion Battery Charger on eBay for 1€.
![Parts](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/PartsAll.jpg)

## Installation of tools
Install the Digispark board for the Arduino IDE as described in http://digistump.com/wiki/digispark/tutorials/connecting
Since we want to save power, the board clock is switched to 1MHz in our setup() so please choose **Digispark (1mhz - No USB)** 
as board in the *Tools* menu in order to get the right timing.

## Programming the Digispark board
Create a new sketch with *File/New* and name it e.g. OpenWindowAlarm
Copy the code from [OpenWindowAlarm.ino](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/OpenWindowAlarm.ino)
Compile and upload it. Keep in mind, that upload can not work if speaker is connected.
If everything was right, the build in LED of the Digispark will blink 5 times and then do a first flash after 8 seconds and afterwards flashes at a 24 seconds interval.

## Power bank as supply
**Only power banks without a auto switch-off function (which are not allowed in the EU) will work for this circuit.**

## Power reduction
Before power reduction changes
![Final power reduction](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Digispark.jpg)

Now we have a module which uses 6/9.5 mA at 3,7/5 Volt. With a power bank of 2000mAh it will run for 9 days. But it is possible to reduce power consumption to 6-9 µA in 3 Steps.
1. Disabling the power LED by breaking the the copper wire from the power LED to the diode with a knife or removing / disabling the 102 resistor saves 2/2.2 mA.
2. Removing the VIN voltage regulator saves 1.5/3.8 mA.
       Now we have 2.5/3.5 mA at 3.7/5 Volt. Gives 23 days for a 2000mAh power bank.
3. Disconnecting the USB Pullup resistor (marked 152) from 5 Volt (VCC) saves the 2.5/3.5 mA. Disconnect it by breaking the copper wire at the side of the resistor, which shows to the ATTiny. BUT this disables the USB interface and in turn the bootloader too.

   But there is a solution: 
   If you use a battery then just connect the resistor directly to the USB 5 Volt which is easily available at one side of the diode. 
   The right side of the diode can be found by using a continuity tester. One side of this diode is connected to pin 8 of the ATtiny (VCC).
   The other side is connected to USB 5V.
   
After power reduction changes
![Final power reduction](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-Version-Detail.jpg)

   If you want to connect the WindowAlarm to an USB power bank **without auto switch off**, you must just disconnect the resistor, but do it AFTER programming the device.
   Or flush first a new bootloader with [Burn_upgrade_micronucleus-t85_pullup_at_0.cmd](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/Burn_upgrade_micronucleus-t85_pullup_at_0.cmd) and then connect the resistor to P0 (Pin4).
   Now the board consumes 6-9 Microampere which is nothing compared to the 100 Microampere internal loss of the power bank itself. This gives more than 2 years for a 2000mAh power bank **without auto switch off**.
   
After power reduction changes for power bank
![Power reduction for USB power bank](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/OpenWindowAlarmPBDetail.jpg)


## Reset button
If you do not want to remove power to reset the alarm connect a reset button between PB5 and ground. 
I did it by connecting the unconnected VIN copper area to PB5 and solder the reset button on the VIN pin hole and on the big ground area of the removed VIN voltage regulator.

## Loudspeaker
Loudspeaker disassembly part 1
![Loudspeaker disassembly](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Loudspeaker1.jpg)
Loudspeaker disassembly part 2
![Loudspeaker disassembly](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Loudspeaker2.jpg)

OpenWindowAlarm circuit with battery
![OpenWindowAlarm circuit with battery](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-Version.jpg)

# Placement
To use the board place it on a windowsill and connect it with a battery or USB power bank.
If the temperature on the sill is lower than the temperature, the board was located from,
it takes additional 5 Minutes to intelligently adopt to the new start value.

# Internal Operation
* Open window is detected after `TEMPERATURE_COMPARE_AMOUNT * TEMPERATURE_SAMPLE_SECONDS` seconds of reading a temperature which value is `TEMPERATURE_DELTA_THRESHOLD_DEGREE` lower than the temperature `TEMPERATURE_COMPARE_DISTANCE * TEMPERATURE_SAMPLE_SECONDS` seconds before.
* The delay is implemented by 3 times sleeping at `SLEEP_MODE_PWR_DOWN` for a period of 8 seconds in order to reduce power consumption.
* Detection of an open window is indicated by a longer 20ms blink and a short click every 24 seconds.
   The internal sensor has therefore 3 minutes time to adjust to the outer temperature, to get even small changes in temperature.
   The greater the temperature change the earlier the change of sensor value and detection of an open window.
* After open window detection Alarm is activated after `OPEN_WINDOW_MINUTES` if actual temperature is not higher than the minimum temperature (+ 1) i.e. the window is not closed yet.
* Every `VCC_MONITORING_DELAY_MIN` minutes the battery voltage is measured. A low battery (below `VCC_VOLTAGE_LOWER_LIMIT_MILLIVOLT` Millivolt) is indicated by beeping and flashing LED every 24 seconds. Only the beep (not the flash) is significantly longer than at open window detection.
* The alarm lasts 10 minutes, then it sounds 10 seconds with a break of 24 seconds initially up to a 5 minutes break. 


