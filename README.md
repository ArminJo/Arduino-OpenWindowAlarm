# OpenWindowAlarm
Place this on a windowsill and you will be alarmed if you opened the window too long, which means more than 5 minutes.
It senses the falling temperature, so it works best in winter.
![OpenWindowAlarm on a windowsill](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/OpenWindowAlarm.jpg)
OpenWindowAlarm on a windowsill

# Internal Function
Every 24 seconds a sample is taken of the ATtiny internal temperature sensor which has a resolution of 1 degree.
If temperature is lower than "old" temperature value, then an alarm is issued 5 minutes later, if "the condition still holds".
This delay is implemented by 3 times sleeping at SLEEP_MODE_PWR_DOWN for a period of 8 seconds in order to reduce power consumption.

# How to make your own
![Parts](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Parts.jpg)
The parts you need

## Installation of tools
Install the Digispark board for the Arduino IDE as described in http://digistump.com/wiki/digispark/tutorials/connecting
Since we want to save power, the board clock is switched to 1MHz in our setup() so please choose **Digispark (1mhz - No USB)** 
as board in the *Tools* menu in order to get the right timing.

## Programming the Digispark board
Create a new sketch with *File/New* and name it e.g. OpenWindowAlarm
Copy the code from [OpenWindowAlarm.ino](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/OpenWindowAlarm.ino)
Compile and upload it. Keep in mind, that upload can not work if speaker is connected.
If everything was right, the build in LED of the Digispark will blink 5 times and then do a first flash after 8 seconds and afterwards flashes at a 24 seconds interval.

## Power reduction
![Final power reduction](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Digispark.jpg)
Before power reduction changes

Now we have a module which uses 6 mA. It is possible to reduce power consumption to 6 µA in 3 Steps.
1. Disabling the power LED by removing / disabling the 102 resistor or the LED saves 2 mA.
2. Removing the VIN Voltage regulator saves 1.5 mA.
3. Disconnecting the USB Pullup 152 resistor from 5 Volt (VCC) saves 2mA, but disables the USB interface and in turn the bootloader too.

   But there is a solution: connect the resistor directly to the USB 5 Volt which is easily available at one side of the diode. 
   The right side can be found by using a continuity tester. One side of this diode is connected to pin 8 of the ATtiny (VCC).
   The other side is connected to USB 5V.
![Final power reduction](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Final-Version-Detail.jpg)
After power reduction changes
![OpenWindowAlarm circuit with battery](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/OpenWindowAlarm1.jpg)
OpenWindowAlarm circuit with battery

## Assembly
![Loudspeaker disassembly](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Loudspeaker1.jpg)
Loudspeaker disassembly part 1
![Loudspeaker disassembly](https://github.com/ArminJo/Arduino-OpenWindowAlarm/blob/master/pictures/Loudspeaker2.jpg)
Loudspeaker disassembly part 2

# Placement
To use the board place it on a windowsill and connect it with a battery or USB power bank.
If the temperature on the sill is lower than the temperature, the board was located from,
it takes additional 5 Minutes to intelligently adopt to the new reference value.

