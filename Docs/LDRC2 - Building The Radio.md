


# Building the LockDownRadioControl

This section will explain how to build a system as well as list needed parts.

# 1. Transmitter 

To build the transmitter, first acquire all the components (see list below). 
The printed circuit board can be created from the published Gerber files.
The current transmitter uses version 1b of the PCB  

## Parts list:

All parts can be obtained from Amazon, eBay and/or many other sources.

* Plastic filament for your 3D printer (PETG is recommended).
* Teensy 4.1 Microcontroller (x1)
* 32 gigabyte micro SD card (x1)
* Nextion 5” Enhanced display (x1)
* EBYTE ML01DP5 (Transceiver) (x1)
* Three position transmitter style switches (x4 -> x8)
* 10K potentiometers with knobs (x0 -> x4)
* Tiny DS1307 (Real time clock) (x1)
* INA 219 voltage sensor (the smaller, purple kind) (x1)
* Pololu 2808 power switch (x1)
* M9-Gimbal M9 Hall effect Gimbal (x2)
* 7805 voltage regulators (x2)
* AMS 1117 3.3V voltage regulator (x1)
* 100nF 1206 ceramic capacitors (x3)
* 100uF Electrolytic capacitor (through hole) (x1)
* 10uF 1206 ceramic capacitors (x1)
* 330nF 1206 ceramic capacitors (x2)
* 47uF Tantalum capacitors case B (x2)
* Single row Dupont headers male (Several)
* Single row Dupont headers female (Several)
* Triple row Dupont headers male (Several)
* Gebildet Momentary Push Button (16mm) (x1)
* KY-016 RGB 3 Color Full Color LED Module   (x1)
* Various small screws and nuts
* Micro USB Male to Micro USB Female Extension Panel Mount type cable (x1)
* 2S LIFE Battery (x1)
* 2S balance lead extension leads 
* Schotkky diodes 3A 40V (x2)
* 220uF TTH electrolytic capactors (across the Nextion screen's power supply) (x2)
* Various servo extension cables
* Solder

[](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1067.jpeg)

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1071.jpeg)
**Case**

The plastic case is usually 3D printed and this can take many hours or even days, so it might be a good idea to start your printer before doing anything else. The latest .STL files needed are on Github. If no printer is available there are many 3D printing services available on the Internet, such as www.shapeways.com.

**Printed circuit board**

The Gerber files for this (Transmitter 1b) are on GitHub,

Most components are pretty clearly marked on the board to show where they should go. But three at the top right are not marked. These are voltage regulators as can be seen in the picture. Two 7805 5 volt regulators, and above these, an AMS 1117 3.3 volt regulator. The 7805 regulators can be replaced if preferred with **Pololu S9V11F5 boost-buck convertors**. These luckily are pinout compatible with a 7805 and thus can be simply soldered to the PCB without any change or problem - but do be careful to put them the right way around! This substitution would also permit using a 2S lipo battery in place of the LIFEPO4 one I use - but that would also require a change to voltage sensing code.

All capacitors' values are marked on the PCB. They are mostly ceramic SMD 1206 capacitors except for the 47uF capacitors which should be **Tantalum case size B** and a 100uF electrolytic TTH capactor near the Pololu 2808 power switch. The 47uF Tantalum capacitors are not symmetrical so the end with the darker stripe must be at the end marked positive (+). Similarly the 100uF electrolytic TTH capacitor must have its longer lead through the hole marked (+). Also note that the INA219 voltage sensor will be above this capacitor - so if it's too tall, lie it down out of the way. 

To prevent the Nextion screen putting noise onto the 5V power supply (which it would because of its use of PWM for brightness) it gets it's very own 5V regulator, and two extra large electrolytic capacitors across its power in. I suggest one is attached to the back of the screen itself, and another to the back of the PCB where the screen is connected. 

The two Schotkky diodes also can go on the back of the PCB in order to prevent these capacitors from discharging through the voltage regulators 'backwards' when you turn the power off. This might damage the regulators.

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1070.jpeg)

Start by soldering on all the SMD components and the 100uF through hole electrolytic capacitor, which should be lying down unless its a very short one - because the INA219 will be above it.

There are several breakout boards: the Teensy 4.1, the Pololu 2808, and so on. Solder male header pins to all of these, and female headers to the PCB. This will allow them easily to be removed and replaced should this ever be needed, or just removed for access and cleaning.

To be continued....


#  