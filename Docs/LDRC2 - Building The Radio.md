


# Building the LockDownRadioControl

---

This section list the parts and explains how to build the radio.

# 1. Transmitter 

<p align="center">
<img src="2021-09-23-15-34-58.png" width="550"/>
</p>
To build the transmitter, first acquire all the components listed below. 
The printed circuit board can be created from the published Gerber files.
The current transmitter uses version 1b of the PCB.  

---

## Parts list:

All parts can be obtained from Amazon, eBay and/or many other sources.

* Plastic filament for your 3D printer (I prefer PETG).
* Teensy 4.1 Microcontroller (x1)
* 32 gigabyte micro SD card (x1)
* Nextion 5” Enhanced display (x1)
* EBYTE ML01DP5 (Transceiver) (x1)
* Three position transmitter style switches (x4 -> x8)
* 10K potentiometers with knobs (x0 -> x4)
* Tiny DS1307 (Real time clock) (x1)
* INA 219 voltage sensor (the smaller, purple kind) (x1)
* Pololu 2808 power switch (x1)
* M9-Gimbal (M9 Hall effect Gimbal)(x2)
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

The "M9 Hall Effect Gimbal" will fit the 3D-printed case nicely. You could use other gimbals of course, but then you would need to redesign the case, unless the fit happens to be good.

## Case

<p align="center">
<img src="2021-09-23-15-38-35.png" width="450"/>
</p>
The plastic case is usually 3D printed and this can take many hours or even days. So it might be a good idea to start your 3-D printer before doing anything else. The latest .STL files for this are on Github. If no printer is available there are many 3D printing services available on the Internet, such as  http://www.shapeways.com.

**Printed circuit board**
<p align="center">
<img src="2021-09-23-15-16-26.png" width="450"/>
</p>
The Gerber files for this (Transmitter_1b) are on GitHub.

Most components are marked on the board to show where they should go. But three at the top right were not marked. These are simply voltage regulators as can be seen in the picture. Two 7805 5 volt regulators, and above these, an AMS 1117 3.3 volt regulator. The 7805 regulators can be replaced if preferred with **Pololu S9V11F5 boost-buck convertors**. These luckily are pinout compatible with a 7805 and thus can be simply soldered to the PCB without any change or problem - but do be careful to put them the right way around! (This substitution would also permit using a 2S lipo battery in place of the LIFEPO4 one I use - but that would also require a change to voltage sensing code.)

<p align="center">
<img src="2021-09-23-15-44-12.png" width="750"/>
</p>


All capacitors' values are marked on the PCB. They are mostly ceramic SMD 1206 capacitors except for the 47uF capacitors which should be **Tantalum case size B** and a 100uF electrolytic TTH capactor near the Pololu 2808 power switch. The 47uF Tantalum capacitors are not symmetrical so the end with the darker stripe must be at the end marked positive (+). Similarly the 100uF electrolytic TTH capacitor must have its longer lead through the hole marked (+). Also note that the INA219 voltage sensor will be above this capacitor - so if it's too tall, lie it down out of the way. 

<p align="center">
<img src="fdg.png"  width="450"/></p> 

<p align="center">
<img src="cap_flat.png"  width="450"/></p>

To prevent the Nextion screen putting noise onto the 5V power supply (which it would because of its use of PWM for brightness) it gets it's very own 5V regulator, and two extra large electrolytic capacitors across its power in. I suggest one is attached to the back of the screen itself, and another to the back of the PCB where the screen is connected. 


<p align="center">
<img src="2021-10-19-11-03-24.png"  width="450"/></p> 
<p align="center">
<img src="2021-10-19-11-05-09.png"  width="450"/></p> 
The two Schotkky diodes should also be added to the back of the PCB in order to prevent these capacitors from discharging through the voltage regulators 'backwards' when you turn the power off. This might damage the regulators.

<p align="center">
<img src="2021-09-23-15-45-46.png"  width="450"/></p> 


Start by soldering on all the SMD components and the 100uF through hole electrolytic capacitor, which should be lying down unless its a very short one - because the INA219 will be above it.

There are several breakout boards: the Teensy 4.1, the Pololu 2808, and the Tiny DS 1307 RTC. Solder male header pins to all of these, and female headers to the PCB. (The ML01DP5 transceiver already has eight male header pins attached.) This will allow these easily to be removed and replaced should this ever be needed, or just removed for easy access and cleaning. Look at the image to check the corrent placement of male and female header pins.

**Inputs for Channels 1-8**

At the very bottom of the PCB are the places for eight 3-pin servo-style connectors marked `Sticks and knobs`. See image. These are marked `CH1` at one end and `CH8` at the other. These are used to connect to the gimbals or stick units for channels 1-4. Channels 5-8 are for connection to the four potentiometers (knobs). These four pots can be changed to switches if you prefer (and most do). The switches can be two or three position switches. If you use three position switches then you need also to add two 750K resistors as shown in this image:

<p align="center">
<img src="Switch.png"  width="450"/></p> 


The resistors make the switch appear just as a potentiometer would to the firmware,  and give three possible positions instead of only two.
 
You can swap any, none, or all 4 knobs for a switch in this way. 
Note that the row marked `GND` is for GROUND or EARTH. The signal pin is the other end as it would be for servos. The middle row gives 3.3v power. Hence eight ordinary male to male servo extention wires can be used for this.

**Dedicated switch connectors**

Above the Teensy 4.1 location and over to the left is another row of eight similar connectors marked `Switches 1-4`. These are for the four dedicated (or so called 'EDGE') switches. These **must** be three position switches with no resistors added. The centre terminal of each switch will be connected to the `GND` row, so only four are needed (hence the remaining four are left unconnected). 

The outer two terminals of each switch are connected, in adjacent pairs, the the bottom row of this connector, so all eight are needed. Switch 1 is connected to the first pair on the left, then 2 then 3 with switch 4 occupying the extreme right hand pair. The centre row of this connector isn't connected to anything.

When testing the radio later, these switches may well turn out not to be numbered quite as you had expected. Luckily they are very easily moved until the positioning is good. I almost always move and/or rotate one or two.

**Mounting the transceiver**

At the top near the middle is the 8 pin location for the ML01DP5 transceiver. Solder in a 2x4 Dupont female socket there to accept the transceiver. The transceiver, when plugged in, will extend beyond the PCB and will ultimately be secured with a cable tie to the 3D printed plastic tray that holds the PCB using the hole provided. 

This method allows for the transceiver to be easily replaced if that is ever needed. Moreover there are several alternative implentations of the nRF24L01+ that can be easily tried in this way, as they all use the same 8 pins. I've tested many - perhaps all. I recommend the ML01DP5 from EBYTE. It's the best I've found so far - that is also legal. There are some that available that transmit more powefully - but the extra power seems unnecessary and anyway these are not legal in the United Kingdom.

**Assembling the transmitter case**

The first item to go into the case should be the Nextion Screen. 

Before mounting the screen, first modify its connector as shown in these two images:


<p align="center">
<img src="Nextion1.png" width="450"/>


<p align="center">
<img src="Nextion2.png" width="450"/>


This will allow the screen to be more easily connected correctly to the PCB, and also to an FTDI adaptor. The order of the wires must be *exactly* as shown - including the gap between black and red.

If you have not already done so, now download the free Nextion editor from [https://nextion.tech/nextion-editor/ 
](https://nextion.tech/nextion-editor/)

Use this application to load the latest firmware file (currently `Transmitter512.HMI`) and upload it into the Nextion screen itself. This firmware file (latest version) is on GitHub.


Use four small self tapping screws to mount the Nextion screen into the case **with its connector to the left** as in this image: 

<p align="center">
<img src="ScreenMount.png" width="450"/>

Once the screen is in place, the printed cicuit board, bolted to the plastic tray support (using 3M bolts and locknuts) can be mounted above it, and held in place with two more self tapping small screws.

<p align="center">
<img src="TransmitterTray.png" width="450"/>

The four 'Edge' switches need no resistors. Each has three wires, of which the middle one is always GND. So pick wire colours that you'll remember.

<p align="center">
<img src="SwitchWires.png" width="450"/>


Next, mount these four 'Edge' switches using the cable-tidy brackets as shown:

<p align="center">
<img src="CableTidy1.png" width="450"/>

<p align="center">
<img src="CableTidy.png" width="450"/>

<p align="center">
<img src="CableTidy2.png" width="450"/>

Cut their wires to a suitable length and connect these switches to the connector near the Teensy marked 'Switches 1 - 4'. All four GND wires go on the side away from the Teensy, and the signal wires go next to the Teensy. The centre row is not used, so those pins could be removed as I did here.

<p align="center">
<img src="SwitchConnections.png" width="450"/>

The 'ON' button and the three colour LED are now mounted into the case as shown in this image. The LED breakout board is first attached with a cable tie to the little LED mount piece, which is then held in place using 5-minute epoxy resin.

<p align="center">
<img src="LEDSWITCH.png" width="450"/>

To make the internal battery charging lead, modify a couple of 3S balance leads so that they are female at both ends and about 15 cm long. Be sure to get the polarty right! One end is then mounted into the case side with 5-minute epoxy, the other end is eventually connected to the battery's balance lead.

<p align="center">
<img src="BatteryLead.png" width="450"/>



#  
