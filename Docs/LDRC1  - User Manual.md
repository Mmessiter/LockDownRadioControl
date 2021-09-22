# LockDownRadioControl - User Manual
by Malcolm Messiter

![LDRC Image](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/LDRC%201.png)

## Introduction

During 2020 I decided to try to design and build a radio control system from scratch for models of all types. The result has exceeded my most optimistic expectations so I think it should now be shared with all others who might enjoy making and using it for all sorts of reasons. Development will continue and upgrades will be shared with all. But it is already at a stage where it flies my models well - even the expensive helicopters… That's probably enough introduction!


### Features already supported:

* 2.5 km range
* 16 channels
* 99 model memories
* 12 BIT servo resolution (11 BIT via SBUS)
* 32 mixes
* Four flight modes
* Transmitter modes 1,2,3, or 4
* Telemetry for receiver battery voltage, altitude, roll, pitch, yaw, etc.
* Definable Failsafe on any or all channels
* FHSS on 2.4 Ghz with proprietary channel avoidance system for busy channels
* 5 point user defined curves for every flight mode and every channel
* Exponential, smoothed, or straight line interpolations.
* SBUS (Futaba style) and PWM servo outputs
* Four user definable three-position switches
* Four knobs (pots) or optionally more two-position or three-position switches
* Inputs can be mapped from any stick, knob or switch to any channel
* All channels can be renamed
* 2.4 Ghz waveband scanner
* Wireless model memory copying between transmitters
* Binding using unique 48 BIT MAC address of the transmitter’s microcontroller (a Teensy 4.1)
* GUI for setup on a 5” colour touch screen with many help screens
* 32 Gigabyte on-board SD card for model memories, log files, help screens, etc.
* Three colour LED showing current status
* Real-time clock keeps time even when system is off
* Motor timer - runs while motor is running

(More features will be added here …)

When you switch on the transmitter, this is the first screen you will see:

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1039.jpeg)

(Some details will of course be different!)
The very first thing you must do is calibrate the sticks, knobs, and switches. 
To do this press the **Setup** button to reveal the setup screen:

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1040.jpeg)

Then press the **Calibrate** button to see the Calibration screen :

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1058.jpeg)

Press the **Calibrate sticks etc** button, then wiggle both sticks to the extremes of their travel in every direction several times. Also move all the switches to their extremes, and any knobs. Then press the button again. Then make sure every control is **precisely centred** and press the button a third time. Calibration is complete! Hit the **OK** button to exit.

(The calibration process is best done with no defined model memory loaded. So beforehand, select an unused model memory, delete it, and use this deleted one for calibration.)

## Binding

To connect to a model, the receiver must be bound to the transmitter. Switch on the transmitter first, then the receiver. If binding has already been done, the connection will complete automatically after a moment. If not, a prompt will appear on the transmitter’s front screen: **>>>BIND MODEL?>>>**.  Press the **Setup** button, then select the **Bind** option, then **Bind now** … to complete binding. Once bound in this way the receiver will not connect with another transmitter until it has been turned off and back on again.Probably the next thing you need to do is to define a model. Select the **Model**s screen by pressing **Models** on the Setup screen. I recommend setting up for a flight simulator on a computer first in order to become familiar with all the options before you set up an actual model.

The Models screen looks like this:

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1046.jpeg)

All you need to do here now is pick a number on the left by pressing plus or minus, then click the name below the number and edit it to the name you wish to appear there. 

Then go to the **Channels** screen:

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1043.jpeg)

Here you can observe the effect of moving sticks etc. and if necessary you can redefine the names of the channels using the **Inputs** screen:

From the **Channels** screen you can select to edit the curve for that channel. Press the button on the left or on the right relating to the channel in question in order to see the curve editing screen:

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1064%202.jpeg)

Here you can experiment with different values for the five points on the curve. When you hit **OK**, your settings are saved for the selected flight mode, only. To use this curve for all flight modes, check the box marked **Copy to all flight modes:** before hitting **OK.**

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1045.jpeg)

When you have set up all channels, it’s usually a good idea to return to the models screen in order to save this model memory to a file. On the **Models** screen, click on the green file name on the right and edit it to your chosen file name. (The name must have eight characters or fewer, a dot, then **.MOD**). Press **->Export** to save the file. If that file existed, it’s overwritten with the new data.


Once a model file has been saved in this way, it can be imported to any model memory later, and it can be transmitter to another transmitter. The files are stored on the SD card in the Teensy 4.1.

The **Delete** button on the left of the Models screen will delete that model memory (which essentially just reverts it to all default values - useful for calibration). The **Delete** button on the right the Models screen will delete the file which is currently displayed in green in the black box. The plus and minus buttons on the right hand side of the screen will scroll through all available saved files in alphabetical order. The **List** button below will display all model files.

The **Send** and **Receive** buttons are useful when you have two or more transmitters, or perhaps when a friend has one. These are used to send a model memory wirelessly to another transmitter. To send a model file, first select the right model file at the send end. At the receiving end, select a model memory which you do not mind over writing. Press the **Receive** button at the receiving end. Then within 30 seconds, press the **Send** button at the Send end. That will copy the model memory to the other transmitter. Wirelessly. It takes only, a few seconds.

After copying a model memory to another transmitter, check all functionality before the model leaves the ground! In particular check the sense of the switches because these may have been mounted the other way round in the other transmitter.

## Failsafe
Immediately after setting up a model, it is recommended to set up Failsafe – especially for the motor. Go to the **Failsafe** screen:

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1053.jpeg)

Usually all you need to do is reduce the throttle to minimum, centre all of the other controls, check every channel that you are using, and press the **Save** button. This must be done while the model is connected since the data are saved at the receiver end.

You may notice that during a connection you cannot turn off the transmitter using its hardware button or the front screen option. This is for safety. So in order to test failsafe you must go to the options screen and select **Switch off now**… (…even if still connected.)

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1054.jpeg)

While on the **Options** screen, note that to save battery the screen turns itself off after a definable period of no use. To turn it back on again, just touch the screen. The transmitter will switch itself right off after a defined inactivity period. The transmitter will never switch itself off if it’s still connected. Inactivity means no stick movements and no screen touches.

The FHSS (frequency hopping spread spectrum) can be setup to use a defined range of channels. Add 2400 to the channel number to derive the frequency. In the United Kingdom it’s legal to use 1 - 83 (= 2.401 - 2.483 Ghz). The hardware (an nRF24L01) supports 2.401 - 2.525. Don’t use frequencies that are not legal in your country.

While the transmitter is booting up, it quietly scans the entire waveband for about a second, and records any frequencies that appear to be busy. It uses this information later to avoid hopping to those frequencies.

## Switches 

It’s a good idea as part of model setup to define the use of the primary switches. Every transmitter has four primary switches on the top: two on the extreme left and two on the extreme right. Depending on how it was made, there may be four more (secondary) switches or four more knobs, or some combination of these possibilities. These just operate proportional channels by default.

Setting up a **Flight modes** switch and an **Auto** switch is an important part of a model’s set up.

For this radio, **Auto** means exactly the same as **Flight mode 4**. When Auto or Flight mode 4 is selected, the system assumes that the **motor is not running** and so it pauses the timer on the front screen. This is useful for knowing how long the motor has been running and hence perhaps the likely state of its battery - if you didn’t setup voltage telemetry yet.

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1063.jpeg)

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1065.jpeg)

So whatever model you have, always setup Flight mode 4 so the motor is stopped when it’s selected.

The front screen always displays the current flight mode. So use the front screen to check that the switches are operating as planned. Functions can then be reversed or moved until it looks good!


## Mixes 

For gliders and aeroplanes with multiple flaps, V tails, deltas, elevons, ailerons etc., mixes are certainly needed. The mixes screen looks like this:

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1051.jpeg)

Often it is desirable that a mix applies to equally in every flight mode. In this case select 0 (zero) as the flight mode. Otherwise the mix will only be applied for the selected flight mode. 

If you have given names to the channels, then these names will appear on the right hand side as you enter the master’s and slave’s channel number. This helps to clarify what you’re setting up.

Be sure to check carefully the effects of your mixes before leaving the ground!

## Features still in development 

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1060.jpeg)

The Gains screen (below) is still a work in progress and not yet ready for use. It will be for multi-rotors and helicopters. Support for these is planned to come soon. For now please ignore it. 

## Scanning 

Sometimes it’s interesting to know the extent of possible interference on the 2.4 Ghz band. That’s why I included this scanner (below) which here shows rather a lot of traffic - probably WiFi because this was done at home. Things are usually much quieter at the flying field.

![](https://github.com/Mmessiter/LockDownRadioControl/blob/main/Pictures/IMG_1061.jpeg)

## Power and data rate

The nRF24L01 transceiver chip offers various data rates and power levels. I’ve included the option to select from all these for range tests and general research. Please always select the highest power and lowest data rate for all flying as these give best range.

## Trims

Traditional trims are on screen. These are saved per flight mode and per model. I do not recommend adjusting these while flying because you might need to look away from the model. Instead it’s better to land carefully and then adjust them while on the ground. The R check box reverses the sense of that trim. 

“Sub trim” and “end points” and “servo reverse’’ are handled by simply editing the relevant curve. 

## Helicopters, multirotors and simulators 

With the current version of this radio, the SBUS output from the receiver can connect to most flybarless systems such as a VBar Neo and to most drone flight controllers. This is the best option for now.

The SBUS output is also perfect for connection to simulators via a RX2SIM or similar device.

## Disclaimer 

So far this radio has flown my models for many months without even one failure that resulted in a crash. This has been a nice surprise! I hope this good luck continues for me and for others. I certainly try my best to ensure maximum reliability. 

But perfection is both rare and unlikely - so eventually of course there will be crashes, and some may be because of my code or design. **So if you fly with this system it must be with this knowledge and entirely at your own risk.** Please don’t send me the bill for your crashed model! 
