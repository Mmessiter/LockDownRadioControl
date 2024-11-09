# Lockdown Radio Control

![Image](./Images/TX_and_RX.jpeg "The radio!")

## Introduction

This is a very full featured 16 channel Radio Control system for models of all kinds. I've been developing this project since Lockdown in May 2020.

I and several friends have flown many models with it for about 4 years so far, without any problem.

It's proven to be as good as, and in some respects better than, popular commercially available radio control systems, as well as the other open source radio control firmware.

It is also **far cheaper**. Though I doubt that a really equivalent commercial system even exists.

I estimate the cost to be only around £120 per transmitter, and around £35 per receiver depending on where you buy the components.

The microcontrollers are **Teensy 4.1** in the transmitter and **Teensy 4.0** in the receiver. These use the ARM Cortex M7 (at 600 MHz) which is probably the fastest MCU used in any radio control system.

The transceivers are Ebyte's **ML01DP5** for the transmitter, and two **ML01SP4**s for the receiver.

The screen is a **Nextion NX8048P050 5" Capacitive touch screen**.

The sticks used are **FrSky M9 Hall Sensor Gimbal.**

The digitial trims are also FrSky (X9D Plus transmitter parts Trim switch).

The cases for the transmitter and receiver are 3D printed in whatever plastic you prefer. I prefer PETG.

This repository contains not only the code for the transmitter, its screen and the receiver; but also the Gerber files for the printed circuit boards, the .STL files for the cases etc. and the text files for the help screens (to go onto the SD card.). So there's nothing to stop anyone from buiding one.

If you would like to make one, I'll be happy to help you. [Email me (Malcolm Messiter mmessiter@gmail.com).](mailto:mmessiter@gmailcom)

Eventually I plan to put building instructions onto this repo. I would have already, but I have been busy writing the code.

## Features

New features are added from time to time, especially when requested by users and other club flyers.

Here is a brief summary of the features supported at the time of writing (August 2024): -

- 16 Channels.
- 4 flight modes, each with its own curve for output.
- 32 mixes (for inputs or outputs).
- 90 model memories; with almost unlimited backup file space.
- 11 PWM servo outputs, all with definable centre points and frequencies.
- 'Dual rates' (... except there are actually _three_ rates.)
- Model memory backup and restore using internal SD card.
- 50 Hz FHSS using 82 separate frequencies on the 2.4 GHz ISM waveband.
- 200 Hz frame rate (for exceptionally low latency.)
- 12 BIT servo resolution.
- Very small data packet size, to reduce FHSS frequency collisions.
- Wireless Buddy Box for training beginners.
- "AMS" (**A**utomatic **M**odel **S**election) loads correct model memory automatically on connection to model.
- **Buddy** transmitter also automatically loads the same model memory as **Master** has loaded.
- Wireless model memory sharing. This allows copying model memories to another transmitter **without** removing any SD cards.
- Telemetry including model's battery volts and GPS location, speed, altitude, heading, distance from home, etc.
- Speaker gives critical voice messages and other audio prompts.
- User defined Channel names, and inputs and outputs,
- 5 point curves for each channel and all four flight modes. Supports straight, smoothed, and expo.
- Failsafe.
- 8 user definable switches. (and four of these can be knobs for continious variation of a channel).
- Digital trims, subtrim, servo reverse, servo speeds, user macros, motor timer, log files, etc., etc...
- Support for SBUS, PWM (with definable frequency and centre position), and PPM.
- Support for third party transmitter models (JR type).
- Context sensitive help screens for all functions.
