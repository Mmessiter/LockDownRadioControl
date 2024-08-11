
# LockDownRadioControl
This is a very full featured 16 channel Radio Control system for models of all kinds. 

I started this project during Lockdown (May 2020) because I've always loved flying R/C model aircraft - and during Lockdown we all had to stay busy at home! 

Back then, I didn't expect it to become useful for flying expensive models. But after flying all my models with it for about 4 years without even one failure, I am delighted to say it's now proven to be at least as good as, and in many respects better than, commercially available systems. 

It is also **much** cheaper than an equivalent commercial system. I estimate the cost to be only around £120 per transmitter, and around £35 per receiver depending on where you buy the components. If you would like to have one, I'll be happy to help. 

Just email me: Malcolm Messiter (mmessiter@gmail.com)

## Features 

The microcontroller is an **ARM Cortex M7 (at 600 Mhz)**. These are on the Teensy 4.1 and 4.0 boards.
The strongly recommended transceivers to use are Ebyte's **ML01DP5** for the transmitter, and two **ML01SP4**s for the receiver. 

Almost any nRF24L01+ could be used, but testing was done using only the recommended Ebyte transceivers.

New features are added from time to time, especially when requested by users. 
Here is a summary of the features this radio offers at the time of writing (August 2024).

1. 16 Channels.
2. Range of about 2 kilometers.
3. Very easy to use touch screen graphic user interface.
4. 32 mixes (mix inputs or outputs).
5. 4 flight modes (or banks).
6. 90 model memories.
7. Model memory backup and restore using internal SD card.
8. 50 Hz FHSS at using 82 separate frequencies on the 2.4 Ghz ISM waveband.
9. 200 Hz frame rate for very low latency.
10. 12 BIT resolution.
11. Wireless Buddy Box for training beginners.
12. "AMS" (**A**utomatic **M**odel **S**election) loads correct model memory automatically on connection to model. 
13. Buddy transmitter automatically loads the same model as Master has loaded.
14. Wireless model memory exchange. This allows sharing model memories without removing SD card.
15. Telemetry including model volts (and GPS, temperature & barometric pressure, when using custom sensor hub)
16. Speaker gives critical voice messages and other audio prompts.
17. User defined Channel names.
18. User mapped input and output channels.
19. 5 point editable curves for every channel and every flight mode. Supports straight, smoothed, and expo.
20. Failsafe.
21. User definalble switches.
22. Digital trims, rates, subtrim, servo reverse, servo speeds, macros, timer, log files, etc.
23. Support for SBUS, PWM (with definable frequency and centre position), and PPM.
24. Support for third party transmitter models (JR type).






