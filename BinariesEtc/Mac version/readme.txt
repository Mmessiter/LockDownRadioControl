Version 1.9.3 (RX: 1.9.2) 

How to use the transmitter or receiver firmware updater:


(1) Switch on transmitter or receiver.
(2) Load "Teensy.app". 
(3) In Teensy.app, load "rx1.hex", "rx2.hex" or "tx.hex, and enable "Auto" (top right).
(4) Connect the transmitter or receiver to computer with micro USB cable.
(5) Click teensy_reboot. Firmware should then be updated!
(6) Copy revised help files onto SD card replacing many older ones.

Note: If the receiver has two transceivers, use RX2.hex otherwise use RX1.hex.

To update the Nextion screen, download their free editor (Windows only), load the .HMI file into it, and then upload it to the screen using an FTDI adapter over USB.






New in version: 1.9.3:
----------------------
The Bank 4 (Auto) switch now allows Kill Motor in its third position. While motor is off, it's now possible to edit the curves for all modes even while connected without starting the motor. Also there was a bug affecting trims when in stick mode 2. This is here fixed.

A "Safety" switch possibility is also added. When safety is on, the motor cannot be started.


New in version: 1.9.2:
----------------------
Bug fix and performance enhancements.

New in version: 1.9.1:
----------------------
New audio prompts for ModelMatch and binding. Model number now added to name display.
Some small performance enhancements and one display bug fix.


New in version: 1.9.0:
----------------------

This version adds a feature very like ModelMatch, only better. When binding a model to the transmitter, its Teensy's unique MAC address is saved with the model memory at the transmitter. On subsequent connections, and if this feature is turned on (in Setup -> System -> page 3) then the transmitter will automatically switch to that model even if another was loaded at the time. 	



New in version: 1.8.7:
----------------------

The "Reverse" button displayed with the curve (under "Setup/Channels") now reverses the channel in ALL Banks (AKA Flight modes). Previously, it reversed it only in the selected Bank. But this could cause dangerous confusion we found, so I have modified it. It's very unlikely that anyone would ever want a channel reversed in only one Bank. This does remain possible nonetheless, by simply moving points instead of hitting the Reverse button.
