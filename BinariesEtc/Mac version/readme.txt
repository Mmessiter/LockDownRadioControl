Version 1.8.7 (TX) 1.8.6 (RX)

How to use the transmitter or receiver firmware updater:


(1) Switch on transmitter or receiver.
(2) Load "Teensy.app". 
(3) In Teensy.app, load "rx1.hex", "rx2.hex" or "tx.hex, and enable "Auto" (top right).
(4) Connect the transmitter or receiver to computer with micro USB cable.
(5) Click teensy_reboot. Firmware should then be updated!
(6) Copy revised help files onto SD card replacing many older ones.

Note: If the receiver has two transceivers, use RX2.hex otherwise use RX1.hex.

To update the Nextion screen, download their free editor (Windows only), load the .HMI file into it, and then upload it to the screen using an FTDI adapter over USB.


New in version: 1.8.7:
----------------------

The "Reverse" button displayed with the curve (under "Setup/Channels") now reverses the channel in ALL Banks (AKA Flight modes). Previously, it reversed it only in the selected Bank. But this could cause dangerous confusion we found, so I have modified it. It's very unlikely that anyone would ever want a channel reversed in only one Bank. This does remain possible nonetheless, by simply moving points instead of hitting the Reverse button.
