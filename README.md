# NYFi 64
A wireless N64 controller adapter.

Features an integrated (non-volatile) controller pak (aka memory pak) and rumble pak support.
Switching between the integrated controller pak and a local rumble pak can be done on-the-fly via a button combination.

This project consists of 2 main parts, the controller-part (main TX) and the console-part (main RX).

The controller-part is made using a microcontroller (currently an Arduino Nano, ATmega328P at 16MHz) and an NRF24L01+ transceiver, whereas the console-part is made from an FPGA (currently an UPDuino v1.0, iCE40 UltraPlus), a memory chip (currently an FRAM, FM25W256) and an NRF24L01+ transceiver.


Some words on the controller-side schematics: Never turn on the battery power **and** power the circuit via USB.
The schematic is **very simple** and does not include any protection against short-circuits or similar.
Hence, this could destroy your computer (or wherever the USB power comes from) and the circuit.
Always **only** use one power source.

In general, everything you do with the files from here, you do on **your responsibility and risk**.
The adapter works nicely on my controllers and N64.
Yet, the adapter will draw more power from the N64 than a regular controller, which could potentially risk your console's health.
As always, do not proceed with anything you do not fully understand.
