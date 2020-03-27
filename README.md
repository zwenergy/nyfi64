# NYFi 64
A wireless N64 controller adapter.

Features an integrated (non-volatile) controller pak (aka memory pak) and rumble pak support.
Switching between the integrated controller pak and a local rumble pak can be done on-the-fly via a button combination.

This project consists of 2 main parts, the controller-part (main TX) and the console-part (main RX).

The controller-part is made using a microcontroller (currently an Arduino Nano, ATmega328P) and an NRF24L01+ transceiver, whereas the console-part is made from an FPGA (currently an UPDuino v1.0), a memory chip (currently an FRAM, FM25W256) and an NRF24L01+ transceiver.
