# IDP8
Infrared Data Protocol 8 - AVR ATtiny85 (assembly)

# Abstract
This repository contains a bare-bones implementation of a 3 bit IR (infrared) data protocol using the AVR architecture, providing the means for basic short range robotics control.

# Features
1. Variable 38kHz IR transmitter burst time (Protocol Independent).
2. Pulse delay encoding.
3. Simple data transmission error checking.
4. High noise immunity.
5. Uses internal 8MHz RC oscillator
6. Timer resolution of 100μs. 
7. Fully configurable and extensible.



# Warnings
The sample protocol provided is only intended to be used by one transmitter-receiver pair. The addition of multiple transmitters may cause data corruption or other unknown side effects. However adding multiple receivers may still be viable.

# Info
Included in the main source files is a led demo. Four leds can be controlled remotely using four tacticle push-buttons.  Schematics for the project are provided.

Folders:
* **/firmware** - TX and RX assembly firmware.
* **/schematics** - Circuit schematics for transmitter and receiver.
* **/spec** - IDP Protocol specification.

**Project Completion Date: 12/03/2015**
