# idp8
`Infrared Data Protocol 8` - `AVR` `ATtiny85` `(avr-asm)`

# Abstract
This repository contains a bare-bones implementation of a `3 bit` `IR` data protocol using the `AVR` architecture, providing the means for basic short range robotics control.

# Firmware Features
1. Variable `38kHz` `IR` transmitter burst time (protocol independent).
2. Pulse delay encoding.
3. Simple data transmission error checking.
4. High noise immunity.
5. Uses internal `8MHz` `RC` oscillator.
6. Timer resolution of `100μs`. 
7. Fully configurable and extensible.

# Warnings
The sample protocol provided is only intended to be used by one `TX/RX` pair.

> Note: The addition of multiple transmitters may cause data corruption or other unknown side effects. <br>

# Info
Included in the main source files is a `LED` demo. <br>
Four `LED's` can be controlled remotely using four tacticle push-buttons.

Schematics for the project are provided.

Folders:
* `/firmware` - `TX` and `RX` `avr-asm` firmware.
* `/schematics` - circuit schematics for `IR` `TX` and `RX`.
* `/spec` - `IDP` protocol specification.
