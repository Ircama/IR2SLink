# IR2SLink
Infrared input driver for the Sony S-Link Control-A1 interface, implementing a general purpose infrared interface to remotely drive Sony audio consumer devices connected through the Sony S-Link bus-system.

## Installation
Create a new folder called "IR2SLink" in your Arduino sketchbook folder.
Place IR2SLink.ino in the "IR2SLink" folder.

This software needs the following library: https://github.com/Ircama/Sony_SLink.git

## Description
This code allows receiving IR sequeces from a generic IR remote and drive a consumer
audio Sony device via Sony S-Link Control-A1 input interface; it also includes
trivial monitoring capabilities for IR codes and for the S-Link interface, to test
and dump received sequences, which can be used for code customization.

In standard operation, the code is targeted to the ATtiny85 microcontroller (e.g.
Digispark Digistump ATtiny85). As the ATtiny85 USB interface does not allow
confortable monitoring and debugging features, testing IR codes or dumping the
S-Link interface can be achieved through an ATmega328P micro (e.g., Arduino Nano).
(Also, a separate program allows raw IR code monitorig with the ATtiny85.)

The code uses the IRemote library (http://z3t0.github.io/Arduino-IRremote) to read IR
keys and Sony_SLink to drive the S-Link interface. IRemote should be appropriately
customized by editing IRremote.h and disabling all the IR protocols except the needed
one (keeping just the reception part, not the transmission which is not used by this
program); this is in order to reduce the code size to the minimum so that it can fit
the very limited ATtiny85 space.

The code sets the ATtiny85 to the sleep mode and keeps the micro in quiescent state
for most of the time, in order to save power when the IR receiver does not detect
data (total consumption < 1mA including IR receiver); a timer limits the operation
time of the mocrocontroller just to process infrared sequences (tyical consumption of
about 10 mA @ 3.2 V during IR reception and for MILLIS_TO_SLEEP milliseconds).

The Digispark Digistump ATtiny85 device has to be modified as follows to save power:
- remove the power led;
- to supply power, either the 12.6V internal rectifier of the Sony device (e.g., close
  to the secondary AC transformer) or the 5.6V logic circuit rail can be used; if
  supplying power with 5.6V, remove the linear voltage regulator of the Digispark
  (LM78L05 has a quiescent current of about 2.5 to 3 milliamperes) and substitute it
  with diodes in series (or zener) to reduce the voltage to about 3.2V (this will be
  the most effective way to reduce quiescent current to the minimum);
- enable D- pull-up only when the USB is connected (this can be achieved by moving 
  the pull-up resistor connection to take the positive voltage directly from the 
  USB +5V and not from the Digispark internal +5V, which is after the schottky diode
  separating USB from the internal +5V; this configuration reduces power consumption
  by avoiding current flow throgh the D- zener during normal operation, preserving
  the D- pull-up when USB is connected, which is anyway needed by the USB hw interface);
- use a low power infrared receiver (e.g., TSOP4838 connected to P0);
- connect S-Link to P2
- flash the Digispark bootloader with the latest Micronucleus V2.03 version (which has
  a significantly compact size) and with "#define ENTRYMODE ENTRY_EXT_RESET" in
  micronucleus-master/firmware/configuration/t85_default/bootloaderconfig.h
- Use the Digispark reset pin to enable the firmware upload (setting ENTRYMODE to
  ENTRY_EXT_RESET avoids to hang the Digispark if D- is not pulled-up); a microswitch
  between D5 and ground is appropriate, also pulling up D5 to VCC with a 22kohm resistor.

## How to use
- connect the Digispark Digistump ATtiny85 to the S-Link interface of the Sony device
- provide power supply from the Sony device (or through external unit, or battery)
- the mapping of IR sequences with related S-Link commands has to be preconfigured
  (code customization is needed for this)
- Pressing a key on the IR remote will control the Sony device

## How to customize this program
- edit IRremote.h and configure the needed protocols (e.g, just to read SAMSUNG codes)
- select the related "results.decode_type" (e.g, decode SAMSUNG)
- within the code, define the S-Link commands to be invoked by each IR key

## Notes for this program
 - A standard Arduino or an ATTINY85 are both supported for standard operation
 - The serial COM port can only be used on a standard Arduino and is not compatible
   with ATTINY85
 - Only ATmega328P is supported for code monitoring (check another program for
   ATTINY85)
