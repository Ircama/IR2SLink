/*
 * IR2SLink
 *
 * Infrared input driver for the Sony S-Link Control-A1 interface,
 * implementing a general purpose infrared interface to remotely drive Sony audio
 * consumer devices connected through the Sony S-Link bus-system.
 * 
 * https://github.com/Ircama/IR2SLink.git
 *
 * This software needs the following library: https://github.com/Ircama/Sony_SLink.git
 *
 * (C) Ircama, 2017, CC-BY-SA 4.0
 * https://creativecommons.org/licenses/by-sa/4.0/
 * 
 * This code allows receiving IR sequeces from a generic IR remote and drive a consumer
 * audio Sony device via Sony S-Link Control-A1 input interface; it also includes
 * trivial monitoring capabilities for IR codes and for the S-Link interface, to test
 * and dump received sequences, which can be used for code customization.
 * 
 * In standard operation, the code is targeted to the ATtiny85 microcontroller (e.g.
 * Digispark Digistump ATtiny85). As the ATtiny85 USB interface does not allow
 * confortable monitoring and debugging features, testing IR codes or dumping the
 * S-Link interface can be achieved through an ATmega328P micro (e.g., Arduino Nano).
 * (Also, a separate program allows raw IR code monitorig with the ATtiny85.)
 * 
 * The code uses the IRemote library (http://z3t0.github.io/Arduino-IRremote) to read IR
 * keys and Sony_SLink to drive the S-Link interface. IRemote should be appropriately
 * customized by editing IRremote.h and disabling all the IR protocols except the needed
 * one (keeping just the reception part, not the transmission which is not used by this
 * program); this is in order to reduce the code size to the minimum so that it can fit
 * the very limited ATtiny85 space.
 * 
 * The code sets the ATtiny85 to the sleep mode and keeps the micro in quiescent state
 * for most of the time, in order to save power when the IR receiver does not detect
 * data (total consumption < 1mA including IR receiver); a timer limits the operation
 * time of the mocrocontroller just to process infrared sequences (tyical consumption of
 * about 10 mA @ 3.2 V during IR reception and for MILLIS_TO_SLEEP milliseconds).
 * 
 * The Digispark Digistump ATtiny85 device has to be modified as follows to save power:
 * - remove the power led;
 * - to supply power, either the 12.6V internal rectifier of the Sony device (e.g., close
 *   to the secondary AC transformer) or the 5.6V logic circuit rail can be used; if
 *   supplying power with 5.6V, remove the linear voltage regulator of the Digispark
 *   (LM78L05 has a quiescent current of about 2.5 to 3 milliamperes) and substitute it
 *   with diodes in series (or zener) to reduce the voltage to about 3.2V (this will be
 *   the most effective way to reduce quiescent current to the minimum);
 * - enable D- pull-up only when the USB is connected (this can be achieved by moving 
 *   the pull-up resistor connection to take the positive voltage directly from the 
 *   USB +5V and not from the Digispark internal +5V, which is after the schottky diode
 *   separating USB from the internal +5V; this configuration reduces power consumption
 *   by avoiding current flow throgh the D- zener during normal operation, preserving
 *   the D- pull-up when USB is connected, which is anyway needed by the USB hw interface);
 * - use a low power infrared receiver (e.g., TSOP4838 connected to P0);
 * - connect S-Link to P2
 * - flash the Digispark bootloader with the latest Micronucleus V2.03 version (which has
 *   a significantly compact size) and with "#define ENTRYMODE ENTRY_EXT_RESET" in
 *   micronucleus-master/firmware/configuration/t85_default/bootloaderconfig.h
 * - Use the Digispark reset pin to enable the firmware upload (setting ENTRYMODE to
 *   ENTRY_EXT_RESET avoids to hang the Digispark if D- is not pulled-up); a microswitch
 *   between D5 and ground is appropriate, also pulling up D5 to VCC with a 22kohm resistor.
 * 
 * How to use:
 * - connect the Digispark Digistump ATtiny85 to the S-Link interface of the Sony device
 * - provide power supply from the Sony device (or through external unit, or battery)
 * - the mapping of IR sequences with related S-Link commands has to be preconfigured
 *   (code customization is needed for this)
 * - Pressing a key on the IR remote will control the Sony device
 *
 * How to customize this program:
 * - edit IRremote.h and configure the needed protocols (e.g, just to read SAMSUNG codes)
 * - select the related "results.decode_type" (e.g, decode SAMSUNG)
 * - within the code, define the S-Link commands to be invoked by each IR key
 * 
 * Notes for this program:
 *  - A standard Arduino or an ATTINY85 are both supported for standard operation
 *  - The serial COM port can only be used on a standard Arduino and is not compatible
 *    with ATTINY85
 *  - Only ATmega328P is supported for code monitoring (check another program for
 *    ATTINY85)
 */

// Configuration

#if defined (__AVR_ATtiny85__) // This section is for Digispark Digistump ATTINY85
#pragma message("ATtiny85 selected")

//#define STARTUP_INDICATOR // switch on the led for two seconds on startup
#define LED_ENABLED // disable to save additional power
#define SLEEP_ENABLED // go to sleep after MILLIS_TO_SLEEP idle time with no IR data received; wake-up on IR data
#define MILLIS_TO_SLEEP 500

#define RECV_PIN 0 // Infrared receiver pin (TSOP4838 or equivalent)
#define LED_PIN 1 // internal LED pin
#define SLINK_PIN 2 // S-Link Control-A1 pin

#else // This section is for Arduino Nano

#define RECV_PIN 3 // Infrared receiver pin (TSOP4838 or equivalent)
#define LED_PIN 13 // internal LED pin
#define SLINK_PIN 2 // S-Link Control-A1 pin
#define SERIAL_COM_PORT_SPEED 115200UL // 115 kbps
#define IR_SUPPLY_PIN 4 // used to power the TSOP4838 infrared receiver via a pin

#endif

// IR Remote library
#include <IRremote.h>
IRrecv irrecv(RECV_PIN);
decode_results results;

// TV Samsung IR codes
#define TV_SAMSUNG_PREDATA    0xE0E0  //(samsung 32 bits)

#define TV_SAMSUNG_KEY_1      0xE0E020DF
#define TV_SAMSUNG_KEY_2      0xE0E0A05F
#define TV_SAMSUNG_KEY_3      0xE0E0609F
#define TV_SAMSUNG_KEY_4      0xE0E010EF
#define TV_SAMSUNG_KEY_5      0xE0E0906F
#define TV_SAMSUNG_KEY_6      0xE0E050AF
#define TV_SAMSUNG_KEY_7      0xE0E030CF
#define TV_SAMSUNG_KEY_8      0xE0E0B04F
#define TV_SAMSUNG_KEY_9      0xE0E0708F
#define TV_SAMSUNG_KEY_0      0xE0E08877
#define TV_SAMSUNG_KEY_DECADE 0xE0E0C43B
#define TV_SAMSUNG_PRE_CH     0xE0E0C837

#define TV_SAMSUNG_POWER      0xE0E040BF
#define TV_SAMSUNG_VOL_UP     0xE0E0E01F
#define TV_SAMSUNG_VOL_DOWN   0xE0E0D02F
#define TV_SAMSUNG_CH_UP      0xE0E048B7
#define TV_SAMSUNG_CH_DOWN    0xE0E008F7
#define TV_SAMSUNG_MUTE       0xE0E0F00F
#define TV_SAMSUNG_SOURCE     0xE0E0807F
#define TV_SAMSUNG_TXT        0xE0E034CB
#define TV_SAMSUNG_CHLIST     0xE0E0D629
#define TV_SAMSUNG_SMARTHUB   0xE0E09E61
#define TV_SAMSUNG_INFO       0xE0E0F807

#define TV_SAMSUNG_UP         0xE0E006F9
#define TV_SAMSUNG_RIGHT      0xE0E046B9
#define TV_SAMSUNG_LEFT       0xE0E0A659
#define TV_SAMSUNG_DOWN       0xE0E08679
#define TV_SAMSUNG_OK         0xE0E016E9

#define TV_SAMSUNG_FAMILY     0xE0E0639C // FamilyStory
#define TV_SAMSUNG_SUPPORT    0xE0E0FC03
#define TV_SAMSUNG_A          0xE0E036C9 // Red
#define TV_SAMSUNG_B          0xE0E028D7 // Green
#define TV_SAMSUNG_C          0xE0E0A857 // Yellow
#define TV_SAMSUNG_D          0xE0E06897 // Blue

#define TV_SAMSUNG_PLAY_REW   0xE0E0A25D
#define TV_SAMSUNG_PLAY_FF    0xE0E012ED
#define TV_SAMSUNG_PLAY_STOP  0xE0E0629D
#define TV_SAMSUNG_PLAY_PAUSE 0xE0E052AD
#define TV_SAMSUNG_PLAY_GO    0xE0E0E21D

#define TV_SAMSUNG_EXIT       0xE0E0B44B
#define TV_SAMSUNG_RETURN     0xE0E01AE5

#define TV_SAMSUNG_MENU       0xE0E058A7
#define TV_SAMSUNG_GUIDE      0xE0E0F20D
#define TV_SAMSUNG_TOOLS      0xE0E0D22D

// Global variables
unsigned long timeLastCmd = 0, lastCommand = 0, pressureKey = 0;
boolean commandEnabled = true; // true means that the IR input keys are active by default
                               // and generate S-Link commands. It can be set to false to
                               // invert the TV_SAMSUNG_FAMILY key behaviour
boolean muteEnabled = true;
int bank = 1;

// Sleep mode
#ifdef SLEEP_ENABLED
#include <avr/sleep.h>
#include <avr/interrupt.h>

unsigned long previousTime, elapsedTime;

void sleep() {
#if defined (__AVR_ATtiny85__)
  cli();                                  // Disable interrupts
  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT0);                   // Use infrared receiver interrupt pin
  //PCMSK |= _BV(PCINT2);                   // Use PB2 as interrupt pin
 
  ADCSRA &= ~_BV(ADEN);                   // ADC off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // set sleep to the lowest power mode

  sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                  // Enable interrupts
  sleep_cpu();                            // sleep

  cli();                                  // Disable interrupts
  previousTime = millis();                // reset sleep timer
  PCMSK &= ~_BV(PCINT0);                  // Turn off infrared receiver interrupt pin
  GIMSK &= ~_BV(PCIE);                    // Turn off Pin Change Interrupts
  //PCMSK &= ~_BV(PCINT2);                  // Turn off PB2 as interrupt pin

  sleep_disable();                        // Clear SE bit
  ADCSRA &= ~_BV(ADEN);                   // ADC off

  sei();                                  // Enable interrupts
#endif
} // sleep

#if defined (__AVR_ATtiny85__)
ISR(PCINT0_vect) {
// This is called when the interrupt occurs
  TIMER_RESET;              // reset irrecv timer
  irrecv.resume();          // reset irrecv counter
  previousTime = millis();  // reset sleep timer
}
/*
ISR(PCINT2_vect) {
// This is called when the interrupt occurs
  previousTime = millis(); // reset sleep timer
}
*/
#endif
#endif

// SONY S-LINK/Control-A1 Protocol Library
#include "Sony_SLink.h"
Slink slink;

void setup() {
#ifdef STARTUP_INDICATOR
digitalWrite(LED_PIN, HIGH);
delay(2000); // switch on the LED for two seconds at boot, to indicate that a reset occurs
digitalWrite(LED_PIN, LOW);
#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  Serial.begin(SERIAL_COM_PORT_SPEED);
#endif
  irrecv.enableIRIn(); // Start the IR receiver
  slink.init(SLINK_PIN); // Set-up S-Link pin
#ifdef SLEEP_ENABLED
  previousTime = millis(); // reset sleep timer
#endif
#if defined (__AVR_ATtiny85__)
  ADCSRA &= ~_BV(ADEN);                   // ADC off (save power)
#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  //Code in here will only be compiled if an Arduino Uno/Nano is used.
#ifdef IR_SUPPLY_PIN
  pinMode(IR_SUPPLY_PIN, OUTPUT);
  digitalWrite(IR_SUPPLY_PIN, HIGH);
#endif
Serial.println("Enter command:");
#endif
}

void sendSlinkCommand(unsigned int deviceId = 0x00, unsigned int commandId1 = 0x00, int commandId2 = -1, int commandId3 = -1) {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  Serial.print("sendSlinkCommand: ");
  Serial.print(deviceId, HEX);
  Serial.print(", ");
  Serial.print(commandId1, HEX);
  if (commandId2 != -1) {
    Serial.print(", ");
    Serial.print(commandId2, HEX);
  }
  if (commandId3 != -1) {
  Serial.print(", ");
  Serial.print(commandId3, HEX);
  }
  Serial.println();
#endif
#ifdef LED_ENABLED
  digitalWrite(LED_PIN, HIGH);
#endif
  TIMER_DISABLE_INTR; // stop IRrecv timer to avoid timing conflicts with the delayMicroseconds instructions included in slink.sendCommand
  slink.sendCommand(deviceId, commandId1, commandId2, commandId3);
  TIMER_ENABLE_INTR; // resume IRrecv timer 
#ifdef LED_ENABLED
  digitalWrite(LED_PIN, LOW);
#endif
}

int expiredCmd(unsigned long cmd) {
  return( (lastCommand != cmd) || (micros()-timeLastCmd > 400000UL) );
}

void loop() {
  unsigned long Start;
  byte cmd;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  if (Serial.available()) {
    cmd = Serial.read();
    switch (cmd) {
      case 'S': // enter standard operation mode for 60 seconds, with no serial dump (do not write hex results)
      case 's': // standard operation with monitoring messages (write hex results). Map IR received commands with S-Link output commands
        Start = micros();
        if (cmd != 'S')
          Serial.println("Start receiving");
        do {
#endif
          if (irrecv.decode(&results)) {
#ifdef SLEEP_ENABLED
              previousTime = millis(); // reset sleep timer
#endif
              if ( (results.decode_type == SAMSUNG) && (results.value == TV_SAMSUNG_FAMILY) ) {
                if (expiredCmd(TV_SAMSUNG_FAMILY)) {
                  commandEnabled = !commandEnabled;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
                  Serial.print("commandEnabled: ");
                  Serial.println(commandEnabled);
#endif
                }
                timeLastCmd = micros();
                lastCommand=results.value;
              }
              else
              if ( (results.decode_type == SAMSUNG) && (commandEnabled) ) {
                switch (results.value) {
                  case TV_SAMSUNG_UP:
                      if (expiredCmd(results.value))
                        pressureKey=1;
                      else
                        pressureKey = pressureKey>4? pressureKey++ : pressureKey;
                      for (int loop = 0; loop < pressureKey; loop ++)
                        sendSlinkCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_VOLUME_UP);
                      break;
                  case TV_SAMSUNG_DOWN:
                      if (expiredCmd(results.value))
                        pressureKey=1;
                      else
                        pressureKey = pressureKey>4? pressureKey++ : pressureKey;
                      for (int loop = 0; loop < pressureKey; loop ++)
                        sendSlinkCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_VOLUME_DOWN);
                      break;
                  case TV_SAMSUNG_LEFT:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_POWER_OFF);
                      break;
                  case TV_SAMSUNG_POWER:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_POWER_OFF);
                      break;
                  case TV_SAMSUNG_RIGHT:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_POWER_ON);
                      break;
                  case TV_SAMSUNG_A:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_SET_INPUT_CHAN, SLINK_CMDP_IN_TV);
                      break;
                  case TV_SAMSUNG_B:
                      if (expiredCmd(results.value)) {
                        muteEnabled = !muteEnabled;
                        if (muteEnabled)
                          sendSlinkCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_MUTE_ON);
                        else
                          sendSlinkCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_MUTE_OFF);
                      }
                      break;
                  case TV_SAMSUNG_C:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_SET_INPUT_CHAN, SLINK_CMDP_IN_TUNER);
                      break;
                  case TV_SAMSUNG_D:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_SET_INPUT_CHAN, SLINK_CMDP_IN_CD);
                      break;
                  case TV_SAMSUNG_PRE_CH:
                      if (expiredCmd(results.value)) {
                        switch (bank) {
                          case 1: bank=2; break;
                          case 2: bank=3; break;
                          case 3: bank=1; break;
                        }
                      }
                      break;
                  case TV_SAMSUNG_PLAY_FF:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_UP);
                      break;
                  case TV_SAMSUNG_PLAY_REW:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_DOWN);
                      break;
                  case TV_SAMSUNG_KEY_1:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, bank, 1);
                      break;
                  case TV_SAMSUNG_KEY_2:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, bank, 2);
                      break;
                  case TV_SAMSUNG_KEY_3:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, bank, 3);
                      break;
                  case TV_SAMSUNG_KEY_4:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, bank, 4);
                      break;
                  case TV_SAMSUNG_KEY_5:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, bank, 5);
                      break;
                  case TV_SAMSUNG_KEY_6:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, bank, 6);
                      break;
                  case TV_SAMSUNG_KEY_7:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, bank, 7);
                      break;
                  case TV_SAMSUNG_KEY_8:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, bank, 8);
                      break;
                  case TV_SAMSUNG_KEY_9:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, bank, 9);
                      break;
                  case TV_SAMSUNG_KEY_0:
                      if (expiredCmd(results.value))
                        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, bank, 10);
                      break;
                }
                timeLastCmd = micros();
                lastCommand=results.value;
              }
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
          if (cmd != 'S')
            Serial.println(results.value, HEX);
#endif
          irrecv.resume(); // Receive the next value
          }
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
        }
        while (micros()-Start < 60000000UL);
        if (cmd != 'S') {
          Serial.println("");
          Serial.println("End receiving");
        }
        break;
      case 'r': // dump received IR codes for 10 seconds
        Start = micros();
        Serial.println("Start monitor");
        do {
          if (irrecv.decode(&results)) {
            if (results.decode_type == SAMSUNG) {
                Serial.print("Decoded SAMSUNG: ");
              }
            Serial.println(results.value, HEX);
            irrecv.resume(); // Receive the next value
          }
        }
        while (micros()-Start < 10000000UL);
        Serial.println("");
        Serial.println("End monitor");
        break;
      // test specific S-Link commands
      case 'M':
        slink.sendCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_MUTE_OFF);
        break;
      case 'm':
        slink.sendCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_MUTE_ON);
        break;
      case '1':
        slink.sendCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_POWER_ON);
        break;
      case '0':
        slink.sendCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_POWER_OFF);
        break;
      case '+':
        slink.sendCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_VOLUME_UP);
        break;
      case '-':
        slink.sendCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_VOLUME_DOWN);
        break;
      case 't':
        slink.sendCommand(SLINK_DEVICE_AMP, SLINK_CMD_AMP_SET_INPUT_CHAN, SLINK_CMDP_IN_TV);
        break;
      case 'p':
        sendSlinkCommand(SLINK_DEVICE_TUNER, SLINK_CMD_TUNER_PRESET_STATION, 01, 01);
        break;
      // Dump Control-A1 S-Link sequences
      // IRrecv timer has to be stopped and restarted to avoid timing conflicts with the delayMicroseconds instructions included in slink.inputMonitor
      case 'q':
        TIMER_DISABLE_INTR;
        slink.inputMonitor(0); // measure timing of mark periods in microseconds (sync should be 2400, one about 1200, zero ab. 600)
        TIMER_ENABLE_INTR;
        break;
      case 'z':
        TIMER_DISABLE_INTR;
        slink.inputMonitor(1); // monitor bytes displaying binary and HEX format of each byte
        TIMER_ENABLE_INTR;
        break;
      case 'Z':
        TIMER_DISABLE_INTR;
        slink.inputMonitor(2); // monitor bytes displaying HEX dump
        TIMER_ENABLE_INTR;
        break;
      case 'Q':
        TIMER_DISABLE_INTR;
        slink.inputMonitor(0, true); // measure timing of idle periods (e.g., delimiter; all idle periods should be about 600 microseconds)
        TIMER_ENABLE_INTR;
        break;
      default:
        break;
    }
  }
#endif
#ifdef SLEEP_ENABLED
  elapsedTime =  millis() - previousTime;
  if (elapsedTime > MILLIS_TO_SLEEP) {
    sleep();
  }
#endif
}

