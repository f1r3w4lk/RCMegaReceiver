/******************************************************************************/
/**
 * \file
 * \brief Implementation of Radio Control Receiver to Arduino Mega only
 *        All code here is adapted from Multiwii 2.4 - RX.cpp file
 */
/******************************************************************************/

#if defined(__AVR_ATmega2560__)

#include <Arduino.h>
#include "RCMegaReceiver.h"

#define AVERAGING_ARRAY_LENGTH 4
#define RC_CHANS 8

//RX PIN assignment inside the port //for PORTK
#define THROTTLEPIN                0  //PIN 62 =  PIN A8
#define ROLLPIN                    1  //PIN 63 =  PIN A9
#define PITCHPIN                   2  //PIN 64 =  PIN A10
#define YAWPIN                     3  //PIN 65 =  PIN A11
#define AUX1PIN                    4  //PIN 66 =  PIN A12
#define AUX2PIN                    5  //PIN 67 =  PIN A13
#define AUX3PIN                    6  //PIN 68 =  PIN A14
#define AUX4PIN                    7  //PIN 69 =  PIN A15

#define PCINT_PIN_COUNT            8
#define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7),(1<<0),(1<<1),(1<<3)
#define PCINT_RX_PORT              PORTK
#define PCINT_RX_MASK              PCMSK2
#define PCIR_PORT_BIT              (1<<2)
#define RX_PC_INTERRUPT            PCINT2_vect
#define RX_PCINT_PIN_PORT          PINK

volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
uint8_t rcChannel[RC_CHANS]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit
static int16_t rcData[RC_CHANS];     // interval [1000;2000]
int16_t rcSerial[RC_CHANS];   // interval [1000;2000] - is rcData coming from MSP
uint8_t rcSerialCount = 0;    // a counter to select legacy RX when there is no more MSP rc serial data

// predefined PC pin block (thanks to lianj)  - Version without failsafe
#define RX_PIN_CHECK(pin_pos, rc_value_pos)                        \
  if (mask & PCInt_RX_Pins[pin_pos]) {                             \
    if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
      dTime = cTime-edgeTime[pin_pos];                             \
      if (900<dTime && dTime<2200) {                               \
        rcValue[rc_value_pos] = dTime;                             \
      }                                                            \
    } else edgeTime[pin_pos] = cTime;                              \
}

  // port change Interrupt
ISR(RX_PC_INTERRUPT) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a RX input pin
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;
  
    pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins
   
    mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
    cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
    sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]
  
    RX_PIN_CHECK(0,2);
    RX_PIN_CHECK(1,4);
    RX_PIN_CHECK(2,5);
    RX_PIN_CHECK(3,6);
    RX_PIN_CHECK(4,7);
    RX_PIN_CHECK(5,0);
    RX_PIN_CHECK(6,1);
    RX_PIN_CHECK(7,3);
}

void configureReceiver() {
  /******************    Configure each rc pin for PCINT    ***************************/
  DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
  // PCINT activation
  for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ // i think a for loop is ok for the init.
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
  }
  PCICR = PCIR_PORT_BIT;
}

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG; cli(); // Let's disable interrupts
  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
  SREG = oldSREG;        // Let's restore interrupt state
  return data; // We return the value correctly copied when the IRQ's where disabled
}

void computeRC() {
  uint16_t rcDataTmp;
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan;

  rc4ValuesIndex++;
  if (rc4ValuesIndex == AVERAGING_ARRAY_LENGTH-1) {
    rc4ValuesIndex = 0;
  }
  for (chan = 0; chan < RC_CHANS; chan++) {
    rcDataTmp = readRawRC(chan);
    rcData[chan] = rcDataTmp;
    if (chan<8 && rcSerialCount > 0) { // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
      rcSerialCount --;
      if (rcSerial[chan] >900) {
        rcData[chan] = rcSerial[chan];
      } // only relevant channels are overridden
    }
  }
}

int16_t *getRcData(){
    computeRC();
    return rcData;
}

#else
#error "### LIB RCMegaReceiver WILL BE USED ONLY WITH ARDUINO MEGA 2560 ###"
#endif