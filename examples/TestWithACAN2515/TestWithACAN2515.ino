//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517 / ACAN2515 Demo
//  ACAN2517 uses hardware SPI1 and an external interrupt pin
//  This sketch runs only on a Teensy 3.x
//  It uses the Teensy 3.x builtin CAN0 interface for testing intensive
//  communication with a MCP2517 CAN controller.
//  The builtin CAN0 interface and the MCP2517 controller should be connected
//  throught transceivers.
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515.h>  // For the external MCP2515
#include <ACAN2517.h>  // For the external MCP2517FD

//——————————————————————————————————————————————————————————————————————————————
// Select CAN baud rate.
// Select a baud rate common to the MCP2515 and the MCP2517FD

static const uint32_t CAN_BIT_RATE = 1000 * 1000 ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2517 connections: adapt theses settings to your design
//  As hardware SPI is used, you should select pins that support SPI functions.
//  This sketch is designed for a Teensy 3.5, using SPI1
//  But standard Teensy 3.5 SPI1 pins are not used
//    SCK input of MCP2517 is connected to pin #32
//    SDI input of MCP2517 is connected to pin #0
//    SDO input of MCP2517 is connected to pin #1
//  CS input of MCP2517 should be connected to a digital output port
//  INT output of MCP2517 should be connected to a digital input port, with interrupt capability
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2517_SCK = 32 ; // SCK input of MCP2517
static const byte MCP2517_SDI =  0 ; // SI input of MCP2517
static const byte MCP2517_SDO =  1 ; // SO output of MCP2517

static const byte MCP2517_CS  = 31 ; // CS input of MCP2517
static const byte MCP2517_INT = 38 ; // INT output of MCP2517

//——————————————————————————————————————————————————————————————————————————————
//  MCP2517 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517 can2517 (MCP2517_CS, SPI1, MCP2517_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 connections: adapt theses settings to your design
//  This sketch is designed for a Teensy 3.5, using SPI0 (named SPI)
//  But standard Teensy 3.5 SPI0 pins are not used
//    SCK input of MCP2515 is pin #27
//    SI input of MCP2515 is pin #28
//    SO output of MCP2515 is pin #39
//  User code should configure MCP2515_IRQ pin as external interrupt
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_SCK = 27 ; // SCK input of MCP2515
static const byte MCP2515_SI  = 28 ; // SI input of MCP2515
static const byte MCP2515_SO  = 39 ; // SO output of MCP2515

static const byte MCP2515_CS  = 20 ; // CS input of MCP2515
static const byte MCP2515_INT = 37 ; // INT output of MCP2515

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can2515 (MCP2515_CS, SPI, MCP2515_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t MCP2515_QUARTZ_FREQUENCY = 16 * 1000 * 1000 ; // 16 MHz

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
//--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
//--- Start serial
  Serial.begin (38400) ;
//--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
//--- Define alternate pins for SPI1 (see https://www.pjrc.com/teensy/td_libs_SPI.html)
  SPI1.setMOSI (MCP2517_SDI) ;
  SPI1.setMISO (MCP2517_SDO) ;
  SPI1.setSCK (MCP2517_SCK) ;
//--- Begin SPI
  SPI1.begin () ;
//--- Configure ACAN2517
  ACAN2517Settings settings2517 (ACAN2517Settings::OSC_4MHz10xPLL, CAN_BIT_RATE) ;
  const uint32_t errorCode2517 = can2517.begin (settings2517, [] { can2517.isr () ; }) ;
  if (errorCode2517 == 0) {
    Serial.println ("ACAN2517 configuration: ok") ;
  }else{
    Serial.print ("ACAN2517 configuration error 0x") ;
    Serial.println (errorCode2517, HEX) ;
  }
//--- Configure ACAN2515
//--- Define alternate pins for SPI0 (see https://www.pjrc.com/teensy/td_libs_SPI.html)
//    These settings are defined by Teensyduino for Teensy 3.x
  SPI.setMOSI (MCP2515_SI) ;
  SPI.setMISO (MCP2515_SO) ;
  SPI.setSCK (MCP2515_SCK) ;
//--- Configure SPI
  SPI.begin () ;
//--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings2515 (MCP2515_QUARTZ_FREQUENCY, CAN_BIT_RATE) ;
  const uint32_t errorCode2515 = can2515.begin (settings2515, [] {can2515.isr () ; }) ;
  if (errorCode2515 != 0) {
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode2515, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gReceivedFrameCount2517 = 0 ;
static uint32_t gSentFrameCount = 0 ;
static uint32_t gSentFrameCount2517 = 0 ;

static const uint32_t MESSAGE_COUNT = 10UL * 1000 ;

//——————————————————————————————————————————————————————————————————————————————
// A CAN network requires that stations do not send frames with the same identifier.
// So:
//   - MCP2517 sends frame with even identifier values;
//   - builtin CAN0 sends frame with odd identifier values;

void loop () {
//--- Blink led
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 1000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print (gSentFrameCount) ;
    Serial.print (" ") ;
    Serial.print (gReceivedFrameCount) ;
    Serial.print (" ") ;
    Serial.print (gSentFrameCount2517) ;
    Serial.print (" ") ;
    Serial.println (gReceivedFrameCount2517) ;
//    Serial.print (" ") ;
//    Serial.println (can.readErrorCounters (), HEX) ;
  }
  CANMessage frame ;
//--- Send messages via the MCP2517
  if (gSentFrameCount2517 < MESSAGE_COUNT) {
  //--- Make an even identifier for MCP2517
    frame.id = millis () & 0x7FE ;
    frame.len = 8 ;
    for (uint8_t i=0 ; i<8 ; i++) {
      frame.data [i] = i ;
    }
  //--- Send frame via MCP2517
    const bool ok = can2517.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount2517 += 1 ;
    }
  }
//--- Send messages via the MCP2515, trying all transmit buffers
  if (gSentFrameCount < MESSAGE_COUNT) {
  //--- Make an odd identifier for builtin CAN0
    frame.id = millis () & 0x7FE ;
    frame.id |= 1 ;
    frame.len = 8 ;
  //--- Send frame via the MCP2515, using transmit buffer 0
    bool ok = can2515.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
    }
  //--- Send frame via the MCP2515, using transmit buffer 1
    frame.idx = 1 ;
    ok = can2515.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
    }
  //--- Send frame via the MCP2515, using transmit buffer 2
    frame.idx = 2 ;
    ok = can2515.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
    }
  }
//--- Receive frame from MCP2517
  if (can2517.receive (frame)) {
//    can.receive (frame) ;
    gReceivedFrameCount2517 ++ ;
  }
//--- Receive frame via builtin CAN0
  if (can2515.available ()) {
    can2515.receive (frame) ;
    bool ok = frame.len == 8 ;
    for (uint8_t i=0 ; (i<8) && ok ; i++) {
      ok = frame.data [i] == i ;
    }
    if (!ok) {
      Serial.println ("RECEIVED DATA ERROR") ;
    }
    gReceivedFrameCount ++ ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
