//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517 / ACAN Demo
//  ACAN2517 uses hardware SPI1 and an external interrupt pin
//  This sketch runs only on a Teensy 3.5 or 3.6 (or 3.1 / 3.2 see below)
//  It uses the Teensy 3.x builtin CAN0 interface for testing intensive
//  communication with a MCP2517 CAN controller.
//  The builtin CAN0 interface and the MCP2517 controller should be connected
//  throught transceivers.
//  Note that the Tx and Rx alternate pins are used for the Teensy builtin CAN0.
//  On a teensy 3.1 / 3.2, do not use alternate pins, they are not supported
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN.h>      // For the Teensy 3.x builtin CAN
#include <ACAN2517.h>  // For the external MCP2517

//——————————————————————————————————————————————————————————————————————————————
// Select CAN baud rate.
// Select a baud rate common to the builtin CAN interface and the MCP2517

static const uint32_t CAN_BIT_RATE = 1000 * 1000 ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2517 connections: adapt theses settings to your design
//  As hardware SPI is used, you should select pins that support SPI functions.
//  This sketch is designed for a Teensy 3.5/3.6, using SPI1
//  But standard Teensy 3.5/3.6 SPI1 pins are not used
//    SCK input of MCP2517 is connected to pin #32
//    SDI input of MCP2517 is connected to pin #0
//    SDO output of MCP2517 is connected to pin #1
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

ACAN2517 can (MCP2517_CS, SPI1, MCP2517_INT) ;

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
  Serial.print ("MCP2517FD RAM usage: ") ;
  Serial.print (settings2517.ramUsage ()) ;
  Serial.println (" bytes") ;
  settings2517.mDriverReceiveFIFOSize = 1 ;
  const uint32_t errorCode2517 = can.begin (settings2517, [] { can.isr () ; }) ;
  if (errorCode2517 == 0) {
    Serial.println ("ACAN2517 configuration: ok") ;
  }else{
    Serial.print ("ACAN2517 configuration error 0x") ;
    Serial.println (errorCode2517, HEX) ;
  }
//--- Configure ACAN
  ACANSettings settings (CAN_BIT_RATE) ;
  settings.mUseAlternateTxPin = true ;
  settings.mUseAlternateRxPin = true ;
  const uint32_t errorCode = ACAN::can0.begin (settings) ;
  if (errorCode == 0) {
    Serial.println ("ACAN configuration: ok") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
  }else{
    Serial.print ("ACAN configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gReceivedFrameCount2517 = 0 ;
static uint32_t gSentFrameCount = 0 ;
static uint32_t gSentFrameCount2517 = 0 ;

static const uint32_t MESSAGE_COUNT = 100UL * 1000 ;

static CANMessage gMessageSentByMCP2517 ;
static bool gMCP2517ReadyToSend = true ;
static CANMessage gMessageSentByTeensy ;
static bool gTeensyReadyToSend = true ;

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
  }
//--- Send messages via the MCP2517
  if (gMCP2517ReadyToSend && (gSentFrameCount2517 < MESSAGE_COUNT)) {
  //--- Make an even identifier for MCP2517
    gMessageSentByMCP2517.id = 1 << (gSentFrameCount2517 % 29) ; // millis () & 0x1FFFFFFE ;
    gMessageSentByMCP2517.len = 8 ;
    gMessageSentByMCP2517.ext = true ;
    for (uint8_t i=0 ; i<8 ; i++) {
      gMessageSentByMCP2517.data [i] = i ;
    }
  //--- Send frame via MCP2517
    const bool ok = can.tryToSend (gMessageSentByMCP2517) ;
    if (ok) {
      gSentFrameCount2517 += 1 ;
      gMCP2517ReadyToSend = false ;
    }
  }
//--- Send messages via the builtin CAN0
  if (gTeensyReadyToSend && (gSentFrameCount < MESSAGE_COUNT)) {
  //--- Make an odd identifier for builtin CAN0
    gMessageSentByTeensy.id = 1 << (gReceivedFrameCount2517 % 29) ; // (millis () & 0x1FFFFFFE) | 1 ;
    gMessageSentByTeensy.len = 8 ;
    gMessageSentByTeensy.ext = true ;
  //--- Send frame via builtin CAN0
    const bool ok = ACAN::can0.tryToSend (gMessageSentByTeensy) ;
    if (ok) {
      gSentFrameCount += 1 ;
      gTeensyReadyToSend = false ;
    }
  }
//--- Receive frame from MCP2517
  CANMessage frame ;
  if (can.receive (frame)) {
    gReceivedFrameCount2517 ++ ;
    bool ok = gMessageSentByTeensy.ext == frame.ext ;
    if (ok) {
      ok = gMessageSentByTeensy.id == frame.id ;
    }
    if (ok) {
      gTeensyReadyToSend = true ;
    }else{
      Serial.print ("MCP2517 Reception error: ext ") ;
      Serial.print (gMessageSentByTeensy.ext) ;
      Serial.print (" -> ") ;
      Serial.println (frame.ext) ;
      Serial.print ("MCP2517 Reception error: id ") ;
      Serial.print (gMessageSentByTeensy.id, HEX) ;
      Serial.print (" -> ") ;
      Serial.println (frame.id, HEX) ;
    }
  }
//--- Receive frame via builtin CAN0
  if (ACAN::can0.available ()) {
    ACAN::can0.receive (frame) ;
    gReceivedFrameCount ++ ;
    bool ok = frame.len == 8 ;
    for (uint8_t i=0 ; (i<8) && ok ; i++) {
      ok = frame.data [i] == i ;
    }
    if (ok) {
      ok = gMessageSentByMCP2517.ext == frame.ext ;
    }
    if (ok) {
      ok = gMessageSentByMCP2517.id == frame.id ;
    }
    if (ok) {
      gMCP2517ReadyToSend = true ;
    }else{
      Serial.print ("Teensy Reception error: ext ") ;
      Serial.print (gMessageSentByMCP2517.ext) ;
      Serial.print (" -> ") ;
      Serial.println (frame.ext) ;
      Serial.print ("Teensy Reception error: id ") ;
      Serial.print (gMessageSentByMCP2517.id, HEX) ;
      Serial.print (" -> ") ;
      Serial.println (frame.id, HEX) ;
    }
  }
}

//——————————————————————————————————————————————————————————————————————————————
