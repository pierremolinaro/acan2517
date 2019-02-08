//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517 Demo in loopback mode, for Arduino Uno
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517.h>
#include <SPI.h>

//——————————————————————————————————————————————————————————————————————————————
// Very very important: put a 10kΩ resistor between CS and VDD of MCP2517FD

static const byte MCP2517_CS  = 10 ; // CS input of MCP2517 
static const byte MCP2517_INT =  3 ; // INT output of MCP2517

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517 can (MCP2517_CS, SPI, MCP2517_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
//--- Start serial
  Serial.begin (115200) ;
//--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
  }
//----------------------------------- Begin SPI
  SPI.begin () ;
//--- Configure ACAN2517
  Serial.print ("sizeof (ACAN2517Settings): ") ;
  Serial.print (sizeof (ACAN2517Settings)) ;
  Serial.println (" bytes") ;
  Serial.println ("Configure ACAN2517") ;
  ACAN2517Settings settings (ACAN2517Settings::OSC_40MHz, 125UL * 1000UL) ;
  settings.mRequestedMode = ACAN2517Settings::InternalLoopBack ; // Select loopback mode
//--- Default values are too high for an Arduino Uno that contains 2048 bytes of RAM: reduce them
  settings.mDriverTransmitFIFOSize = 1 ;
  settings.mDriverReceiveFIFOSize = 1 ;
//--- RAM Usage
  Serial.print ("MCP2517FD RAM Usage: ") ;
  Serial.print (settings.ramUsage ()) ;
  Serial.println (" bytes") ;
//--- Begin
  const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW:") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

static uint32_t gSendDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  CANMessage frame ;
  if (gSendDate < millis ()) {
    gSendDate += 2000 ;
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentFrameCount) ;
    }else{
      Serial.println ("Send failure") ;
    }
  }
  if (can.available ()) {
    can.receive (frame) ;
    gReceivedFrameCount ++ ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedFrameCount) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
