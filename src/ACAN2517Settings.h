//----------------------------------------------------------------------------------------------------------------------
// A CAN driver for MCP2517 (CAN 2.0B mode)
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2517
//
//----------------------------------------------------------------------------------------------------------------------

#pragma once

//----------------------------------------------------------------------------------------------------------------------

#include <stdint.h>

//----------------------------------------------------------------------------------------------------------------------
//  ACAN2517Settings class
//----------------------------------------------------------------------------------------------------------------------

class ACAN2517Settings {

//······················································································································
//   ENUMERATED TYPES
//······················································································································

  public: typedef enum : uint8_t {
    OSC_4MHz,
    OSC_4MHz_DIVIDED_BY_2,
    OSC_4MHz10xPLL,
    OSC_4MHz10xPLL_DIVIDED_BY_2,
    OSC_20MHz,
    OSC_20MHz_DIVIDED_BY_2,
    OSC_40MHz,
    OSC_40MHz_DIVIDED_BY_2
  } Oscillator ;

  public: typedef enum : uint8_t {
    CLKO_DIVIDED_BY_1,
    CLKO_DIVIDED_BY_2,
    CLKO_DIVIDED_BY_4,
    CLKO_DIVIDED_BY_10, SOF
  } CLKOpin ;

  public: typedef enum : uint8_t {
    InternalLoopBack = 2,
    ExternalLoopBack = 5,
    ListenOnly = 3,
    Normal20B = 6
  } RequestedMode ;

  public: typedef enum : uint8_t {
    Disabled,
    ThreeAttempts,
    UnlimitedNumber
  } RetransmissionAttempts ;

//······················································································································
//   CONSTRUCTOR
//······················································································································

  public: ACAN2517Settings (const Oscillator inOscillator,
                            const uint32_t inDesiredBitRate,
                            const uint32_t inTolerancePPM = 1000) ;

//······················································································································
//   CAN BIT TIMING
//······················································································································

  private: uint32_t mSysClock ; // In Hz
  public: const uint32_t mDesiredBitRate ; // In kb/s
  public: uint16_t mBitRatePrescaler = 0 ; // 1...256
  public: uint16_t mPhaseSegment1 = 0 ; // 2...256
  public: uint8_t mPhaseSegment2 = 0 ; // 1...128
  public: uint8_t mSJW = 0 ; // 1...128
  private: Oscillator mOscillator ;
  public: bool mBitRateClosedToDesiredRate = false ; // The above configuration is not correct

//······················································································································
//    MCP2517FD TXCAN pin is Open Drain ?
//······················································································································

  public: bool mTXCANIsOpenDrain = false ; // false --> Push/Pull Output, true --> Open Drain Output

//······················································································································
//    MCP2517FD INT pin is Open Drain ?
//······················································································································

  public: bool mINTIsOpenDrain = false ; // false --> Push/Pull Output, true --> Open Drain Output

//······················································································································
//    CLKO pin function (default value is MCP2517FD power on setting)
//······················································································································

  public: CLKOpin mCLKOPin = CLKO_DIVIDED_BY_10 ;

//······················································································································
//    Requested mode
//······················································································································

  public: RequestedMode mRequestedMode = Normal20B ;

//······················································································································
//   TRANSMIT FIFO
//······················································································································

//--- Driver transmit buffer size
  public: uint16_t mDriverTransmitFIFOSize = 16 ; // >= 0

//--- Controller transmit FIFO size
  public: uint8_t mControllerTransmitFIFOSize = 32 ; // 1 ... 32

//--- Controller transmit FIFO priority (0 --> lowest, 31 --> highest)
  public: uint8_t mControllerTransmitFIFOPriority = 0 ; // 0 ... 31

//--- Controller transmit FIFO retransmission attempts
  public: RetransmissionAttempts mControllerTransmitFIFORetransmissionAttempts = UnlimitedNumber ;

//······················································································································
//   TXQ BUFFER
//······················································································································

//--- TXQ buffer size (0 --> TXQ disabled)
  public: uint8_t mControllerTXQSize = 0 ; // 0 ... 32


//--- TXQ buffer priority (0 --> lowest, 31 --> highest)
  public: uint8_t mControllerTXQBufferPriority = 31 ; // 0 ... 31

//--- Controller TXQ buffer retransmission attempts
  public: RetransmissionAttempts mControllerTXQBufferRetransmissionAttempts = UnlimitedNumber ;


//······················································································································
//   RECEIVE FIFO
//······················································································································

//--- Driver receive buffer size
  public: uint16_t mDriverReceiveFIFOSize = 32 ; // > 0

//--- Controller receive FIFO size
  public: uint8_t mControllerReceiveFIFOSize = 32 ; // 1 ... 32

//······················································································································
//    SYSCLOCK frequency computation
//······················································································································

  public: static uint32_t sysClock (const Oscillator inOscillator) ;

//······················································································································
//    Accessors
//······················································································································

  public: Oscillator oscillator (void) const { return mOscillator ; }
  public: uint32_t sysClock (void) const { return mSysClock ; }
  public: uint32_t ramUsage (void) const ;
  public: uint32_t actualBitRate (void) const ;
  public: bool exactBitRate (void) const ;

//······················································································································
//    Distance between actual bit rate and requested bit rate (in ppm, part-per-million)
//······················································································································

  public: uint32_t ppmFromDesiredBitRate (void) const ;

//······················································································································
//    Distance of sample point from bit start (in ppc, part-per-cent, denoted by %)
//······················································································································

  public: uint32_t samplePointFromBitStart (void) const ;

//······················································································································
//    Bit settings are consistent ? (returns 0 if ok)
//······················································································································

  public: uint32_t CANBitSettingConsistency (void) const ;

//······················································································································
//    Constants returned by CANBitSettingConsistency
//······················································································································

  public: static const uint32_t kBitRatePrescalerIsZero                 = 1 <<  0 ;
  public: static const uint32_t kBitRatePrescalerIsGreaterThan256       = 1 <<  1 ;
  public: static const uint32_t kPhaseSegment1IsLowerThan2              = 1 <<  2 ;
  public: static const uint32_t kPhaseSegment1IsGreaterThan256          = 1 <<  3 ;
  public: static const uint32_t kPhaseSegment2IsZero                    = 1 <<  4 ;
  public: static const uint32_t kPhaseSegment2IsGreaterThan128          = 1 <<  5 ;
  public: static const uint32_t kSJWIsZero                              = 1 <<  6 ;
  public: static const uint32_t kSJWIsGreaterThan128                    = 1 <<  7 ;
  public: static const uint32_t kSJWIsGreaterThanPhaseSegment1          = 1 <<  8 ;
  public: static const uint32_t kSJWIsGreaterThanPhaseSegment2          = 1 <<  9 ;

//······················································································································
// Max values
//······················································································································

  public: static const uint16_t MAX_BRP             = 256 ;
  public: static const uint16_t MAX_PHASE_SEGMENT_1 = 256 ;
  public: static const uint8_t  MAX_PHASE_SEGMENT_2 = 128 ;
  public: static const uint8_t  MAX_SJW             = 128 ;

} ;

//----------------------------------------------------------------------------------------------------------------------

