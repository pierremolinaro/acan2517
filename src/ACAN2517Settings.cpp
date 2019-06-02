//----------------------------------------------------------------------------------------------------------------------
// A CAN driver for MCP2517 (CAN 2.0B mode)
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2517
//
//----------------------------------------------------------------------------------------------------------------------

#include <ACAN2517Settings.h>

//----------------------------------------------------------------------------------------------------------------------
//    sysClock
//----------------------------------------------------------------------------------------------------------------------

uint32_t ACAN2517Settings::sysClock (const Oscillator inOscillator) {
  uint32_t sysClock = 40UL * 1000 * 1000 ;
  switch (inOscillator) {
  case OSC_4MHz:
    sysClock =  4UL * 1000 * 1000 ;
    break ;
  case OSC_4MHz_DIVIDED_BY_2:
    sysClock =  2UL * 1000 * 1000 ;
    break ;
  case OSC_4MHz10xPLL_DIVIDED_BY_2 :
  case OSC_40MHz_DIVIDED_BY_2:
  case OSC_20MHz:
    sysClock = 20UL * 1000 * 1000 ;
    break ;
  case OSC_20MHz_DIVIDED_BY_2:
    sysClock = 10UL * 1000 * 1000 ;
    break ;
  case OSC_4MHz10xPLL:
  case OSC_40MHz:
    break ;
  }
  return sysClock ;
}

//----------------------------------------------------------------------------------------------------------------------
//   CONSTRUCTOR
//----------------------------------------------------------------------------------------------------------------------

ACAN2517Settings::ACAN2517Settings (const Oscillator inOscillator,
                                    const uint32_t inDesiredBitRate,
                                    const uint32_t inTolerancePPM) :
mSysClock (sysClock (inOscillator)),
mDesiredBitRate (inDesiredBitRate),
mOscillator (inOscillator) {
  const uint32_t maxTQCount = MAX_PHASE_SEGMENT_1 + MAX_PHASE_SEGMENT_2 + 1 ; // Setting for slowest bit rate
  uint32_t BRP = MAX_BRP ;
  uint32_t smallestError = UINT32_MAX ;
  uint32_t bestBRP = 1 ; // Setting for highest bit rate
  uint32_t bestTQCount = 4 ; // Setting for highest bit rate
  uint32_t TQCount = mSysClock / inDesiredBitRate / BRP ;
//--- Loop for finding best BRP and best TQCount
  while ((TQCount <= (MAX_PHASE_SEGMENT_1 + MAX_PHASE_SEGMENT_2 + 1)) && (BRP > 0)) {
  //--- Compute error using TQCount
    if ((TQCount >= 4) && (TQCount <= maxTQCount)) {
      const uint32_t error = mSysClock - inDesiredBitRate * TQCount * BRP ; // error is always >= 0
      if (error <= smallestError) {
        smallestError = error ;
        bestBRP = BRP ;
        bestTQCount = TQCount ;
      }
    }
  //--- Compute error using TQCount+1
    if ((TQCount >= 3) && (TQCount < maxTQCount)) {
      const uint32_t error = inDesiredBitRate * (TQCount + 1) * BRP - mSysClock ; // error is always >= 0
      if (error <= smallestError) {
        smallestError = error ;
        bestBRP = BRP ;
        bestTQCount = TQCount + 1 ;
      }
    }
  //--- Continue with next value of BRP
    BRP -= 1 ;
    TQCount = (BRP == 0) ? (maxTQCount + 1) : (mSysClock / inDesiredBitRate / BRP) ;
  }
//--- Compute PS2 (1 <= PS2 <= 128)
  uint32_t PS2 = bestTQCount / 5 ; // For sampling point at 80%
  if (PS2 == 0) {
    PS2 = 1 ;
  }else if (PS2 > MAX_PHASE_SEGMENT_2) {
    PS2 = MAX_PHASE_SEGMENT_2 ;
  }
//--- Compute PS1 (1 <= PS1 <= 256)
  uint32_t PS1 = bestTQCount - PS2 - 1 /* Sync Seg */ ;
  if (PS1 > MAX_PHASE_SEGMENT_1) {
    PS2 += PS1 - MAX_PHASE_SEGMENT_1 ;
    PS1 = MAX_PHASE_SEGMENT_1 ;
  }
//---
  mBitRatePrescaler = (uint16_t) bestBRP ;
  mPhaseSegment1 = (uint16_t) PS1 ;
  mPhaseSegment2 = (uint8_t) PS2 ;
  mSJW = mPhaseSegment2 ; // Always 1 <= SJW <= 128, and SJW <= mPhaseSegment2
//--- Final check of the configuration
  const uint32_t W = bestTQCount * mDesiredBitRate * bestBRP ;
  const uint64_t diff = (mSysClock > W) ? (mSysClock - W) : (W - mSysClock) ;
  const uint64_t ppm = (uint64_t) (1000UL * 1000UL) ; // UL suffix is required for Arduino Uno
  mBitRateClosedToDesiredRate = (diff * ppm) <= (((uint64_t) W) * inTolerancePPM) ;
} ;

//----------------------------------------------------------------------------------------------------------------------
//   ACCESSORS
//----------------------------------------------------------------------------------------------------------------------

uint32_t ACAN2517Settings::actualBitRate (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mPhaseSegment1 + mPhaseSegment2 ;
  return mSysClock / mBitRatePrescaler / TQCount ;
}

//----------------------------------------------------------------------------------------------------------------------

bool ACAN2517Settings::exactBitRate (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mPhaseSegment1 + mPhaseSegment2 ;
  return mSysClock == (mBitRatePrescaler * mDesiredBitRate * TQCount) ;
}

//----------------------------------------------------------------------------------------------------------------------

uint32_t ACAN2517Settings::ppmFromDesiredBitRate (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mPhaseSegment1 + mPhaseSegment2 ;
  const uint32_t W = TQCount * mDesiredBitRate * mBitRatePrescaler ;
  const uint64_t diff = (mSysClock > W) ? (mSysClock - W) : (W - mSysClock) ;
  const uint64_t ppm = (uint64_t) (1000UL * 1000UL) ; // UL suffix is required for Arduino Uno
  return (uint32_t) ((diff * ppm) / W) ;
}

//----------------------------------------------------------------------------------------------------------------------

uint32_t ACAN2517Settings::samplePointFromBitStart (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mPhaseSegment1 + mPhaseSegment2 ;
  const uint32_t samplePoint = 1 /* Sync Seg */ + mPhaseSegment1 ;
  const uint32_t partPerCent = 100 ;
  return (samplePoint * partPerCent) / TQCount ;
}

//----------------------------------------------------------------------------------------------------------------------

uint32_t ACAN2517Settings::CANBitSettingConsistency (void) const {
  uint32_t errorCode = 0 ; // Means no error
  if (mBitRatePrescaler == 0) {
    errorCode |= kBitRatePrescalerIsZero ;
  }else if (mBitRatePrescaler > MAX_BRP) {
    errorCode |= kBitRatePrescalerIsGreaterThan256 ;
  }
  if (mPhaseSegment1 < 2) {
    errorCode |= kPhaseSegment1IsLowerThan2 ;
  }else if (mPhaseSegment1 > MAX_PHASE_SEGMENT_1) {
    errorCode |= kPhaseSegment1IsGreaterThan256 ;
  }
  if (mPhaseSegment2 == 0) {
    errorCode |= kPhaseSegment2IsZero ;
  }else if (mPhaseSegment2 > MAX_PHASE_SEGMENT_2) {
    errorCode |= kPhaseSegment2IsGreaterThan128 ;
  }
  if (mSJW == 0) {
    errorCode |= kSJWIsZero ;
  }else if (mSJW > MAX_SJW) {
    errorCode |= kSJWIsGreaterThan128 ;
  }
  if (mSJW > mPhaseSegment1) {
    errorCode |= kSJWIsGreaterThanPhaseSegment1 ;
  }
  if (mSJW > mPhaseSegment2) {
    errorCode |= kSJWIsGreaterThanPhaseSegment2 ;
  }
  return errorCode ;
}

//----------------------------------------------------------------------------------------------------------------------

uint32_t ACAN2517Settings::ramUsage (void) const {
  uint32_t result = 0 ;
//--- TXQ
  result += 16 * mControllerTXQSize ;
//--- Receive FIFO (FIFO #1)
  result += 16 * mControllerReceiveFIFOSize ;
//--- Send FIFO (FIFO #2)
  result += 16 * mControllerTransmitFIFOSize ;
//---
  return result ;
}

//----------------------------------------------------------------------------------------------------------------------

