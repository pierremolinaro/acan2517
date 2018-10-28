//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// An utility class for:
//   - ACAN2517 CAN driver for MCP2517FD (CAN 2.0B mode)
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2517
//
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#ifndef ACAN2517_FILTERS_CLASS_DEFINED
#define ACAN2517_FILTERS_CLASS_DEFINED

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include <CANMessage.h>

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//  ACAN2517Filters class
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

class ACAN2517Filters {

//······················································································································
//   EMBEDDED CLASS
//······················································································································

  private: class Filter {
    public: Filter * mNextFilter ;
    public: const uint32_t mFilterMask ;
    public: const uint32_t mAcceptanceFilter ;
    public: const ACANCallBackRoutine mCallBackRoutine ;

    public: Filter (const uint32_t inFilterMask,
                    const uint32_t inAcceptanceFilter,
                    const ACANCallBackRoutine inCallBackRoutine) :
    mNextFilter (NULL),
    mFilterMask (inFilterMask),
    mAcceptanceFilter (inAcceptanceFilter),
    mCallBackRoutine (inCallBackRoutine) {
    }

  //--- No copy
    private: Filter (const Filter &) ;
    private: Filter & operator = (const Filter &) ;
  } ;

//······················································································································
//   ENUMERATED TYPE
//······················································································································

  public: typedef enum {
      kFiltersOk,
      kStandardIdentifierTooLarge,
      kExtendedIdentifierTooLarge,
      kStandardAcceptanceTooLarge,
      kExtendedAcceptanceTooLarge,
      kStandardMaskTooLarge,
      kExtendedMaskTooLarge,
      kInconsistencyBetweenMaskAndAcceptance
  } FilterStatus ;

//······················································································································
//   CONSTRUCTOR
//······················································································································

  public: ACAN2517Filters (void) {}

//······················································································································
//   DESTRUCTOR
//······················································································································

  public: ~ ACAN2517Filters (void) {
    while (mFirstFilter != NULL) {
      Filter * next = mFirstFilter->mNextFilter ;
      delete mFirstFilter ;
      mFirstFilter = next ;
    }
  }

//······················································································································
//   RECEIVE FILTERS
//······················································································································

  public: void appendPassAllFilter (const ACANCallBackRoutine inCallBackRoutine) {  // Accept any frame
    Filter * f = new Filter (0, 0, inCallBackRoutine) ;
    if (mFirstFilter == NULL) {
      mFirstFilter = f ;
    }else{
      mLastFilter->mNextFilter  = f ;
    }
    mLastFilter = f ;
    mFilterCount += 1 ;
  }

//······················································································································

  public: void appendFormatFilter (const tFrameFormat inFormat, // Accept any identifier
                                   const ACANCallBackRoutine inCallBackRoutine) {
    Filter * f = new Filter (1 << 30,
                             (inFormat == kExtended) ? (1 << 30) : 0,
                             inCallBackRoutine) ;
    if (mFirstFilter == NULL) {
      mFirstFilter = f ;
    }else{
      mLastFilter->mNextFilter  = f ;
    }
    mLastFilter = f ;
    mFilterCount += 1 ;
  }

//······················································································································

  public: void appendFrameFilter (const tFrameFormat inFormat,
                                  const uint32_t inIdentifier,
                                  const ACANCallBackRoutine inCallBackRoutine) {
  //--- Check identifier
    if (inFormat == kExtended) {
      if (inIdentifier > 0x1FFFFFFF) {
        mFilterStatus = kExtendedIdentifierTooLarge ;
        mFilterErrorIndex = mFilterCount ;
      }
    }else if (inIdentifier > 0x7FF) {
      mFilterStatus = kStandardIdentifierTooLarge ;
      mFilterErrorIndex = mFilterCount ;
    }
  //--- Enter filter
    const uint32_t mask = (1 << 30) | ((inFormat == kExtended) ? 0x1FFFFFFF : 0x7FF) ;
    const uint32_t acceptance = inIdentifier | ((inFormat == kExtended) ? (1 << 30) : 0) ;
    Filter * f = new Filter (mask, acceptance, inCallBackRoutine) ;
    if (mFirstFilter == NULL) {
      mFirstFilter = f ;
    }else{
      mLastFilter->mNextFilter  = f ;
    }
    mLastFilter = f ;
    mFilterCount += 1 ;
  }

//······················································································································

  public: void appendFilter (const tFrameFormat inFormat,
                             const uint32_t inMask,
                             const uint32_t inAcceptance,
                             const ACANCallBackRoutine inCallBackRoutine) {
  //--- Check consistency between mask and acceptance
    if ((inMask & inAcceptance) != inAcceptance) {
      mFilterStatus = kInconsistencyBetweenMaskAndAcceptance ;
      mFilterErrorIndex = mFilterCount ;
    }
  //--- Check identifier
    if (inFormat == kExtended) {
      if (inAcceptance > 0x1FFFFFFF) {
        mFilterStatus = kExtendedAcceptanceTooLarge ;
        mFilterErrorIndex = mFilterCount ;
      }
    }else if (inAcceptance > 0x7FF) {
      mFilterStatus = kStandardAcceptanceTooLarge ;
      mFilterErrorIndex = mFilterCount ;
    }
  //--- Check mask
    if (inFormat == kExtended) {
      if (inMask > 0x1FFFFFFF) {
        mFilterStatus = kExtendedMaskTooLarge ;
        mFilterErrorIndex = mFilterCount ;
      }
    }else if (inMask > 0x7FF) {
      mFilterStatus = kStandardMaskTooLarge ;
      mFilterErrorIndex = mFilterCount ;
    }
  //--- Enter filter
    const uint32_t mask = (1 << 30) | inMask ;
    const uint32_t acceptance = ((inFormat == kExtended) ? (1 << 30) : 0) | inAcceptance ;
    Filter * f = new Filter (mask, acceptance, inCallBackRoutine) ;
    if (mFirstFilter == NULL) {
      mFirstFilter = f ;
    }else{
      mLastFilter->mNextFilter  = f ;
    }
    mLastFilter = f ;
    mFilterCount += 1 ;
  }

//······················································································································
//   ACCESSORS
//······················································································································

  public: FilterStatus filterStatus (void) const { return mFilterStatus ; }

  public: uint8_t filterErrorIndex (void) const { return mFilterErrorIndex ; }

  public: uint8_t filterCount (void) const { return mFilterCount ; }

//······················································································································
//   PRIVATE PROPERTIES
//······················································································································

  private: uint8_t mFilterCount = 0 ;
  private: Filter * mFirstFilter = NULL ;
  private: Filter * mLastFilter  = NULL ;
  private: FilterStatus mFilterStatus = kFiltersOk ;
  private: uint8_t mFilterErrorIndex = 0 ;

//······················································································································
//   NO COPY
//······················································································································

  private: ACAN2517Filters (const ACAN2517Filters &) ;
  private: ACAN2517Filters & operator = (const ACAN2517Filters &) ;

//······················································································································
// Friend
//······················································································································

  friend class ACAN2517 ;

//······················································································································

} ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#endif
