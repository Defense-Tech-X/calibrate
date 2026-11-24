#pragma once

#include "identify/AltitudePHold.hpp"
#include "identify/VisionMotionParams.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

// Altitude hold laws for calibrate pipeline.
//
// Pipeline roles:
//   hover_trim   -> apex lock + AltitudePHold::stepCascade (SoftFloor+SoftCeil)
//   roll_pulse   -> same cascade + SoftCeil while measuring rollHold*
//   pitch_pulse  -> same SoftCeil + LAT (LAT frozen during pitch pulse)
//
// Image Y down. Close screen gap to lock: dy>0 => CUT; dy<0 => ADD.
namespace AltitudeHold {

inline constexpr int kFloorBelowHover = AltitudePHold::kFloorBelowHover;
inline constexpr int kSettleFloorBelowHover = 1500;
inline constexpr int kSettleDumpBelowHover = 150;
inline constexpr float kKpAlt = AltitudePHold::kKpAltDefault;
inline constexpr float kVzMax = AltitudePHold::kVzMaxDefault;
inline constexpr float kKv = AltitudePHold::kKvDefault;
inline constexpr float kAltDeadband = AltitudePHold::kAltDeadband;
inline constexpr int kSlewMax = AltitudePHold::kSlewMax;
inline constexpr float kThrPerPx = AltitudePHold::kThrPerPxDefault;

enum class AboveLockPolicy {
    SoftCeil,  // never ADD while above lock (hover_trim / roll / pitch)
    SoftTop = SoftCeil, // alias kept for call sites
    SoftNoCut, // never CUT while sinking from above; ADD toward hover allowed
    None,
};

enum class BelowLockPolicy {
    SoftFloor,
    None,
};

inline int floorThr(uint16_t measuredHover) {
    return std::max(0, static_cast<int>(measuredHover) - kFloorBelowHover);
}

inline int settleFloorThr(uint16_t measuredHover) {
    return std::max(0, static_cast<int>(measuredHover) - kSettleFloorBelowHover);
}

inline int settleDumpThr(uint16_t measuredHover) {
    return std::max(
        settleFloorThr(measuredHover),
        static_cast<int>(measuredHover) - kSettleDumpBelowHover);
}

using PStep = AltitudePHold::PStep;
using CascadeStep = AltitudePHold::PStep;

inline PStep stepP(
    float dyFromLockPx,
    uint16_t hoverThr,
    uint16_t currentThr,
    int floorCap,
    int ceilCap,
    float thrPerPx = kThrPerPx,
    int slewMax = kSlewMax,
    float altDeadbandPx = kAltDeadband,
    bool boostAuthorityBelowLock = true) {
    return AltitudePHold::stepP(
        dyFromLockPx,
        hoverThr,
        currentThr,
        floorCap,
        ceilCap,
        thrPerPx,
        slewMax,
        altDeadbandPx,
        boostAuthorityBelowLock);
}

// Thin policy wrapper around plant AltitudePHold::stepCascade.
// Gains come from visionMotion (hover_trim); SoftCeil/SoftFloor match that law.
inline CascadeStep stepCascade(
    float dyFromLockPx,
    float vzPxPerSec,
    uint16_t hoverThr,
    uint16_t currentThr,
    int floorCap,
    int ceilCap,
    const VisionMotionParams& gains,
    AboveLockPolicy abovePolicy = AboveLockPolicy::SoftCeil,
    BelowLockPolicy belowPolicy = BelowLockPolicy::SoftFloor,
    int slewMax = kSlewMax) {
    const bool softFloor = (belowPolicy == BelowLockPolicy::SoftFloor);
    // SoftNoCut: SoftCeil off (plant SoftCeil already allows ADD to hover).
    const bool softCeil = (abovePolicy == AboveLockPolicy::SoftCeil);

    CascadeStep out = AltitudePHold::stepCascade(
        dyFromLockPx,
        vzPxPerSec,
        hoverThr,
        currentThr,
        floorCap,
        ceilCap,
        gains,
        slewMax,
        softFloor,
        softCeil);

    if (out.skipped || out.frozen) {
        return out;
    }

    // SoftNoCut: while sinking from above, never CUT (pitch_pulse).
    const float altDb = gains.holdAltDeadband > 0.0f ? gains.holdAltDeadband
        : (gains.hoverHoldDyPx > 0.0f ? gains.hoverHoldDyPx : kAltDeadband);
    if (abovePolicy == AboveLockPolicy::SoftNoCut && dyFromLockPx > altDb &&
        vzPxPerSec < 0.0f && out.rawTarget < static_cast<int>(currentThr)) {
        out.rawTarget = static_cast<int>(currentThr);
        const int lo = std::min(floorCap, ceilCap);
        const int hi = std::max(floorCap, ceilCap);
        out.limitedTarget = std::clamp(out.rawTarget, lo, hi);
        out.delta = AltitudePHold::applySlew(
            out.limitedTarget - static_cast<int>(currentThr), slewMax);
    }
    return out;
}

} // namespace AltitudeHold
