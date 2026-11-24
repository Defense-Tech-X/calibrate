#pragma once

#include <algorithm>
#include <cmath>

// Accepted lateral (roll) hold for pitch_pulse (P2 on VelociDrone).
// Requires sticks.rollHoldStep / rollHoldMax from roll_pulse.
// Vision: latRate>0 / growing +dx = craft LEFT; latRate<0 = craft RIGHT.
// Oppose: LEFT -> roll+, RIGHT -> roll-.
namespace LateralHold {

// Soft P2 base: Kr/8, Kx/64, slew≤step/8; plus dx bias + rate-spike clamp.
inline constexpr float kRateGainDiv = 8.0f;
inline constexpr float kPosGainDiv = 64.0f;
inline constexpr float kBiasGainDiv = 64.0f;
inline constexpr float kRateClampMult = 3.0f;
inline constexpr float kBiasLeakPerSec = 0.25f;
inline constexpr float kPosEscalateMult = 2.0f;

inline float rateGain(int rollHoldStep, float holdRatePxPerSec) {
    return static_cast<float>(rollHoldStep) / (kRateGainDiv * std::max(holdRatePxPerSec, 0.5f));
}

inline float posGain(int rollHoldStep) {
    return static_cast<float>(rollHoldStep) / kPosGainDiv;
}

inline float biasGain(int rollHoldStep) {
    return static_cast<float>(rollHoldStep) / kBiasGainDiv;
}

inline int slewMax(int rollHoldStep) {
    return std::max(1, rollHoldStep / 8);
}

inline float posDeadband(float hoverHoldDyPx) {
    return std::max(1.5f, hoverHoldDyPx * 2.0f);
}

inline float escalatePosDeadband(float posDeadbandPx) {
    return posDeadbandPx * kPosEscalateMult;
}

inline float clampRate(float latRate, float holdRatePxPerSec) {
    const float lim = kRateClampMult * std::max(holdRatePxPerSec, 0.5f);
    return std::clamp(latRate, -lim, lim);
}

inline float effectivePosGain(int rollHoldStep, float dx, float posDeadbandPx) {
    const float kx = posGain(rollHoldStep);
    return std::fabs(dx) > escalatePosDeadband(posDeadbandPx) ? kx * kPosEscalateMult : kx;
}

// Leaky integral of signed dx (same sign as LEFT/+dx → +bias → +roll).
inline float updateBias(
    float bias,
    float dx,
    float dtSec,
    float posDeadbandPx,
    int rollHoldStep,
    int rollTrimMax) {
    const float lim = static_cast<float>(std::max(1, std::min(rollTrimMax, rollHoldStep)));
    const float dt = std::max(dtSec, 0.0f);
    if (std::fabs(dx) < posDeadbandPx) {
        bias *= std::max(0.0f, 1.0f - kBiasLeakPerSec * dt);
    } else {
        bias += biasGain(rollHoldStep) * dx * dt;
    }
    return std::clamp(bias, -lim, lim);
}

inline uint64_t trimIntervalUs(uint64_t baseIntervalUs, float dx, float posDeadbandPx) {
    if (std::fabs(dx) > escalatePosDeadband(posDeadbandPx)) {
        return baseIntervalUs / 2;
    }
    return baseIntervalUs;
}

} // namespace LateralHold
