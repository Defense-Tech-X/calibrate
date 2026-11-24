#pragma once

#include "Experiment.hpp"

// Step 1. Visual vertical channel for one calibration flight.
// The channel reports a hypothesis of visual height (px) and vertical speed (px/s)
// in this flight's scale, with quality, limits, and the time of that hypothesis.
// A clean climb hypothesis records liftoffThrottle; a clean descent hypothesis
// records stopClimbThrottle. Coefficient identification is a later change.
class ThrottleStepExperiment : public Experiment {
public:
    const char* name() const override;
    ExperimentResult run(ExperimentContext& context) override;
};
