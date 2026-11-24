#pragma once

#include "Experiment.hpp"

// Pipeline step 2: after throttle_step liftoff + stopClimb.
// Clearance → dump stopClimb → apex HEIGHT LOCK → thr := thr - K·dy.
// First sink step must approach liftoffThrottle; else bump K and stop.
// Done after 3s relative still on lock; writes hoverThrottle + holdThrPerPx.
class HoverTrimExperiment : public Experiment {
public:
    const char* name() const override;
    ExperimentResult run(ExperimentContext& context) override;
};
