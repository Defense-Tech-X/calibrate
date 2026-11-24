#pragma once

#include "Experiment.hpp"

// Step 2. Climb at u_lift for 1s after height is valid, coast at u_eq until
// speed crosses zero, and lock that inertial apex as h_ref.
// Speed damping stays at kV/kU. The active model's kH is tested once, for 5s
// after the apex lock. Staying inside +/- 0.5 px of h_ref accepts that value.
// Samples both above and below h_ref mean kH should be decreased and the test
// restarted. Never rising above h_ref means kH should be increased and the
// test restarted. The run does not try another value.
class HoverTrimExperiment : public Experiment {
public:
    const char* name() const override;
    ExperimentResult run(ExperimentContext& context) override;
};
