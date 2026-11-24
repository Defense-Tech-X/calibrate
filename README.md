# Calibrate suite

Mathematical model of drone control from reference points on the visual-recognition matrix.

The fit reads the point cloud (`PointsReader`) and the sticks' current position (`ObservationChannel`). It describes how that cloud moves when commands are written back to the sticks (`ControllerChannel`). `task` flies the maneuver with the finished model.

## Build

```powershell
cd d:\projects\bpla\calibrate
cmake --build --preset debug --target calibrate
```

Binary: `calibrate\out\debug\calibrate.exe`

## Live pipeline (VelociDrone)

Vision-client and virtual controller running. Each experiment starts with **reset** (~3 s, thr 0, sticks centered) so you can arm / prepare the sim UI.

```powershell
.\calibrate\out\debug\calibrate.exe throttle_step
.\calibrate\out\debug\calibrate.exe hover_trim
.\calibrate\out\debug\calibrate.exe roll_pulse
.\calibrate\out\debug\calibrate.exe pitch_pulse
.\calibrate\out\debug\calibrate.exe validate_forecast
```

### `throttle_step`

Visual vertical channel for one calibration flight. From point tracks with stable ids, frame timestamps, and the throttle command actually sent, it reports a hypothesis of visual height (px) and vertical speed (px/s) in that flight's own scale. Height and speed are two outputs of this one channel. Each sample also carries support, raw quality figures, validity reasons, and limits such as rotation or extraneous motion. `quality` is the score of those explicit rules, and it is a probability only after a separate check.

Speed is the step between two adjacent frames. Height uses whatever non-empty group agrees on the downward shift. A missing sample stays invalid and is not stored as zero. A valid near-zero speed is an observation of no noticeable motion.

The scale locks once a track has moved down by at least 1 px. The same `stepUs` (0.2 s) is the stick hold while the picture is still and the window of the speed slope. The model records that basis as `stepUs` and `minCoherentDyPx`. The sweep raises throttle until that speed is a valid climb, records `liftoffThrottle`, dumps, then cuts until a valid descent records `stopClimbThrottle`.

### Case: coefficients accumulate one calibration at a time

Session `61766524357` is the worked example. Each step writes the coefficients it can see and leaves the next one open.

**`throttle_step`** saw open-loop throttle against visual height \(h\) (px) and vertical speed \(v\) (px/s). In the air the record fits

\[
\dot h = v, \qquad \dot v = k_u (u - u_{eq}) - k_v v
\]

| Coefficient | Value | Reading |
|---|---|---|
| \(u_{eq}\) | 11210 | `stopClimbThrottle`: at the apex, \(h \approx 22\,\mathrm{px}\), \(v\) crosses zero |
| \(u_{lift}\) | 12880 | `liftoffThrottle`: no motion on the ground below this stick |
| \(k_u / k_v\) | 0.0076 px/s per stick count | along the cut, \(v \approx (u - 11210) / 130\) |
| \(k_u\) | 0.004 px/s² per stick count | just after liftoff, \(v\) rose about 4 px/s² with ~1500 counts of excess throttle |
| \(k_v\) | 0.53 1/s | speed settles in about 2 s |

On the ground and below \(u_{lift}\), \(\dot v = 0\). After liftoff the balance sits at \(u_{eq}\), 1670 counts lower. That deadzone is the nonlinearity this step established.

**`hover_trim`** is the next coefficient. The same law as a height hold is

\[
u = u_{eq} - \frac{k_h}{k_u}(h - h_{ref}) - \frac{k_v}{k_u} v
\]

\(k_u\) and \(k_v\) stay as above. The damping term is 132.5 stick counts per px/s. `hover_trim` tests the single \(k_h\) stored on the active model. After the apex lock that value stays fixed for 5 s, starting from rest at \(h_{ref}\). Staying inside \(\pm 0.5\,\mathrm{px}\) accepts it. Motion both above and below \(h_{ref}\) means decrease \(k_h\) and run the test again. Never rising above \(h_{ref}\) means increase \(k_h\) and run the test again.

Repeated runs fixed \(k_h = 2.6907\). At \(2.6906\), three holds stayed below \(h_{ref}\). At \(2.6907\), one of three holds rose above \(h_{ref}\). With the coefficients already on the model, `holdThrottle` is

\[
u = 11210 - 672.675\,(h - h_{ref}) - 132.5\, v
\]

\(h\) and \(h_{ref}\) are pixels of the flight scale, \(v\) is px/s, and \(u\) is stick counts. \(672.675 = k_h / k_u = 2.6907 / 0.004\) counts per pixel of height error. \(132.5 = k_v / k_u = 0.53 / 0.004\) counts per px/s. \(11210\) is \(u_{eq}\) from `throttle_step`. This is the second coefficient written onto that same plant: \(u_{eq}\), \(k_u\), and \(k_v\) were already fixed, and `hover_trim` adds \(k_h\).

Session files: `visual_channel.jsonl` (one hypothesis per cloud) and `visual_scale.json` (reference observation, normalization version, sensitivities, rejection and gap rules).

| Step | Experiment | Uses prior | Measures / locks | Hold during measure |
|---|---|---|---|---|
| 1 | `throttle_step` | - | visual height and vertical speed hypothesis (px, px/s), plus quality and limits. Stores `liftoffThrottle` and `stopClimbThrottle` | raise until a clean climb hypothesis, dump, cut until a clean descent hypothesis |
| 2 | `hover_trim` | \(u_{eq}\), \(u_{lift}\), \(k_u\), \(k_v\) | \(k_h\) | 1 s at \(u_{lift}\), coast at \(u_{eq}\) until \(v = 0\), then \(u = u_{eq} - (k_h/k_u)(h-h_{ref}) - (k_v/k_u)\,v\) |
| 3 | `roll_pulse` | hover + stopClimb + cascade | `sticks.rollHoldStep` / `rollHoldMax`, ARX draft | shared `ApexClimb` then SoftCeil hold (no LAT - same axis as excite) |
| 4 | `pitch_pulse` | hover + rollHold | pitch response / ARX draft | shared `ApexClimb` then SoftCeil + LAT P2 (LAT frozen during pulse) |
| 5 | `validate_forecast` | ARX OK model | NRMSE on short roll pulse; `appliedToControl` | hover thr, one roll pulse |

Active model: `plant/models/active.json` (OK fit from roll/pitch is kept; sticks-only refresh must not overwrite it). After step 5 accept → `takeoff-hover`.

### Accepted hold laws (pulse support)

**Altitude** (`hold/AltitudeHold.hpp`):

- Shared takeoff (`takeoff/ApexClimb.hpp`): reset → climb liftoff → **clearance 1.0s** → dump `stopClimb` → coast to apex lock. Used by `roll_pulse`, `pitch_pulse`.
- `hover_trim`: 1 s at \(u_{lift}\) after the channel height is valid, then \(u_{eq}\) until speed crosses zero. That apex is \(h_{ref}\). The active model's \(k_h\) is held for 5 s from that point. A hold inside 0.5 px is accepted. Oscillation across \(h_{ref}\) means a smaller \(k_h\). Failure to rise above \(h_{ref}\) means a larger \(k_h\). `hoverThrottle` stays at \(u_{eq}\).
- Pulse altitude hold (SoftCeil/SoftFloor): SoftCeil = above lock never ADD. Apex lock keeps `stopClimb` thr (no jump to measured hover); floor down to `stopClimb`. Cascade reference stays measured `hover`.

**Lateral** (`hold/LateralHold.hpp`, pitch only): oppose drift - craft LEFT→roll+, RIGHT→roll−; gains from `rollHoldStep` (Kr/8, slew≤step/8) plus leaky `dx` bias, `latRate` clamp 3×holdRate, 2×Kx and faster trim when `|dx|` large. During pitch `pulse` phase `holdRoll` is frozen; trim resumes in hold/recover. Not used on `roll_pulse` (same stick axis as excitation).

Apex picture lock stays fixed during pulses (no still-relock); cascade runs through pulse/recover.

### Logs

`SETTLE DUMP`, `SETTLE climb STOPPED`, `MOTION=ALT … action=CUT|ADD|HOLD`, `LAT_HOLD`, `ROLL_RESP`, `validate_forecast: accepted nrmse=…`.

Session micro-trim is local. `hover_trim` writes \(k_h\) and leaves `hoverThrottle` at \(u_{eq}\).

## Offline smoke

```powershell
.\calibrate\out\debug\calibrate.exe fit_synthetic
```

Synthetic OK model → `plant/models/active.json` without hardware.
