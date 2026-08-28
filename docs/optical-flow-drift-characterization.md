# Optical-flow position drift over long distances

Status: characterization, not a gate. The two missions this records land in the
tree and are runnable on demand; neither is registered as a qualification row,
and Sec. 6 says why.

Question asked: does the optical-flow simulation show position drift over long
distances, and does the filter stay honest about it?

**Answer: yes, and yes.** Horizontal error grows as the square root of distance
travelled at 0.074 m per root metre, reaching 1.02 m after 224 m against 0.075 m
for the same flight on GPS. The filter's own position sigma grows with it and
stays LARGER than the error throughout, so the estimate is conservative rather
than overconfident.

## 1. What was flown

`Vehicles/Rdd2/Test/LongRangeFlowMission.mo` and `LongRangeGpsMission.mo`: the
qualification's own box opened from 4 m sides to 50 m, so one circuit is 200 m
of commanded track and 224 m of flown track, at 4 m/s instead of 1 m/s to keep
the run tractable. Identical plant, route, speed, noise seed and controller;
the only difference between the two is which source aids the filter. Flow mode
is the unchanged aiding set: optical flow, barometer and magnetometer, GPS off.

**Range is not limited by the flow model.** The simulated ground is the
infinite plane `opticalFlowGroundNormalWorldEnu = {0,0,1}` at
`opticalFlowGroundPlaneOffset_m = 0`, and `simulateOpticalFlowPlane` derives
feature visibility from height and tilt, never from how far the vehicle has
travelled. What the model DOES assume is a flat, level, infinitely textured
floor. Real flow over 200 m meets terrain relief, texture dropouts and
exposure changes that this fixture does not contain, so every number here is a
floor rather than a field expectation.

## 2. The measurement is odometric, and that was checked rather than assumed

The first thing to rule out was a hidden position leak, because a sim that
showed no drift would be a defect and not a result.
`simulateOpticalFlowPlane` forms its translation flow from `velocityBody`, the
body-frame velocity, divided by per-feature depth. Position enters only through
`numerator := groundPlaneOffset_m - groundNormal * position`, which is the
height above the plane -- a range measurement, which a real flow unit also has.
Horizontal position never reaches the measurement. So flow observes velocity,
the barometer bounds altitude and the magnetometer bounds heading, and nothing
observes horizontal position: it is the integral of an estimated velocity, and
must drift.

The GPS control is what turns that reading into evidence. A growth curve
appearing in both runs would have been the plant, the controller or the route;
one appearing in neither would have been the leak. It appears in flow only.

## 3. Drift rate and shape

Distance is the flown track, not time, and the fits are through the origin over
the airborne segment beyond 5 m.

| | flow | GPS |
| --- | ---: | ---: |
| distance flown | 224.3 m | 223.7 m |
| final horizontal error | **1.022 m** | 0.075 m |
| worst horizontal error | 2.370 m | 0.269 m |
| endpoint rate | **0.456 m / 100 m** | 0.034 m / 100 m |
| sqrt fit | **0.0745 m / sqrt(m)**, rms 0.362 | 0.0076, rms 0.048 |
| linear fit | 0.00583 m / m, rms 0.389 | 0.00058, rms 0.055 |

**The square-root fit wins**, for the error and for the filter's own sigma, so
the drift is random-walk dominated and not heading-bias dominated. Binned means
show the shape directly:

| distance | flow error | flow sigma | GPS error | GPS sigma |
| --- | ---: | ---: | ---: | ---: |
| 5-25 m | 0.083 | 0.313 | 0.086 | 0.118 |
| 50-75 m | 0.214 | 1.025 | 0.060 | 0.104 |
| 100-125 m | 1.353 | 1.961 | 0.083 | 0.104 |
| 150-175 m | 0.872 | 2.364 | 0.114 | 0.104 |
| 200-230 m | 0.978 | 1.685 | 0.068 | 0.107 |

GPS is flat in both columns, which is what an observable position looks like.

Quoting a single rate for a square-root process is quoting one point on it, so
both parameterizations are given: 0.456 m per 100 m is the endpoint ratio at
224 m, while 0.0745 m per root metre is the shape-correct constant and predicts
1.7 m at 500 m and 2.4 m at 1 km.

**One realization cannot separate the two fits decisively**, and the rms values
above are close, 0.362 against 0.389. The independent argument is stronger than
the fit: heading error is bounded at 0.072 rad and is zero-mean rather than a
bias, and a constant 0.072 rad heading error over 224 m would produce 16 m of
cross-track error. The observed 1 m is sixteen times smaller, so the linear
coupling term is absent and what remains is the random walk.

## 4. The other channels stay bounded, and the control says which are shared

| | flow | GPS |
| --- | ---: | ---: |
| worst altitude error | 0.426 m | 0.391 m |
| worst heading error | 0.072 rad (4.11 deg) | 0.038 rad (2.18 deg) |

Altitude is bounded in both and at nearly the same value, so it is the
barometer and its datum rather than anything flow-specific. Heading is bounded
in both by the magnetometer; flow mode is roughly twice as bad, which is the
attitude error that unobservable position feeds back through the velocity
estimate, and it is still small enough not to produce a linear drift term.

## 5. Filter honesty, which is the flight-safety half

| | flow | GPS |
| --- | ---: | ---: |
| position NEES, 3 dof, mean | **1.14** | 2.50 |
| position NEES, median | 0.71 | 2.10 |
| position sigma at end | 0.69 / 0.62 m | 0.074 / 0.082 m |
| flow innovation NIS, 2 dof, mean | **2.015** | n/a |

**The filter does not go overconfident.** Its position sigma grows with
distance, tracking the error's shape, and it stays LARGER than the error: mean
NEES of 1.14 against a dimension of 3 says the actual error is consistently
smaller than the covariance advertises. The failure mode this run was looking
for -- a filter that drifts while reporting a tight covariance, and so lies to
an operator about position quality -- is not present. What is present is the
opposite and milder fault: the covariance is conservative by roughly a factor
of two in NEES terms, so an operator would be told the position is worse than
it is.

Flow innovation NIS sits at 2.015 against its 2 degrees of freedom for the
whole run. Flow itself stays observable and consistent even as position runs
away, which is the expected decomposition: the measurement is fine, the
integral of it is what drifts.

## 6. Recommendation: characterization only, no CI row

Do NOT add a drift-rate ratchet row, for three reasons.

Cost. Each mission is 70 s of simulated flight, about 6 minutes in CI, and the
pair would add roughly 12 minutes to an RDD2 job already at about 32 minutes.

Value. What this measures is a slow-moving architectural property -- position
is unobservable under flow, so it random-walks -- and slow-moving properties do
not regress tick to tick. The existing `eskf_flow` row already gates that flow
correction works at short range, and the consistency gates already catch a
filter going overconfident on any row.

Sensitivity. A single realization of a random walk is a noisy statistic. A
ratchet on 0.0745 m per root metre would either be loose enough to pass a real
regression or tight enough to fail on seed changes, and neither is worth 12
minutes.

Re-run by hand instead, and specifically when `simulateOpticalFlowPlane`, the
flow correction, or the process noise changes -- those are the things that
would move this number. If a gate is ever wanted anyway, the cheapest one that
keeps its meaning is a 100 m circuit at about 35 s, roughly 3 minutes, gating
the square-root coefficient with a wide ratchet; at the 16 m of the existing
box the drift is below the noise floor and cannot be gated at all.
