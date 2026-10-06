# Measuring the GPS-to-Mocap survey offset

The two handoff missions share their sensor noise, controller, and coverage
window. The surveyed mission adds 5 cm east and 3 cm north to the rig placement;
the ideal mission adds no offset. Both retain the same crossing-step limits,
source-acceptance checks, flight limits, and per-phase NEES/NIS limits.

Survey discrimination compares the two missions' **estimate minus truth** at
matching estimator ticks. This removes each vehicle's controller motion. The
errors must agree before coverage to within 10 nm. In the first 0.5 seconds of
coverage, and separately throughout the remaining coverage, their mean
difference must recover the configured survey vector. Checking the vector also
checks its sign and direction.

The residual budget is `3 * sqrt(2) * 0.01 m`, derived from the per-axis 1 cm
Mocap noise in each mission. It does not shrink with the number of ticks:
estimator errors are time-correlated, so this is a deterministic mission check,
not an independent-sample confidence interval. The 5.83 cm expected offset is
larger than the 4.24 cm budget; a missing, reversed, doubled, or rotated offset
fails. Missing windows, mismatched ticks, and non-finite errors also fail.

The former check required the surveyed entry step to exceed the largest
unrelated GPS correction anywhere in the flight. That does not follow from the
survey geometry: GPS position noise has 50 cm sigma. With corrected GPS
covariance and initialization, the largest ordinary correction is 25.2 cm,
while the surveyed entry step is 8.76 cm. The paired trace nevertheless recovers
5.02 cm east and 3.01 cm north at entry, and 5.00 cm east and 3.00 cm north after
settling. Changing the physical survey or filter to inflate that single step
would conceal the faulty comparison.

`Vehicles.Rdd2.Test.test_waypoint_qualification` runs in the regression job. Its
negative controls verify that larger shared GPS corrections do not hide a valid
offset and that absent, wrong, premature, or disappearing offsets fail.
