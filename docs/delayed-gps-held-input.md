# Delayed GPS and uncertainty in a held IMU packet

`ESKF.correctGps` estimates the state at GPS capture time by retrodicting the
current nominal state with the latest held IMU packet. That packet is a noisy
measurement. Treating it as exact underestimates the GPS innovation covariance,
particularly when velocity aiding is precise and transport latency spans many
IMU packets.

The same packet has also just predicted the current state, so its contribution
to the retrodicted observation is correlated with the current state error.
Adding observation variance alone does not account for that correlation.

For a preintegrated packet of duration `dt`, the covariance of its mean angular
rate and specific force is `Q = diag(Qgyro, Qaccel) / dt`, using the configured
continuous noise densities. `heldInputJacobian` integrates the existing local
error transition and noise-input matrix over an interval `h`:

```
B(h) = h (I + h A / 2 + h² A² / 6 + h³ A³ / 24) Gimu
```

With GPS age `a`, delayed-state observation Jacobian `Hd`, and the existing
current-to-delayed transition `Phi(-a)`, the observation has:

```
H = Hd Phi(-a)
J = Hd B(-a)
R = Rgps + J Q J'
C = B(dt) Q J'
```

Here `C` is `Cov(current state error, observation noise)`. The correlated
measurement update uses:

```
S = H P H' + H C + C' H' + R
K = (P H' + C) S^-1
F = I - K H
Pposterior = F P F' + K R K' - F C K' - K C' F'
```

The generalized Joseph form remains valid after the existing attitude trust
limit or consider-state projection changes the gain. With `C=0`, this is the
existing independent-noise update. Neither the innovation gate nor the mission
qualification thresholds change.

A scalar velocity example exposes the error without a vehicle simulation.
Let `dt=0.01 s`, `a=0.11 s`, accelerometer density `0.03 m²/s³`, prior velocity
variance `0.01 m²/s²`, and GPS velocity variance `0.01 m²/s²`. The latest packet
has mean noise variance `3 m²/s⁴`. After prediction `P=0.0103`, while
`R=0.0463` and `C=-0.0033`. The correct innovation variance is `0.05`, compared
with `0.0203` when the held input is treated as exact. This can more than double
the velocity contribution to the reported innovation statistic.

`Tests.CorrelatedGpsTests` checks a scalar correlated update and a joint GPS
update against independent linear-Gaussian reference values, including gain,
innovation statistic and posterior covariance. It also checks the signs and
position/velocity integration terms of the held-input Jacobian.

This correction applies to joint GPS position/velocity aiding after a fresh
preintegrated prediction. Optional covariance inputs default to zero for callers
that do not supply a packet uncertainty model. Retrodiction still assumes a
constant held input over the delay; this is not a replacement for a fixed-lag
history and replay estimator when the vehicle motion changes substantially
within that interval.
