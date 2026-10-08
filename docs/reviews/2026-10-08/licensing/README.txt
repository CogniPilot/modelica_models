Native comparison source boundary qualification
8 October 2026

The auxiliary PX4 coning probe no longer contains the transcribed ArduPilot
recurrence. An independently executed external GPL-marked native oracle uses
verbatim pinned backend arithmetic and actual AP_Math vector types. Its source,
license and executable stay in owned scratch storage. No ArduPilot code or
objects are linked into ESKF. See docs/estimator-licensing.txt for the boundary
and the remaining limits of this engineering review.

All twelve native oracle/probe sample schedules match. Against the independent
double-precision quaternion ODE, the ArduPilot native arithmetic is closer in
12/12 cases. Maximum angular errors are 1.72963145e-7 rad for ArduPilot and
4.88895596e-6 rad for PX4. This checks isolated coning arithmetic, not the full
sensor backend, flight performance, or an estimator superiority result.

Receipts contain hashes, commands and numerical outputs only. Extracted GPL
source and binaries are not included here. The previous historical coning
receipt remains labeled as using a transcription; it is not reinterpreted as
this native run. This cleanup does not certify all historical source provenance.
