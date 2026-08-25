within Estimation;

package Mahony
  "Nonlinear explicit complementary attitude filter on SO(3)"
  annotation(Documentation(info = "<html>
    <p>Clean-room implementation of the explicit complementary filter with
    gyroscope-bias estimation, written from the published mathematics of
    R. Mahony, T. Hamel, and J.-M. Pflimlin, &quot;Nonlinear Complementary
    Filters on the Special Orthogonal Group&quot;, IEEE Transactions on
    Automatic Control, vol. 53, no. 5, pp. 1203-1218, 2008. No third-party
    filter source code was consulted.</p>
    <p>The filter fuses a rate gyroscope with vectorial direction
    measurements (accelerometer gravity direction, optionally a
    magnetometer heading) through proportional and integral feedback of
    the vector-product innovation. It carries only a unit quaternion and a
    gyro-bias vector, needs no covariance matrices and no linear algebra,
    and is written as branch-light scalar arithmetic so the generated
    embedded C fits and runs on very small single-precision or even
    softfloat microcontrollers.</p>
    <h4>Measured embedded footprint</h4>
    <p>Pipeline: <code>rumoca compile -m Estimation.Mahony.Filter --target
    embedded-c-galec</code> (rumoca build-info 9d867568ad8b-dirty, emitted
    Estimation_Mahony_Filter.c sha256 13c53b29..., rumoca_galec_kernels.c
    sha256 4ade7423...), cross-compiled with arm-none-eabi-gcc 15.2.1
    (Arm GNU Toolchain 15.2.Rel1) at <code>-Os -std=c99
    -ffunction-sections</code>, the same toolchain and flags as the
    Vehicles.Rdd2.NavigationEstimator ESKF dossier. Executed instructions
    per update are exact counts from qemu-system-arm 10.0.11 translation
    block traces (mps2-an385/an386/an500 machines), as the per-step delta
    of a 500-step driver against the same driver with the step call
    removed; they include the block's kernel-wrapper and driver call
    overhead. Cycles are a stated in-order model (loads and stores 2,
    branches 2, FP div and sqrt 14, FP MAC 3, ldm/stm 1 plus registers,
    otherwise 1), not a silicon measurement.</p>
    <table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
    <tr><th>build</th><th>model .text</th><th>dostep</th>
    <th>state struct</th><th>dostep stack</th>
    <th>insns/update accel only</th><th>insns/update with mag</th>
    <th>modeled cycles/update</th></tr>
    <tr><td>Cortex-M7 fpv5-d16 hard</td><td>1780 B</td><td>1128 B</td>
    <td>280 B</td><td>96 B</td><td>354</td><td>390</td>
    <td>686 / 753</td></tr>
    <tr><td>Cortex-M4 fpv4-sp-d16 hard</td><td>1804 B</td><td>1144 B</td>
    <td>280 B</td><td>96 B</td><td>360</td><td>396</td>
    <td>692 / 759</td></tr>
    <tr><td>Cortex-M3 softfloat</td><td>2588 B</td><td>1800 B</td>
    <td>280 B</td><td>112 B</td><td>6187</td><td>8285</td>
    <td>7694 / 10198</td></tr>
    </table>
    <p>On a 20 MHz core the modeled step budget is: Cortex-M4F 0.69 /
    1.73 / 3.46 percent CPU at 200 / 500 / 1000 Hz accel-only (0.76 /
    1.90 / 3.80 percent with magnetometer); Cortex-M3 softfloat 7.7 /
    19.2 / 38.5 percent accel-only (10.2 / 25.5 / 51.0 percent with
    magnetometer). RAM: 280 B state plus at most 112 B stack per
    instance, no heap and no static scratch, under 0.13 percent of a
    320 KB part. Flash: the whole linked demonstrator including newlib
    sqrtf is 4.9 KB on the softfloat build and 2.4 KB on the FPU
    builds.</p>
    <p>Comparison row, same rumoca plus arm-gcc pipeline: the
    Vehicles.Rdd2.NavigationEstimator ESKF measures 63310 B model text,
    59452 B state, and a worst-case aided step of about 2.69 million
    Cortex-M7 instructions (600 MHz class hardware); this block runs the
    same IMU tick problem at 1780 to 2588 B text, 280 B state, and 354
    to 8285 instructions, which is what makes a 20 MHz, 320 KB
    microcontroller a comfortable target.</p>
  </html>"));
end Mahony;
