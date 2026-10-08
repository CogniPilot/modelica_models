/* SPDX-License-Identifier: Apache-2.0
 * Replay the generated Modelica ESKF with its generated 800 Hz FOH integrator.
 * Native comparator sources are kept in the separately licensed harness.
 */
#define _POSIX_C_SOURCE 200809L
#ifdef COMPARE_UKF
#include "Estimation_StrapdownINS_UKF_Estimator.h"
typedef EstimatorState NavigationEstimatorState;
#define NavigationEstimator_startup Estimator_startup
#define NavigationEstimator_dostep Estimator_dostep
#define status_gpsPositionCorrectionAccepted gpsPositionCorrectionAccepted
#define status_predictionAccepted predictionAccepted
#define errorCovariance stateCovariance
#else
#include "Vehicles_Rdd2_NavigationEstimator.h"
#endif
#include "Tests_PreintegrationReplay.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static NavigationEstimatorState estimator;
#ifndef COMPARE_HORIZON
static PreintegrationReplayState preintegrator;
#endif
#include "flow_packets.h"
#include "replay_timing.h"
#include "transport.h"
static SensorTransport transport;
static void copy3(float out[3], const float in[3]) {
  memcpy(out, in, 3 * sizeof(float));
}
#ifndef COMPARE_HORIZON
static void copy33(float out[3][3], const float in[3][3]) {
  memcpy(out, in, 9 * sizeof(float));
}
#endif
#ifdef COMPARE_HORIZON
#include "horizon.h"
#endif
static bool parse_delay(const char *text, double *seconds) {
  char *end;
  const double milliseconds = strtod(text, &end);
  if (end == text || *end || !isfinite(milliseconds) || milliseconds < 0 ||
      milliseconds > 500)
    return false;
  *seconds = .001 * milliseconds;
  return true;
}

int main(int argc, char **argv) {
  if (argc < 5) {
    fprintf(stderr,
            "usage: replay IMU.csv MODEL_INPUT.csv OUTPUT.csv "
            "gps|denied|transition [foh|zoh|mean [DIAGNOSTICS.csv]] "
            "[--delays GPS_MS FLOW_MS MAG_MS BARO_MS JITTER_MS SEED] "
            "[--timing PROFILE.json] [--stationary-until SECONDS] "
            "[--imu-noise-density GYRO ACCEL] [--flow-packets FLOW.csv]\n");
    return 2;
  }
  int argument = 5;
  const char *integration = argument < argc && strncmp(argv[argument], "--", 2)
                                ? argv[argument++]
                                : "foh";
  const char *diagnostic_path =
      argument < argc && strncmp(argv[argument], "--", 2) ? argv[argument++]
                                                          : NULL;
  const char *timing_path = NULL;
  const char *flow_packet_path = NULL;
  bool use_transport = false;
  double stationary_until_s = 0;
  double imu_noise_density[2] = {0};
  double delays[4] = {0}, jitter = 0;
  uint32_t delay_seed = 7;
  while (argument < argc) {
    const char *option = argv[argument++];
    if (!strcmp(option, "--delays") && argc - argument >= 6) {
      use_transport = true;
      for (unsigned source = 0; source < SOURCE_COUNT; ++source)
        if (!parse_delay(argv[argument++], &delays[source]))
          return 2;
      if (!parse_delay(argv[argument++], &jitter))
        return 2;
      char *end;
      const char *seed_text = argv[argument++];
      const unsigned long long seed = strtoull(seed_text, &end, 10);
      if (end == seed_text || *end || seed > UINT32_MAX)
        return 2;
      delay_seed = (uint32_t)seed;
    } else if (!strcmp(option, "--stationary-until") && argument < argc) {
      char *end;
      const char *value = argv[argument++];
      stationary_until_s = strtod(value, &end);
      if (end == value || *end || !isfinite(stationary_until_s) ||
          stationary_until_s < 0 || stationary_until_s > 60)
        return 2;
#ifdef COMPARE_UKF
      return 2;
#endif
    } else if (!strcmp(option, "--imu-noise-density") && argc - argument >= 2) {
      for (unsigned sensor = 0; sensor < 2; ++sensor) {
        char *end;
        const char *value = argv[argument++];
        imu_noise_density[sensor] = strtod(value, &end);
        if (end == value || *end || !isfinite(imu_noise_density[sensor]) ||
            imu_noise_density[sensor] < 1e-9 || imu_noise_density[sensor] > 100)
          return 2;
      }
    } else if (!strcmp(option, "--flow-packets") && argument < argc) {
      flow_packet_path = argv[argument++];
#ifdef COMPARE_UKF
      return 2;
#endif
    } else if (!strcmp(option, "--timing") && argument < argc) {
      timing_path = argv[argument++];
      timing_enabled = true;
    } else
      return 2;
  }
  if (jitter < 0 || jitter > .5 || !isfinite(jitter))
    return 2;
  for (unsigned source = 0; source < SOURCE_COUNT; ++source)
    if (delays[source] < 0 || delays[source] > .5 || !isfinite(delays[source]))
      return 2;
#ifdef COMPARE_HORIZON
  use_transport = true;
  if (!strcmp(integration, "mean"))
    return 2;
  for (unsigned source = 0; source < SOURCE_COUNT; ++source)
    if (delays[source] + jitter + .01 > .2)
      return 2;
#endif
  transport_startup(&transport, delays, jitter, delay_seed);
  if (flow_packet_path &&
      (!use_transport || !flow_exposure_open(flow_packet_path)))
    return 2;
  if (diagnostic_path && timing_path && !strcmp(diagnostic_path, "-") &&
      !strcmp(timing_path, "-"))
    return 2;
  const int mean_packet = strcmp(integration, "mean") == 0;
  if (strcmp(argv[4], "gps") && strcmp(argv[4], "denied") &&
      strcmp(argv[4], "transition"))
    return 2;
  if (strcmp(integration, "foh") && strcmp(integration, "zoh") && !mean_packet)
    return 2;
  FILE *imu = fopen(argv[1], "r");
  FILE *sensors = fopen(argv[2], "r");
  FILE *out = strcmp(argv[3], "-") == 0 ? stdout : fopen(argv[3], "w");
  if (!imu || !sensors || !out) {
    perror("replay files");
    return 1;
  }
  FILE *diagnostics = diagnostic_path ? (strcmp(diagnostic_path, "-") == 0
                                             ? stderr
                                             : fopen(diagnostic_path, "w"))
                                      : NULL;
  if (diagnostic_path && !diagnostics) {
    perror("diagnostics");
    return 1;
  }
  if (diagnostics) {
    fprintf(diagnostics, "t_s");
#ifdef COMPARE_HORIZON
    fprintf(diagnostics, ",fusion_t_s,fusion_ready");
#endif
    fprintf(diagnostics, ",bgx,bgy,bgz,bax,bay,baz");
    for (int row = 0; row < 15; ++row)
      for (int column = 0; column < 15; ++column)
        fprintf(diagnostics, ",p%d_%d", row, column);
#ifdef SQUARE_ROOT_COVARIANCE
    for (int row = 0; row < 15; ++row)
      for (int column = 0; column < 15; ++column)
        fprintf(diagnostics, ",l%d_%d", row, column);
#endif
    fputc('\n', diagnostics);
  }
  NavigationEstimator_startup(&estimator);
#ifndef COMPARE_HORIZON
  PreintegrationReplay_startup(&preintegrator);
  preintegrator.useFirstOrderHold = strcmp(integration, "foh") == 0;
  preintegrator.samplePeriod = mean_packet ? .01f : .00125f;
#endif
  estimator.initialTerrainAltitudeWorldEnu_m = -1.0f;
  estimator.opticalFlowGroundPlaneOffset_m = -1.0f;
#ifndef COMPARE_UKF
  estimator.pseudoPositionVariance_m2 = 0;
  estimator.zeroVelocityVariance_m2_s2 = 0;
#ifdef SEMIDIRECT_BIAS
  estimator.useSemiDirectBias = true;
#endif
#ifdef EQUIVARIANT_MAGNETOMETER
  estimator.useEquivariantMagnetometer = true;
#endif
#ifdef GEOMETRIC_ALIGNMENT
  estimator.useGeometricAlignment = true;
#endif
#ifdef STATIONARY_IMU_MODEL
  estimator.useStationaryImu = true;
#endif
#ifdef SQUARE_ROOT_COVARIANCE
  estimator.useSquareRootCovariance = true;
#endif
#else
  // Match RDD2 process noise, retaining UKF's declared initial variances.
  // Matching RDD2's 1e-6 gyro-bias variance causes lowerCholesky to reject
  // the initial prior in float32 (global threshold ~1.8e-6).
  for (int j = 0; j < 3; j++) {
    estimator.gyroscope_rad2_s[j][j] = 1e-4f;
    estimator.accelerometer_m2_s3[j][j] = 3e-2f;
    estimator.gyroscopeBias_rad2_s3[j][j] = 1e-10f;
    estimator.accelerometerBias_m2_s5[j][j] = 1e-6f;
  }
#endif
  if (imu_noise_density[0] > 0) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      estimator.gyroscope_rad2_s[axis][axis] =
          imu_noise_density[0] * imu_noise_density[0];
      estimator.accelerometer_m2_s3[axis][axis] =
          imu_noise_density[1] * imu_noise_density[1];
    }
  }
  const float field[3] = {-1.59e-6f, 20.04e-6f, -47.91e-6f};
  copy3(estimator.localMagneticFieldWorldEnu_T, field);
  for (int i = 0; i < 3; i++) {
    estimator.covarianceBody_T2[i][i] = .3e-6f * .3e-6f;
    estimator.gps_positionCovarianceWorld_m2[i][i] =
        i == 2 ? .35f * .35f : .2f * .2f;
    estimator.velocityCovarianceWorld_m2_s2[i][i] = .05f * .05f;
    estimator.integratedGyroscopeCovariance_rad2[i][i] = 1e-10f;
  }
  estimator.variance_m2 = .01f;
  estimator.groundDistanceVariance_m2 = .0004f;
  estimator.opticalFlow_integrationTime_s = .01f;
  estimator.quality = 1;
  for (int i = 0; i < 2; i++)
    estimator.integratedLineOfSightCovariance_rad2[i][i] = 1e-8f;
#ifdef COMPARE_HORIZON
  horizon_startup(strcmp(integration, "foh") == 0, delays, jitter);
#endif
  char line[2048];
  fgets(line, sizeof line, imu);
  fgets(line, sizeof line, sensors);
  fprintf(out, "t_s,e_m,n_m,u_m,ve_m_s,vn_m_s,vu_m_s,qw,qx,qy,qz,pos_valid,att_"
               "valid,gps_fused,flow_fused,mag_fused,baro_fused,step_status,"
               "prediction_accepted");
#ifdef COMPARE_HORIZON
  fprintf(
      out,
      ",fusion_t_s,horizon_e_m,horizon_n_m,horizon_u_m,horizon_ve_m_s,horizon_"
      "vn_m_s,horizon_vu_m_s,horizon_qw,horizon_qx,horizon_qy,horizon_qz");
#endif
  fputc('\n', out);
  unsigned imu_tick = 0;
  while (fgets(line, sizeof line, imu)) {
    double t;
    float gyro[3], accel[3];
    if (sscanf(line, "%lf,%f,%f,%f,%f,%f,%f", &t, gyro, gyro + 1, gyro + 2,
               accel, accel + 1, accel + 2) != 7)
      return 1;
#ifndef COMPARE_HORIZON
    copy3(preintegrator.rate, gyro);
    copy3(preintegrator.force, accel);
    if (imu_tick == 0) {
      // Seed previous endpoint without integrating a fictional interval.
      copy3(preintegrator.previous_previousRate, gyro);
      copy3(preintegrator.previous_previousForce, accel);
      copy3(preintegrator.previousRate, gyro);
      copy3(preintegrator.previousForce, accel);
    } else if (!mean_packet) {
      preintegrator.clear = (imu_tick % 8 == 1);
      if (preintegrator.clear) {
        copy3(preintegrator.gyroBias, estimator.gyroscopeBiasBodyFlu_rad_s);
        copy3(preintegrator.accelBias, estimator.accelerometerBiasBodyFlu_m_s2);
      }
      const uint64_t preintegration_start = timing_now();
      PreintegrationReplay_dostep(&preintegrator);
      timing_record(&preintegration_timing, preintegration_start);
      if (preintegrator.rumoca_galec_error_signal_status) {
        fprintf(stderr, "preintegrator status=%u at %.3f\n",
                preintegrator.rumoca_galec_error_signal_status, t);
        return 1;
      }
    }
#endif
    if (imu_tick % 8 == 0) {
      if (!fgets(line, sizeof line, sensors))
        return 1;
      double row[25];
      char *q = line, *end;
      for (int j = 0; j < 25; j++) {
        row[j] = strtod(q, &end);
        if (end == q)
          return 1;
        q = end + (*end == ',');
      }
      if (fabs(row[0] - t) > 1e-7) {
        fprintf(stderr, "input timestamps differ\n");
        return 1;
      }
#ifndef COMPARE_HORIZON
      if (mean_packet && imu_tick > 0) {
        preintegrator.clear = true;
        for (int axis = 0; axis < 3; ++axis) {
          preintegrator.rate[axis] = row[1 + axis];
          preintegrator.force[axis] = row[4 + axis];
        }
        copy3(preintegrator.gyroBias, estimator.gyroscopeBiasBodyFlu_rad_s);
        copy3(preintegrator.accelBias, estimator.accelerometerBiasBodyFlu_m_s2);
        const uint64_t preintegration_start = timing_now();
        PreintegrationReplay_dostep(&preintegrator);
        timing_record(&preintegration_timing, preintegration_start);
        if (preintegrator.rumoca_galec_error_signal_status)
          return 1;
      }
#endif
      const int gps =
          strcmp(argv[4], "denied") &&
          !(strcmp(argv[4], "transition") == 0 && t >= 25 && t < 40);
      estimator.gps_valid = estimator.positionValid = estimator.velocityValid =
          gps;
      estimator.gps_fresh = gps && row[7] > .5;
      estimator.gps_timestamp_s = row[8];
      for (int j = 0; j < 3; j++) {
        estimator.gps_positionWorldEnu_m[j] = row[9 + j];
        estimator.gps_velocityWorldEnu_m_s[j] = row[12 + j];
      }
#ifndef COMPARE_HORIZON
      estimator.imu_valid = estimator.imu_fresh = true;
      estimator.imu_timestamp_s = t;
      estimator.imu_integrationTime_s = .01f;
      copy3(estimator.imu_angularVelocityBodyFlu_rad_s, gyro);
      copy3(estimator.specificForceBodyFlu_m_s2, accel);
      copy3(estimator.deltaPositionBodyFlu_m, preintegrator.position);
      copy3(estimator.deltaVelocityBodyFlu_m_s, preintegrator.velocity);
      memcpy(estimator.deltaQuaternionBodyFlu, preintegrator.quaternion,
             4 * sizeof(float));
      copy3(estimator.gyroscopeBiasLinearizationBodyFlu_rad_s,
            preintegrator.gyroBias);
      copy3(estimator.accelerometerBiasLinearizationBodyFlu_m_s2,
            preintegrator.accelBias);
      for (int j = 0; j < 3; j++)
        estimator.deltaAngleBodyFlu_rad[j] = row[1 + j] * .01f;
      copy33(estimator.deltaRotationGyroscopeBiasJacobian_s,
             preintegrator.rotationGyro);
      copy33(estimator.deltaVelocityGyroscopeBiasJacobian_m,
             preintegrator.velocityGyro);
      copy33(estimator.deltaVelocityAccelerometerBiasJacobian_s,
             preintegrator.velocityAccel);
      copy33(estimator.deltaPositionGyroscopeBiasJacobian_m_s,
             preintegrator.positionGyro);
      copy33(estimator.deltaPositionAccelerometerBiasJacobian_s2,
             preintegrator.positionAccel);
      if (imu_tick == 0) {
        estimator.deltaQuaternionBodyFlu[0] = 1;
        estimator.deltaVelocityBodyFlu_m_s[2] = 9.81f * .01f;
      }
#endif
      estimator.opticalFlow_valid = estimator.opticalFlow_fresh = true;
      estimator.opticalFlow_timestamp_s = row[15];
      estimator.groundDistance_m = row[18];
      for (int j = 0; j < 3; j++)
        estimator.integratedGyroscopeBodyFlu_rad[j] = row[1 + j] * .01f;
      estimator.integratedLineOfSight_rad[0] =
          -row[17] / row[18] * .01f -
          estimator.integratedGyroscopeBodyFlu_rad[0];
      estimator.integratedLineOfSight_rad[1] =
          row[16] / row[18] * .01f -
          estimator.integratedGyroscopeBodyFlu_rad[1];
      estimator.magnetometer_valid = estimator.barometer_valid = true;
      estimator.magnetometer_fresh = estimator.barometer_fresh = row[19] > .5;
      estimator.magnetometer_timestamp_s = estimator.barometer_timestamp_s =
          row[20];
      for (int j = 0; j < 3; j++)
        estimator.magneticFieldBodyFlu_T[j] = row[21 + j];
      estimator.altitudeWorldEnu_m = row[24];
      if (use_transport) {
        if (!transport_step(&transport, row, argv[4]))
          return 1;
        const TransportQueue *gps_packet = &transport.queues[GPS_SOURCE];
        const TransportQueue *flow_packet = &transport.queues[FLOW_SOURCE];
        const TransportQueue *mag_packet = &transport.queues[MAG_SOURCE];
        const TransportQueue *baro_packet = &transport.queues[BARO_SOURCE];
        estimator.gps_valid = estimator.positionValid =
            estimator.velocityValid = gps_packet->valid;
        estimator.gps_fresh = gps_packet->fresh;
        estimator.gps_timestamp_s = gps_packet->held[8];
        estimator.opticalFlow_valid = flow_packet->valid;
        estimator.opticalFlow_fresh = flow_packet->fresh;
        estimator.opticalFlow_timestamp_s = flow_packet->held[15];
        estimator.groundDistance_m = flow_packet->held[18];
        estimator.magnetometer_valid = mag_packet->valid;
        estimator.magnetometer_fresh = mag_packet->fresh;
        estimator.magnetometer_timestamp_s = mag_packet->held[20];
        estimator.barometer_valid = baro_packet->valid;
        estimator.barometer_fresh = baro_packet->fresh;
        estimator.barometer_timestamp_s = baro_packet->held[20];
        estimator.altitudeWorldEnu_m = baro_packet->held[24];
        for (unsigned axis = 0; axis < 3; ++axis) {
          estimator.gps_positionWorldEnu_m[axis] = gps_packet->held[9 + axis];
          estimator.gps_velocityWorldEnu_m_s[axis] =
              gps_packet->held[12 + axis];
          estimator.magneticFieldBodyFlu_T[axis] = mag_packet->held[21 + axis];
          estimator.integratedGyroscopeBodyFlu_rad[axis] =
              flow_packet->held[1 + axis] * .01f;
        }
        if (flow_packet->valid) {
          estimator.integratedLineOfSight_rad[0] =
              -flow_packet->held[17] / estimator.groundDistance_m * .01f -
              estimator.integratedGyroscopeBodyFlu_rad[0];
          estimator.integratedLineOfSight_rad[1] =
              flow_packet->held[16] / estimator.groundDistance_m * .01f -
              estimator.integratedGyroscopeBodyFlu_rad[1];
        }
      }
      if (flow_exposure.input && estimator.opticalFlow_valid &&
          !flow_exposure_apply(transport.queues[FLOW_SOURCE].held[15],
                               estimator.opticalFlow_fresh)) {
        fprintf(stderr, "Invalid or mistimed flow exposure at %.9f\n", t);
        return 1;
      }
#ifdef COMPARE_HORIZON
      horizon_receive(flow_exposure.input != NULL);
#else
#ifndef COMPARE_UKF
      estimator.vehicleAtRest = estimator.imu_timestamp_s < stationary_until_s;
#endif
      const uint64_t filter_start = timing_now();
      NavigationEstimator_dostep(&estimator);
      timing_record(&filter_timing, filter_start);
#endif
    }
#ifdef COMPARE_HORIZON
    if (!horizon_tick(gyro, accel, imu_tick % 8 == 0, stationary_until_s)) {
      fprintf(stderr, "horizon failed at %.6f\n", t);
      return 1;
    }
#endif
    if (imu_tick % 8 == 0) {
#ifdef COMPARE_HORIZON
      const float *position = predictor.positionWorldEnu_m;
      const float *velocity = predictor.velocityWorldEnu_m_s;
      const float *quaternion = predictor.quaternionWorldBody;
#else
      const float *position = estimator.estimate_positionWorldEnu_m;
      const float *velocity = estimator.estimate_velocityWorldEnu_m_s;
      const float *quaternion = estimator.estimate_quaternionWorldBody;
#endif
      fprintf(out, "%.6f", t);
      for (int j = 0; j < 3; j++)
        fprintf(out, ",%.9g", position[j]);
      for (int j = 0; j < 3; j++)
        fprintf(out, ",%.9g", velocity[j]);
      for (int j = 0; j < 4; j++)
        fprintf(out, ",%.9g", quaternion[j]);
      fprintf(out, ",%d,%d,%d,%d,%d,%d,%u,%d", estimator.estimate_valid,
              estimator.estimate_valid,
              estimator.status_gpsPositionCorrectionAccepted,
              estimator.status_opticalFlowCorrectionAccepted,
              estimator.status_magnetometerCorrectionAccepted,
              estimator.status_barometerCorrectionAccepted,
              estimator.rumoca_galec_error_signal_status,
              estimator.status_predictionAccepted);
#ifdef COMPARE_HORIZON
      fprintf(out, ",%.9g", predictor.timestamp_s);
      for (unsigned axis = 0; axis < 3; ++axis)
        fprintf(out, ",%.9g", estimator.estimate_positionWorldEnu_m[axis]);
      for (unsigned axis = 0; axis < 3; ++axis)
        fprintf(out, ",%.9g", estimator.estimate_velocityWorldEnu_m_s[axis]);
      for (unsigned axis = 0; axis < 4; ++axis)
        fprintf(out, ",%.9g", estimator.estimate_quaternionWorldBody[axis]);
#endif
      fputc('\n', out);
      if (diagnostics) {
        fprintf(diagnostics, "%.6f", t);
#ifdef COMPARE_HORIZON
        fprintf(diagnostics, ",%.9g,%d", predictor.timestamp_s,
                predictor.horizonReady);
#endif
        for (int axis = 0; axis < 3; ++axis)
          fprintf(diagnostics, ",%.9g",
                  estimator.gyroscopeBiasBodyFlu_rad_s[axis]);
        for (int axis = 0; axis < 3; ++axis)
          fprintf(diagnostics, ",%.9g",
                  estimator.accelerometerBiasBodyFlu_m_s2[axis]);
        for (int row = 0; row < 15; ++row)
          for (int column = 0; column < 15; ++column)
            fprintf(diagnostics, ",%.9g",
                    estimator.errorCovariance[row][column]);
#ifdef SQUARE_ROOT_COVARIANCE
        for (int row = 0; row < 15; ++row)
          for (int column = 0; column < 15; ++column)
            fprintf(diagnostics, ",%.9g",
                    estimator.errorCovarianceRoot[row][column]);
#endif
        fputc('\n', diagnostics);
      }
    }
    imu_tick++;
  }
  if (timing_path) {
    FILE *profile =
        strcmp(timing_path, "-") == 0 ? stderr : fopen(timing_path, "w");
    if (!profile)
      return 1;
    fprintf(profile, "{\"clock\":\"CLOCK_THREAD_CPUTIME_ID\",\"imu_ticks\":%u,",
            imu_tick);
    if (stationary_until_s > 0)
      fprintf(profile, "\"stationary_until_s\":%.9g,", stationary_until_s);
    if (flow_exposure.input)
      fprintf(profile, "\"flow_exposure_packets\":%u,",
              flow_exposure.delivered);
    timing_write(profile, "filter", &filter_timing);
    fputc(',', profile);
    timing_write(profile, "preintegration", &preintegration_timing);
    fputc(',', profile);
    timing_write(profile, "predictor", &predictor_timing);
    fputc(',', profile);
    timing_write(profile, "queues", &queue_timing);
#ifdef COMPARE_HORIZON
    fprintf(profile,
            ",\"state_bytes\":%zu,\"buffer_bytes\":%zu,\"fusion_horizon_s\":%."
            "9g,\"worst_residual_age_s\":%.9g,\"late\":%d,\"overflow\":%d,"
            "\"stale\":%d",
            sizeof(estimator) + sizeof(predictor) + sizeof(aiding),
            sizeof(predictor.ring) + sizeof(aiding.gpsQueue) +
                sizeof(aiding.mocapQueue) + sizeof(aiding.magnetometerQueue) +
                sizeof(aiding.barometerQueue) + sizeof(aiding.opticalFlowQueue),
            predictor.fusionHorizon_s, aiding.worstDeliveredAge_s,
            aiding.refusedLateCount, aiding.refusedOverflowCount,
            aiding.droppedStaleCount);
#else
    fprintf(profile, ",\"state_bytes\":%zu,\"buffer_bytes\":0",
            sizeof(estimator) + sizeof(preintegrator));
#endif
    fprintf(profile,
            ",\"transport_bytes\":%zu,\"transport_delivered\":[%llu,%llu,%llu,%"
            "llu]",
            sizeof(transport),
            (unsigned long long)transport.queues[0].delivered,
            (unsigned long long)transport.queues[1].delivered,
            (unsigned long long)transport.queues[2].delivered,
            (unsigned long long)transport.queues[3].delivered);
    fprintf(profile,
            ",\"transport_digest\":[\"%016llx\",\"%016llx\",\"%016llx\",\"%"
            "016llx\"]}\n",
            (unsigned long long)transport.queues[0].delivered_digest,
            (unsigned long long)transport.queues[1].delivered_digest,
            (unsigned long long)transport.queues[2].delivered_digest,
            (unsigned long long)transport.queues[3].delivered_digest);
    fclose(profile);
  }
  if (diagnostics)
    fclose(diagnostics);
  if (flow_exposure.input)
    fclose(flow_exposure.input);
  fclose(out);
  fclose(imu);
  fclose(sensors);
  return 0;
}
