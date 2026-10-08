#ifndef ESTIMATOR_COMPARISON_HORIZON_H
#define ESTIMATOR_COMPARISON_HORIZON_H

#include "Estimation_FusionHorizon_AidingBuffer.h"
#include "Estimation_FusionHorizon_OutputPredictor.h"

static OutputPredictorState predictor;
static AidingBufferState aiding;
static int32_t horizon_correction_count;

#define TRANSFER(destination, destination_field, source, source_field)         \
  memcpy(&(destination).destination_field, &(source).source_field,             \
         sizeof((destination).destination_field))

static void horizon_startup(bool first_order_hold, const double delays[4],
                            double jitter) {
  OutputPredictor_startup(&predictor);
  AidingBuffer_startup(&aiding);
  predictor.useFirstOrderHold = first_order_hold;
  predictor.foldBudget_hz = 10000;
  predictor.correctionRateBudget_hz = 100;
  aiding.maximumSourceDelay_s = 0;
  for (unsigned source = 0; source < SOURCE_COUNT; ++source)
    aiding.maximumSourceDelay_s =
        fmaxf(aiding.maximumSourceDelay_s, delays[source]);
  aiding.horizonJitterMargin_s = jitter + .01f;
  estimator.maximumAidingDelay_s = .01f;
  TRANSFER(aiding, gps_positionCovarianceWorld_m2, estimator,
           gps_positionCovarianceWorld_m2);
  TRANSFER(aiding, gps_velocityCovarianceWorld_m2_s2, estimator,
           velocityCovarianceWorld_m2_s2);
  TRANSFER(aiding, magnetometer_covarianceBody_T2, estimator,
           covarianceBody_T2);
  TRANSFER(aiding, barometer_variance_m2, estimator, variance_m2);
  TRANSFER(aiding, opticalFlow_integratedLineOfSightCovariance_rad2, estimator,
           integratedLineOfSightCovariance_rad2);
  TRANSFER(aiding, opticalFlow_integratedGyroscopeCovariance_rad2, estimator,
           integratedGyroscopeCovariance_rad2);
  TRANSFER(aiding, opticalFlow_integrationTime_s, estimator,
           opticalFlow_integrationTime_s);
  TRANSFER(aiding, opticalFlow_groundDistanceVariance_m2, estimator,
           groundDistanceVariance_m2);
  TRANSFER(aiding, opticalFlow_quality, estimator, quality);
}

static void horizon_receive(bool variable_flow_configuration) {
  if (variable_flow_configuration) {
    TRANSFER(aiding, opticalFlow_integrationTime_s, estimator,
             opticalFlow_integrationTime_s);
    TRANSFER(aiding, opticalFlow_integratedLineOfSightCovariance_rad2,
             estimator, integratedLineOfSightCovariance_rad2);
    TRANSFER(aiding, opticalFlow_integratedGyroscopeCovariance_rad2, estimator,
             integratedGyroscopeCovariance_rad2);
  }
  TRANSFER(aiding, gps_valid, estimator, gps_valid);
  TRANSFER(aiding, gps_fresh, estimator, gps_fresh);
  TRANSFER(aiding, gps_positionValid, estimator, positionValid);
  TRANSFER(aiding, gps_velocityValid, estimator, velocityValid);
  TRANSFER(aiding, gps_timestamp_s, estimator, gps_timestamp_s);
  TRANSFER(aiding, gps_positionWorldEnu_m, estimator, gps_positionWorldEnu_m);
  TRANSFER(aiding, gps_velocityWorldEnu_m_s, estimator,
           gps_velocityWorldEnu_m_s);
  TRANSFER(aiding, magnetometer_valid, estimator, magnetometer_valid);
  TRANSFER(aiding, magnetometer_fresh, estimator, magnetometer_fresh);
  TRANSFER(aiding, magnetometer_timestamp_s, estimator,
           magnetometer_timestamp_s);
  TRANSFER(aiding, magnetometer_magneticFieldBodyFlu_T, estimator,
           magneticFieldBodyFlu_T);
  TRANSFER(aiding, barometer_valid, estimator, barometer_valid);
  TRANSFER(aiding, barometer_fresh, estimator, barometer_fresh);
  TRANSFER(aiding, barometer_timestamp_s, estimator, barometer_timestamp_s);
  TRANSFER(aiding, barometer_altitudeWorldEnu_m, estimator, altitudeWorldEnu_m);
  TRANSFER(aiding, opticalFlow_valid, estimator, opticalFlow_valid);
  TRANSFER(aiding, opticalFlow_fresh, estimator, opticalFlow_fresh);
  TRANSFER(aiding, opticalFlow_timestamp_s, estimator, opticalFlow_timestamp_s);
  TRANSFER(aiding, opticalFlow_integratedLineOfSight_rad, estimator,
           integratedLineOfSight_rad);
  TRANSFER(aiding, opticalFlow_integratedGyroscopeBodyFlu_rad, estimator,
           integratedGyroscopeBodyFlu_rad);
  TRANSFER(aiding, opticalFlow_groundDistance_m, estimator, groundDistance_m);
}

static bool horizon_tick(const float gyro[3], const float accel[3],
                         bool arrival_tick, double stationary_until_s) {
  predictor.horizonStateValid = estimator.estimate_valid;
  predictor.horizonStateShifted =
      estimator.status_acceptedCorrectionCount != horizon_correction_count;
  horizon_correction_count = estimator.status_acceptedCorrectionCount;
  copy3(predictor.angularVelocityMeasuredBodyFlu_rad_s, gyro);
  copy3(predictor.specificForceMeasuredBodyFlu_m_s2, accel);
  TRANSFER(predictor, horizonPositionWorldEnu_m, estimator,
           estimate_positionWorldEnu_m);
  TRANSFER(predictor, horizonVelocityWorldEnu_m_s, estimator,
           estimate_velocityWorldEnu_m_s);
  TRANSFER(predictor, horizonQuaternionWorldBody, estimator,
           estimate_quaternionWorldBody);
  TRANSFER(predictor, horizonGyroscopeBiasBodyFlu_rad_s, estimator,
           gyroscopeBiasBodyFlu_rad_s);
  TRANSFER(predictor, horizonAccelerometerBiasBodyFlu_m_s2, estimator,
           accelerometerBiasBodyFlu_m_s2);
  const uint64_t predictor_start = timing_now();
  OutputPredictor_dostep(&predictor);
  timing_record(&predictor_timing, predictor_start);
  if (predictor.rumoca_galec_error_signal_status) {
    fprintf(stderr, "predictor status=%u\n",
            predictor.rumoca_galec_error_signal_status);
    return false;
  }
  aiding.horizonValid = predictor.horizonReady;
  aiding.horizonReleased = predictor.valid;
  aiding.horizonEpoch_s = predictor.timestamp_s;
  if (arrival_tick || predictor.valid || aiding.reset) {
    const uint64_t queue_start = timing_now();
    AidingBuffer_dostep(&aiding);
    timing_record(&queue_timing, queue_start);
  }
  if (aiding.rumoca_galec_error_signal_status || aiding.deliveryOutOfOrder ||
      aiding.deliveryAfterHorizon)
    return false;
  if (!predictor.valid)
    return true;
  TRANSFER(estimator, imu_valid, predictor, valid);
  TRANSFER(estimator, imu_fresh, predictor, fresh);
  TRANSFER(estimator, imu_timestamp_s, predictor, timestamp_s);
  TRANSFER(estimator, imu_angularVelocityBodyFlu_rad_s, predictor,
           horizonPacket_angularVelocityBodyFlu_rad_s);
  TRANSFER(estimator, specificForceBodyFlu_m_s2, predictor,
           specificForceBodyFlu_m_s2);
  TRANSFER(estimator, imu_integrationTime_s, predictor, integrationTime_s);
  TRANSFER(estimator, deltaAngleBodyFlu_rad, predictor, deltaAngleBodyFlu_rad);
  TRANSFER(estimator, deltaVelocityBodyFlu_m_s, predictor,
           deltaVelocityBodyFlu_m_s);
  TRANSFER(estimator, deltaPositionBodyFlu_m, predictor,
           deltaPositionBodyFlu_m);
  TRANSFER(estimator, deltaQuaternionBodyFlu, predictor,
           deltaQuaternionBodyFlu);
  TRANSFER(estimator, gyroscopeBiasLinearizationBodyFlu_rad_s, predictor,
           gyroscopeBiasLinearizationBodyFlu_rad_s);
  TRANSFER(estimator, accelerometerBiasLinearizationBodyFlu_m_s2, predictor,
           accelerometerBiasLinearizationBodyFlu_m_s2);
  TRANSFER(estimator, deltaRotationGyroscopeBiasJacobian_s, predictor,
           deltaRotationGyroscopeBiasJacobian_s);
  TRANSFER(estimator, deltaVelocityGyroscopeBiasJacobian_m, predictor,
           deltaVelocityGyroscopeBiasJacobian_m);
  TRANSFER(estimator, deltaVelocityAccelerometerBiasJacobian_s, predictor,
           deltaVelocityAccelerometerBiasJacobian_s);
  TRANSFER(estimator, deltaPositionGyroscopeBiasJacobian_m_s, predictor,
           deltaPositionGyroscopeBiasJacobian_m_s);
  TRANSFER(estimator, deltaPositionAccelerometerBiasJacobian_s2, predictor,
           deltaPositionAccelerometerBiasJacobian_s2);
  TRANSFER(estimator, gps_valid, aiding, gpsAtHorizon_valid);
  TRANSFER(estimator, gps_fresh, aiding, gpsAtHorizon_fresh);
  TRANSFER(estimator, positionValid, aiding, gpsAtHorizon_positionValid);
  TRANSFER(estimator, velocityValid, aiding, gpsAtHorizon_velocityValid);
  TRANSFER(estimator, gps_timestamp_s, aiding, gpsAtHorizon_timestamp_s);
  TRANSFER(estimator, gps_positionWorldEnu_m, aiding,
           gpsAtHorizon_positionWorldEnu_m);
  TRANSFER(estimator, gps_velocityWorldEnu_m_s, aiding,
           gpsAtHorizon_velocityWorldEnu_m_s);
  TRANSFER(estimator, gps_positionCovarianceWorld_m2, aiding,
           gpsAtHorizon_positionCovarianceWorld_m2);
  TRANSFER(estimator, velocityCovarianceWorld_m2_s2, aiding,
           gpsAtHorizon_velocityCovarianceWorld_m2_s2);
  TRANSFER(estimator, magnetometer_valid, aiding, magnetometerAtHorizon_valid);
  TRANSFER(estimator, magnetometer_fresh, aiding, magnetometerAtHorizon_fresh);
  TRANSFER(estimator, magnetometer_timestamp_s, aiding,
           magnetometerAtHorizon_timestamp_s);
  TRANSFER(estimator, magneticFieldBodyFlu_T, aiding,
           magnetometerAtHorizon_magneticFieldBodyFlu_T);
  TRANSFER(estimator, covarianceBody_T2, aiding,
           magnetometerAtHorizon_covarianceBody_T2);
  TRANSFER(estimator, barometer_valid, aiding, barometerAtHorizon_valid);
  TRANSFER(estimator, barometer_fresh, aiding, barometerAtHorizon_fresh);
  TRANSFER(estimator, barometer_timestamp_s, aiding,
           barometerAtHorizon_timestamp_s);
  TRANSFER(estimator, altitudeWorldEnu_m, aiding,
           barometerAtHorizon_altitudeWorldEnu_m);
  TRANSFER(estimator, variance_m2, aiding, barometerAtHorizon_variance_m2);
  TRANSFER(estimator, opticalFlow_valid, aiding, opticalFlowAtHorizon_valid);
  TRANSFER(estimator, opticalFlow_fresh, aiding, opticalFlowAtHorizon_fresh);
  TRANSFER(estimator, opticalFlow_timestamp_s, aiding,
           opticalFlowAtHorizon_timestamp_s);
  TRANSFER(estimator, integratedLineOfSight_rad, aiding,
           opticalFlowAtHorizon_integratedLineOfSight_rad);
  TRANSFER(estimator, integratedLineOfSightCovariance_rad2, aiding,
           opticalFlowAtHorizon_integratedLineOfSightCovariance_rad2);
  TRANSFER(estimator, integratedGyroscopeBodyFlu_rad, aiding,
           opticalFlowAtHorizon_integratedGyroscopeBodyFlu_rad);
  TRANSFER(estimator, integratedGyroscopeCovariance_rad2, aiding,
           opticalFlowAtHorizon_integratedGyroscopeCovariance_rad2);
  TRANSFER(estimator, opticalFlow_integrationTime_s, aiding,
           opticalFlowAtHorizon_integrationTime_s);
  TRANSFER(estimator, groundDistance_m, aiding,
           opticalFlowAtHorizon_groundDistance_m);
  TRANSFER(estimator, groundDistanceVariance_m2, aiding,
           opticalFlowAtHorizon_groundDistanceVariance_m2);
  TRANSFER(estimator, quality, aiding, opticalFlowAtHorizon_quality);
  estimator.vehicleAtRest = estimator.imu_timestamp_s < stationary_until_s;
  const uint64_t filter_start = timing_now();
  NavigationEstimator_dostep(&estimator);
  timing_record(&filter_timing, filter_start);
  return estimator.rumoca_galec_error_signal_status == 0;
}

#undef TRANSFER
#endif
