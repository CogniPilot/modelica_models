/* SPDX-License-Identifier: Apache-2.0 */
#include "Vehicles_Rdd2_NavigationEstimator.c"
#include <string.h>

int stationary_imu_update(const float nominal[16], const float prior[225],
                          const float packet[14], const float noise[36],
                          const float gravity[3], float output[485]) {
  NavigationEstimatorState state = {0};
  NavigationEstimator_startup(&state);
  float covariance[15][15], density[4][3][3];
  float root[15][15] = {{0}};
  float zero[3] = {0}, zero_matrix[3][3] = {{0}};
  memcpy(covariance, prior, sizeof(covariance));
  memcpy(density, noise, sizeof(density));
  predictStationary(&state, nominal, nominal + 3, nominal + 6, nominal + 10,
                    nominal + 13, covariance, false, root, packet[13],
                    density[0], density[1], density[2], density[3]);
  NavigationEstimatorScratch_predictStationary *predicted =
      &state.rumoca_galec_scratch.rumoca_galec_g1.predictStationary;
  memcpy(output, predicted->predicted_positionWorldEnu_m_2, 3 * sizeof(float));
  memcpy(output + 3, predicted->predicted_velocityWorldEnu_m_s_2,
         3 * sizeof(float));
  memcpy(output + 6, predicted->predicted_quaternionWorldBody_2,
         4 * sizeof(float));
  memcpy(output + 10, predicted->predicted_gyroscopeBiasBodyFlu_rad_s_2,
         3 * sizeof(float));
  memcpy(output + 13, predicted->predicted_accelerometerBiasBodyFlu_m_s2_2,
         3 * sizeof(float));
  memcpy(output + 16, predicted->predicted_covariance_2, 225 * sizeof(float));
  memcpy(covariance, output + 16, sizeof(covariance));
  correctStationaryImu(
      &state, output, output + 3, output + 6, output + 10, output + 13,
      covariance, false, root, true, true, 0, zero, zero, zero, packet + 4,
      zero, packet, packet[13], packet + 7, packet + 10, zero_matrix,
      zero_matrix, zero_matrix, zero_matrix, zero_matrix, gravity, density[0],
      density[1], density[2], density[3], 0, false);
  NavigationEstimatorScratch_correctStationaryImu *corrected =
      &state.rumoca_galec_scratch.rumoca_galec_g2.correctStationaryImu;
  memcpy(output + 241, corrected->corrected_positionWorldEnu_m,
         3 * sizeof(float));
  memcpy(output + 244, corrected->corrected_velocityWorldEnu_m_s,
         3 * sizeof(float));
  memcpy(output + 247, corrected->corrected_quaternionWorldBody,
         4 * sizeof(float));
  memcpy(output + 251, corrected->corrected_gyroscopeBiasBodyFlu_rad_s,
         3 * sizeof(float));
  memcpy(output + 254, corrected->corrected_accelerometerBiasBodyFlu_m_s2,
         3 * sizeof(float));
  memcpy(output + 257, corrected->corrected_covariance, 225 * sizeof(float));
  output[482] = corrected->normalizedInnovationSquared;
  output[483] = corrected->rejectionReason;
  output[484] = corrected->accepted;
  return state.rumoca_galec_error_signal_status;
}
