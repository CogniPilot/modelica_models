#ifndef ESTIMATOR_COMPARISON_FLOW_PACKETS_H
#define ESTIMATOR_COMPARISON_FLOW_PACKETS_H

typedef struct {
  FILE *input;
  double values[18];
  unsigned delivered;
} FlowExposure;

static FlowExposure flow_exposure;

static bool flow_exposure_open(const char *path) {
  const char *header =
      "t_s,vx_flu_m_s,vy_flu_m_s,dist_m,quality,valid,exposure_midpoint_s,"
      "integration_time_s,integrated_los_x_rad,integrated_los_y_rad,"
      "integrated_gyro_x_rad,integrated_gyro_y_rad,integrated_gyro_z_rad,"
      "los_variance_x_rad2,los_variance_y_rad2,gyro_variance_x_rad2,"
      "gyro_variance_y_rad2,gyro_variance_z_rad2\n";
  flow_exposure.input = fopen(path, "r");
  char line[1024];
  return flow_exposure.input &&
         fgets(line, sizeof(line), flow_exposure.input) &&
         !strcmp(line, header);
}

static bool flow_exposure_apply(double exposure_end, bool fresh) {
  if (fresh) {
    char line[2048];
    if (!fgets(line, sizeof(line), flow_exposure.input))
      return false;
    char *cursor = line;
    for (unsigned column = 0; column < 18; ++column) {
      char *end;
      flow_exposure.values[column] = strtod(cursor, &end);
      if (end == cursor || !isfinite(flow_exposure.values[column]) ||
          (column < 17 && *end != ','))
        return false;
      cursor = end + (column < 17);
    }
    ++flow_exposure.delivered;
  }
  const double *packet = flow_exposure.values;
  if (fabs(packet[0] - exposure_end) > 1e-8 || packet[7] <= 0 ||
      fabs(packet[6] - (packet[0] - packet[7] / 2)) > 1e-8 || packet[4] < 0 ||
      packet[4] > 1 || packet[5] != 1 || packet[3] <= 0)
    return false;
  for (unsigned column = 13; column < 18; ++column)
    if (packet[column] <= 0)
      return false;
  estimator.opticalFlow_timestamp_s = packet[6];
  estimator.opticalFlow_integrationTime_s = packet[7];
  estimator.groundDistance_m = packet[3];
  estimator.quality = packet[4];
  for (unsigned axis = 0; axis < 2; ++axis) {
    estimator.integratedLineOfSight_rad[axis] = packet[8 + axis];
    estimator.integratedLineOfSightCovariance_rad2[axis][axis] =
        packet[13 + axis];
  }
  for (unsigned axis = 0; axis < 3; ++axis) {
    estimator.integratedGyroscopeBodyFlu_rad[axis] = packet[10 + axis];
    estimator.integratedGyroscopeCovariance_rad2[axis][axis] =
        packet[15 + axis];
  }
  return true;
}

#endif
