#include "Estimation_FusionHorizon_OutputPredictor.h"
#include <math.h>
#include <stdio.h>

static OutputPredictorState predictor;

int main(void) {
  OutputPredictor_startup(&predictor);
  predictor.specificForceMeasuredBodyFlu_m_s2[2] = 9.81f;
  unsigned releases = 0;
  double worst_epoch_error_s = 0;
  for (unsigned tick = 0; tick <= 96000; ++tick) {
    predictor.reset = tick == 16003;
    OutputPredictor_dostep(&predictor);
    if (predictor.rumoca_galec_error_signal_status) {
      fprintf(stderr, "Horizon status=%u at tick %u\n",
              predictor.rumoca_galec_error_signal_status, tick);
      return 1;
    }
    if (predictor.horizonReady) {
      double age_s = tick * .00125 - predictor.timestamp_s;
      double error_s = fabs(age_s - predictor.bufferedDeltaCount * .00125);
      worst_epoch_error_s = fmax(worst_epoch_error_s, error_s);
      if (error_s > 1e-5 || age_s < .2 - 1e-5 || age_s > .211) {
        fprintf(stderr, "Horizon epoch mismatch at tick %u: %.9g s\n", tick,
                error_s);
        return 1;
      }
      releases += predictor.valid;
    } else if (predictor.valid) {
      fprintf(stderr, "Released an IMU packet before the horizon was ready\n");
      return 1;
    }
  }
  if (releases < 11900)
    return 1;
  printf("120-second float32 horizon with mid-run reset passed: %u releases, "
         "%.9g s maximum epoch error\n",
         releases, worst_epoch_error_s);
  return 0;
}
