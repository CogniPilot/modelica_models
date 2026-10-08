#include "Integrator.hpp"

#include <cstdio>
#include <initializer_list>

int main() {
  using matrix::Vector3f;
  const Vector3f initial_rate{0.3f, -0.2f, 0.1f};
  const Vector3f angular_acceleration{2.f, 3.f, -1.f};
  for (const float dt : {0.00125f, 0.0025f, 0.005f, 0.01f}) {
    for (const int intervals : {2, 4, 8}) {
      sensors::IntegratorConing integrator;
      integrator.set_reset_samples(1);
      integrator.put(initial_rate - angular_acceleration * dt, 0.f);
      integrator.put(initial_rate, dt);
      Vector3f published;
      uint32_t elapsed_us;
      if (!integrator.reset(published, elapsed_us)) return 1;
      Vector3f previous_increment =
          (initial_rate * 2.f - angular_acceleration * dt) * (0.5f * dt);
      Vector3f ardupilot_accumulator{};
      for (int interval = 1; interval <= intervals; ++interval) {
        const Vector3f previous_rate =
            initial_rate + angular_acceleration * ((interval - 1) * dt);
        const Vector3f current_rate =
            initial_rate + angular_acceleration * (interval * dt);
        const Vector3f increment = (previous_rate + current_rate) * (0.5f * dt);
        const Vector3f correction =
            ((ardupilot_accumulator + previous_increment * (1.f / 6.f)) %
             increment) * 0.5f;
        ardupilot_accumulator += increment + correction;
        previous_increment = increment;
        integrator.put(current_rate, dt);
      }
      if (!integrator.reset(published, elapsed_us)) return 2;
      std::printf("%.9g,%d", double(dt), intervals);
      for (unsigned axis = 0; axis < 3; ++axis)
        std::printf(",%.9g", double(published(axis)));
      for (unsigned axis = 0; axis < 3; ++axis)
        std::printf(",%.9g", double(ardupilot_accumulator(axis)));
      std::printf("\n");
    }
  }
}
