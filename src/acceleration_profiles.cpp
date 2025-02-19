#include "gazebo_ros_control/acceleration_profiles.hpp"
#include <cmath>

namespace gazebo_ros_control
{

double QuinticPolynomial(double t, double start, double end, double duration)
{
  if (t >= duration) {
    return end;
  }

  double normalized_time = t / duration;
  
  double h0 = 1.0 - 10.0 * pow(normalized_time, 3) + 15.0 * pow(normalized_time, 4) - 6.0 * pow(normalized_time, 5);
  double h1 = 10.0 * pow(normalized_time, 3) - 15.0 * pow(normalized_time, 4) + 6.0 * pow(normalized_time, 5);
  
  return start * h0 + end * h1;
}

double CubicPolynomial(double t, double start, double end, double duration)
{
  if (t >= duration) {
    return end;
  }

  double normalized_time = t / duration;
  
  double h0 = 2.0 * pow(normalized_time, 3) - 3.0 * pow(normalized_time, 2) + 1.0;
  double h1 = -2.0 * pow(normalized_time, 3) + 3.0 * pow(normalized_time, 2);
  
  return start * h0 + end * h1;
}

double SCurve(double t, double start, double end, double duration, double steepness)
{
  if (t >= duration) {
    return end;
  }

  double normalized_time = t / duration;
  
  double sigmoid = 1.0 / (1.0 + exp(-steepness * (normalized_time - 0.5)));
  
  return start + (end - start) * sigmoid;
}

double TrapezoidalProfile(double t, double start, double end, double duration, double max_acceleration)
{
  if (t >= duration) {
    return end;
  }

  double delta_pos = end - start;
  double max_velocity = std::sqrt(std::abs(delta_pos) * max_acceleration);
  
  // Calculate acceleration and deceleration times
  double accel_time = max_velocity / max_acceleration;
  double decel_time = duration - accel_time;
  
  if (t < accel_time) {
    // Acceleration phase
    return start + 0.5 * max_acceleration * t * t * (delta_pos > 0 ? 1 : -1);
  } else if (t < decel_time) {
    // Constant velocity phase
    double v = max_velocity * (delta_pos > 0 ? 1 : -1);
    double accel_dist = 0.5 * max_velocity * accel_time;
    return start + accel_dist + v * (t - accel_time);
  } else {
    // Deceleration phase
    double remaining_t = duration - t;
    return end - 0.5 * max_acceleration * remaining_t * remaining_t * (delta_pos > 0 ? 1 : -1);
  }
}

}  // namespace gazebo_ros_control 