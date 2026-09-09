// Offline oracle: installed native segment/JTC implementation, not copied math.
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <trajectory_interface/trajectory_interface.h>
#include <ros/serialization.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <iomanip>

template<class Message> Message read() {
  uint32_t n;
  std::cin.read(reinterpret_cast<char*>(&n), 4);
  if (!std::cin || n > 10000000) throw std::runtime_error("invalid packet");
  std::vector<uint8_t> data(n);
  std::cin.read(reinterpret_cast<char*>(data.data()), n);
  ros::serialization::IStream stream(data.data(), n);
  Message result;
  ros::serialization::deserialize(stream, result);
  return result;
}
int main() {
  using Spline = trajectory_interface::QuinticSplineSegment<double>;
  using Segment = joint_trajectory_controller::JointTrajectorySegment<Spline>;
  using Trajectory = std::vector<std::vector<Segment>>;
  std::cout << std::setprecision(17);
  int index = 0;
  while (std::cin.peek() != EOF) {
    uint32_t held;
    std::cin.read(reinterpret_cast<char*>(&held), 4);
    auto message = read<trajectory_msgs::JointTrajectory>();
    auto hold = read<trajectory_msgs::JointTrajectoryPoint>();
    auto sample_times = read<std_msgs::Float64MultiArray>();
    std::cout << "CASE " << index++;
    if (held) {
      Trajectory current(message.joint_names.size());
      for (size_t j = 0; j < current.size(); ++j) {
        Segment::State state(1);
        state.position[0] = hold.positions[j];
        state.velocity[0] = hold.velocities[j];
        state.acceleration[0] = hold.accelerations[j];
        current[j].emplace_back(0., state, message.points.back().time_from_start.toSec()+2., state);
      }
      joint_trajectory_controller::InitJointTrajectoryOptions<Trajectory> options;
      options.current_trajectory = &current;
      auto trajectory = joint_trajectory_controller::initJointTrajectory<Trajectory>(message, ros::Time(1.), options);
      if (trajectory.size() != current.size()) throw std::runtime_error("native initialization rejected");
      for (double t : sample_times.data)
        for (const auto& joint : trajectory) {
          Segment::State state;
          trajectory_interface::sample(joint, 1.+t, state);
          std::cout << ' ' << state.position[0];
        }
    } else {
      std::vector<Segment> trajectory;
      for (size_t i = 1; i < message.points.size(); ++i) {
        Segment::State first(message.points[i-1]), second(message.points[i]);
        trajectory.emplace_back(message.points[i-1].time_from_start.toSec(), first,
                                message.points[i].time_from_start.toSec(), second);
      }
      for (double t : sample_times.data) {
        Segment::State state;
        trajectory_interface::sample(trajectory, t, state);
        for (double q : state.position) std::cout << ' ' << q;
      }
    }
    std::cout << '\n';
  }
}
