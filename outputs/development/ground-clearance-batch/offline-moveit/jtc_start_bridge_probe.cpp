// Native JTC initialization only: no node, master, simulator, or actions.
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <trajectory_interface/trajectory_interface.h>
#include <iostream>
#include <iomanip>

int main() {
  using Spline = trajectory_interface::QuinticSplineSegment<double>;
  using Segment = joint_trajectory_controller::JointTrajectorySegment<Spline>;
  using Trajectory = std::vector<std::vector<Segment>>;
  Segment::State hold(1);
  hold.position[0] = .4;
  Trajectory current(1);
  current[0].emplace_back(99., hold, 110., hold);
  trajectory_msgs::JointTrajectory msg;
  msg.joint_names = {"j1"};
  trajectory_msgs::JointTrajectoryPoint first, second;
  first.positions = {0}; first.velocities = {0}; first.accelerations = {0};
  second.positions = {1}; second.velocities = {0}; second.accelerations = {0};
  second.time_from_start = ros::Duration(1.);
  msg.points = {first, second};
  joint_trajectory_controller::InitJointTrajectoryOptions<Trajectory> options;
  options.current_trajectory = &current;
  auto initialized = joint_trajectory_controller::initJointTrajectory<Trajectory>(msg, ros::Time(100.), options);
  std::cout << std::setprecision(17);
  for (double t : {100., 100.1, 100.5, 101.}) {
    Segment::State actual;
    trajectory_interface::sample(initialized[0], t, actual);
    std::cout << "NATIVE_INITIALIZED t=" << t-100. << " position=" << actual.position[0] << '\n';
  }
  Spline::State raw_first, raw_second, raw_sample;
  raw_first.position = first.positions; raw_first.velocity = first.velocities; raw_first.acceleration = first.accelerations;
  raw_second.position = second.positions; raw_second.velocity = second.velocities; raw_second.acceleration = second.accelerations;
  Spline raw(0, raw_first, 1, raw_second);
  raw.sample(.5, raw_sample);
  std::cout << "RAW_MESSAGE_SPLINE t=.5 position=" << raw_sample.position[0] << '\n';
  return 0;
}
