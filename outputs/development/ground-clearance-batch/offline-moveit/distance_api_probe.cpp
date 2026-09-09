// Offline diagnostic only: no ros::init, node, master, services, or simulator.
// Build with installed moveit_core pkg-config flags; no new dependencies.
#include <moveit/planning_scene/planning_scene.h>
#include <urdf_parser/urdf_parser.h>
#include <fcl/fcl.h>
#include <iostream>
#include <iomanip>
#include <string>

using collision_detection::DistanceRequest;
using collision_detection::DistanceResult;

void dump(const std::string& label, planning_scene::PlanningScene& scene,
          bool self, bool padded = false, double threshold = 1.0)
{
  auto& state = scene.getCurrentStateNonConst();
  state.update();
  DistanceRequest req;
  req.type = collision_detection::DistanceRequestType::SINGLE;
  req.enable_nearest_points = true;
  req.enable_signed_distance = false;
  req.acm = &scene.getAllowedCollisionMatrix();
  req.distance_threshold = threshold;
  DistanceResult res;
  auto env = padded ? scene.getCollisionEnv() : scene.getCollisionEnvUnpadded();
  if (self) env->distanceSelf(req, res, state);
  else env->distanceRobot(req, res, state);
  std::cout << label << " collision=" << res.collision
            << " minimum=" << res.minimum_distance.distance
            << " pairs=" << res.distances.size() << '\n';
  for (const auto& pair : res.distances)
    for (const auto& d : pair.second)
      std::cout << "PAIR " << pair.first.first << " " << pair.first.second
                << " distance=" << d.distance << " types=" << d.body_types[0]
                << "," << d.body_types[1] << '\n';
}

void collision(const std::string& label, planning_scene::PlanningScene& scene)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.contacts = true;
  req.max_contacts = 100;
  scene.checkCollision(req, res);
  std::cout << label << " collision=" << res.collision << '\n';
  for (const auto& pair : res.contacts)
    std::cout << "CONTACT " << pair.first.first << " " << pair.first.second << '\n';
}

int main()
{
  std::cout << std::setprecision(17);
  for (double separation : {.150405, .1508, .152, .155, .17, .2, .350405})
  for (auto solver : {fcl::GST_LIBCCD, fcl::GST_INDEP}) {
    auto first = std::make_shared<fcl::Boxd>(.2, .2, .2);
    auto second = std::make_shared<fcl::Boxd>(.1, .1, .1);
    fcl::Transform3d transform = fcl::Transform3d::Identity();
    transform.translation().x() = separation;
    fcl::CollisionObjectd a(first), b(second, transform);
    for (bool reverse : {false, true}) {
      fcl::DistanceRequestd req(true, false, 0, 0, 1e-6, solver);
      fcl::DistanceResultd res;
      fcl::distance(reverse ? &b : &a, reverse ? &a : &b, req, res);
      std::cout << "DIRECT_FCL solver=" << solver << " reverse=" << reverse
                << " distance=" << res.min_distance << " expected=" << separation-.15 << '\n';
    }
  }
  // Analytic face-gap family: every target vertex projects inside the larger
  // box face, so the exact Euclidean distance equals gap for all rotations.
  for (auto solver : {fcl::GST_LIBCCD, fcl::GST_INDEP}) {
    int count = 0, inaccurate = 0, missed_5mm_band = 0;
    double maximum_error = 0;
    for (bool real_dimensions : {false, true}) {
      fcl::Vector3d big = real_dimensions ? fcl::Vector3d(1.026335219, .782744936, .395154782)
                                        : fcl::Vector3d(.2, .2, .2);
      fcl::Vector3d small = real_dimensions ? fcl::Vector3d(.24, .053, .115)
                                          : fcl::Vector3d(.1, .1, .1);
      for (double gap : {.000405, .001, .003, .0049})
      for (double roll : {0., .05, .4, 1.3})
      for (double pitch : {0., .08, .7, 1.4})
      for (double yaw : {0., .0558, .6, 1.5}) {
        fcl::Transform3d transform = fcl::Transform3d::Identity();
        transform.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).matrix();
        transform.translation().x() = big.x()/2 + (transform.linear().row(0).cwiseAbs().array()*small.transpose().array()).sum()/2 + gap;
        auto first = std::make_shared<fcl::Boxd>(big);
        auto second = std::make_shared<fcl::Boxd>(small);
        fcl::CollisionObjectd a(first), b(second, transform);
        for (bool reverse : {false, true}) {
          fcl::DistanceRequestd req(true, false, 0, 0, 1e-6, solver);
          fcl::DistanceResultd res;
          fcl::distance(reverse ? &b : &a, reverse ? &a : &b, req, res);
          const double error = res.min_distance-gap;
          ++count;
          if (std::abs(error) > 1e-6) ++inaccurate;
          if (res.min_distance > .005) ++missed_5mm_band;
          maximum_error = std::max(maximum_error, error);
        }
      }
    }
    std::cout << "SMALL_BAND solver=" << solver << " count=" << count
              << " inaccurate_gt_1um=" << inaccurate << " missed_5mm_band=" << missed_5mm_band
              << " maximum_overestimate=" << maximum_error << '\n';
  }
  auto robot = urdf::parseURDF(R"(<robot name='distance_probe'>
    <link name='chassis'><collision><geometry><box size='.2 .2 .2'/></geometry></collision></link>
    <link name='hand'><collision><geometry><box size='.1 .1 .1'/></geometry></collision></link>
    <joint name='slide' type='prismatic'><parent link='chassis'/><child link='hand'/>
      <origin xyz='.150405 0 0'/><axis xyz='1 0 0'/><limit lower='-.1' upper='1' effort='1' velocity='1'/></joint>
    <link name='camera'><collision><geometry><box size='.02 .02 .02'/></geometry></collision></link>
    <joint name='camera_joint' type='fixed'><parent link='chassis'/><child link='camera'/><origin xyz='0 .14 0'/></joint>
  </robot>)");
  auto semantic = std::make_shared<srdf::Model>();
  semantic->initString(*robot, "<robot name='distance_probe'/>");
  planning_scene::PlanningScene scene(robot, semantic);
  scene.getCurrentStateNonConst().setToDefaultValues();
  dump("self_all", scene, true);
  dump("self_threshold_2mm", scene, true, false, .002);
  collision("nominal", scene);
  scene.getCollisionEnvNonConst()->setLinkPadding("hand", .005);
  dump("self_unpadded_after_padding", scene, true);
  dump("self_direct_padded_environment", scene, true, true);
  collision("planning_scene_after_padding", scene);
  scene.getCollisionEnvNonConst()->setLinkPadding("hand", 0.0);

  moveit_msgs::CollisionObject world;
  world.header.frame_id = "chassis";
  world.id = "perceived_target";
  world.operation = world.ADD;
  world.pose.orientation.w = 1;
  shape_msgs::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {.1, .1, .1};
  world.primitives.push_back(box);
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  pose.position.x = .350405;
  world.primitive_poses.push_back(pose);
  scene.processCollisionObjectMsg(world);
  dump("world_target", scene, false);
  scene.getCurrentStateNonConst().setVariablePosition("slide", .2);

  moveit_msgs::AttachedCollisionObject attached;
  attached.link_name = "hand";
  attached.touch_links = {"hand"};
  attached.object = world;
  attached.object.header.frame_id = "hand";
  attached.object.primitive_poses[0].position.x = -.199605;
  std::cout << "attach_success=" << scene.processAttachedCollisionObjectMsg(attached)
            << " still_in_world=" << scene.getWorld()->hasObject(world.id) << '\n';
  dump("attached_self_touch_exception", scene, true);
  world.id = "perceived_obstacle";
  world.primitive_poses[0].position.x = .1508;
  world.primitive_poses[0].position.y = .1009;
  scene.processCollisionObjectMsg(world);
  dump("attached_world", scene, false);
  scene.getAllowedCollisionMatrixNonConst().setEntry("perceived_target", "chassis", true);
  dump("attached_self_acm_exception", scene, true);
  scene.getAllowedCollisionMatrixNonConst().setEntry("perceived_target", "chassis", false);
  scene.getCurrentStateNonConst().setVariablePosition("slide", .198);
  dump("attached_collision", scene, true);
  collision("attached_collision_boolean", scene);
}
