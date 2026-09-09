// Offline native model probe. Input: length-prefixed URDF, SRDF, scene messages.
#include <moveit/planning_scene/planning_scene.h>
#include <urdf_parser/urdf_parser.h>
#include <ros/serialization.h>
#include <iostream>
#include <iomanip>
#include <vector>

std::vector<uint8_t> packet() {
  uint32_t n = 0;
  std::cin.read(reinterpret_cast<char*>(&n), 4);
  if (!std::cin || n > 100000000) throw std::runtime_error("invalid packet");
  std::vector<uint8_t> data(n);
  std::cin.read(reinterpret_cast<char*>(data.data()), n);
  if (!std::cin) throw std::runtime_error("short packet");
  return data;
}
int main() {
  std::cout << std::setprecision(17);
  auto u = packet(), s = packet();
  auto robot = urdf::parseURDF(std::string(u.begin(), u.end()));
  auto semantic = std::make_shared<srdf::Model>();
  semantic->initString(*robot, std::string(s.begin(), s.end()));
  planning_scene::PlanningScene scene(robot, semantic);
  std::cout << "MODEL collision_links=" << scene.getRobotModel()->getLinkModelsWithCollisionGeometry().size() << '\n';
  for (const auto* link : scene.getRobotModel()->getJointModelGroup("manipulator")->getUpdatedLinkModels())
    if (link->getName().find("finger") != std::string::npos || link->getName().find("d435") != std::string::npos)
      std::cout << "MANIPULATOR_UPDATED " << link->getName() << '\n';
  while (std::cin.peek() != EOF) {
    auto data = packet();
    ros::serialization::IStream stream(data.data(), data.size());
    moveit_msgs::PlanningScene message;
    ros::serialization::deserialize(stream, message);
    if (!scene.setPlanningSceneDiffMsg(message)) throw std::runtime_error("scene apply failed");
    if (scene.getWorld()->hasObject("chassis_clearance_probe")) {
      for (const auto* link : scene.getRobotModel()->getLinkModels()) {
        const auto* ancestor = link;
        bool rigid = true;
        while (ancestor && ancestor->getParentLinkModel()) {
          if (ancestor->getParentJointModel()->getType() != moveit::core::JointModel::FIXED) { rigid = false; break; }
          ancestor = ancestor->getParentLinkModel();
        }
        scene.getAllowedCollisionMatrixNonConst().setEntry("chassis_clearance_probe", link->getName(), rigid);
        if (rigid && !link->getShapes().empty()) std::cout << "GUARD_RIGID_COLLISION_LINK " << link->getName() << '\n';
      }
      scene.getAllowedCollisionMatrixNonConst().setEntry("chassis_clearance_probe", "perceived_pick_target", false);
    }
    auto& state = scene.getCurrentStateNonConst();
    state.update();
    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult cres;
    creq.contacts = true; creq.max_contacts = 100;
    scene.checkCollision(creq, cres);
    std::cout << "STATE " << message.name << " collision=" << cres.collision << '\n';
    for (const auto& contact : cres.contacts)
      std::cout << "CONTACT " << contact.first.first << " " << contact.first.second << '\n';
    creq.group_name = "manipulator";
    cres.clear();
    scene.checkCollision(creq, cres);
    std::cout << "MANIPULATOR_GROUP collision=" << cres.collision << '\n';
    for (bool self : {true, false}) {
      collision_detection::DistanceRequest req;
      collision_detection::DistanceResult result;
      req.type = collision_detection::DistanceRequestType::SINGLE;
      req.acm = &scene.getAllowedCollisionMatrix();
      req.enable_nearest_points = true;
      req.distance_threshold = .020;
      if (self) scene.getCollisionEnvUnpadded()->distanceSelf(req, result, state);
      else scene.getCollisionEnv()->distanceRobot(req, result, state);
      std::cout << (self ? "SELF " : "WORLD ") << "min=" << result.minimum_distance.distance
                << " collision=" << result.collision << " pairs=" << result.distances.size() << '\n';
      for (const auto& pair : result.distances)
        for (const auto& d : pair.second)
          std::cout << "PAIR " << pair.first.first << " " << pair.first.second << " distance=" << d.distance << '\n';
    }
  }
}
