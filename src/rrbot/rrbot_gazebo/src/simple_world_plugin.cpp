#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
    // gazebo log
    gzmsg << "gazebo message" << std::endl;
    gzwarn << "gazebo warning" << std::endl;
    gzerr << "gazebo error" << std::endl;

    // ROS log
    RCLCPP_INFO(rclcpp::get_logger("world_plug")," ------ Hello World! ------ ");
    RCLCPP_WARN(rclcpp::get_logger("world_plug")," ------ warning! ------ ");
    RCLCPP_ERROR(rclcpp::get_logger("world_plug")," ------ Error! ------ ");
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
  }

};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}