#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_plugins
{
// Forward declaration of private data class.
class GazeboRosTemplatePrivate;

class GazeboRosTemplate : public gazebo::ModelPlugin
{
public:
  GazeboRosTemplate();
  virtual ~GazeboRosTemplate();
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr _sdf) override;

protected:
  virtual void OnUpdate();

private:
  std::unique_ptr<GazeboRosTemplatePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_