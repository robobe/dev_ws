#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_CONTACT_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_CONTACT_HPP_
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <memory>

namespace gazebo
{
  // Forward declaration of private data class.
  class GazeboRosContactPrivate;

  class GazeboRosContact : public SensorPlugin
  {
  public:
    GazeboRosContact();
    virtual ~GazeboRosContact();
    void Load(sensors::SensorPtr model, sdf::ElementPtr _sdf) override;

  protected:
    virtual void OnUpdate();

  private:
    std::unique_ptr<GazeboRosContactPrivate> impl_;
    
  };
} // namespace gazebo_plugins

#endif