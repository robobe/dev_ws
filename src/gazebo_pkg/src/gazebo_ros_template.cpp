#include <gazebo/physics/Model.hh>
#include <gazebo_pkg/gazebo_ros_template.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace gazebo_plugins
{
    class GazeboRosTemplatePrivate
    {
    public:
        gazebo::event::ConnectionPtr update_connection_;
        /// Node for ROS communication.
        gazebo_ros::Node::SharedPtr ros_node_;
    };

    GazeboRosTemplate::GazeboRosTemplate()
        : impl_(std::make_unique<GazeboRosTemplatePrivate>())
    {
    }

    GazeboRosTemplate::~GazeboRosTemplate()
    {
    }

    void GazeboRosTemplate::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr _sdf)
    {
        impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
        
        gzdbg << "debug" << std::endl;
        gzmsg << "message" << std::endl;
        gzwarn << "warning" << std::endl;
        gzerr << "error" << std::endl;
        
        RCLCPP_INFO(impl_->ros_node_->get_logger(), model->GetName().c_str());
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "hello ros2");
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "hello from gazebo plugin");
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "ROS2 Info");
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "ROS2 Warning");
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "ROS2 Error");
        
        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboRosTemplate::OnUpdate, this));
    }

    void GazeboRosTemplate::OnUpdate()
    {
        // Do something every simulation iteration
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboRosTemplate)
} // namespace gazebo_plugins