#include <gazebo/physics/Model.hh>
#include <gazebo_pkg/gazebo_ros_contact.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace gazebo
{
    class GazeboRosContactPrivate
    {
    public:
        gazebo::event::ConnectionPtr update_connection_;
        gazebo_ros::Node::SharedPtr ros_node_;
        sensors::ContactSensorPtr parentSensor_;
        std::string ground_ = "ground_plane";
    };

    GazeboRosContact::GazeboRosContact() : impl_(std::make_unique<GazeboRosContactPrivate>())
    {
    }
    GazeboRosContact::~GazeboRosContact() {}

    void GazeboRosContact::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
        // Get the parent sensor.
        this->impl_->parentSensor_ =
            std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

        // // Make sure the parent sensor is valid.
        if (!this->impl_->parentSensor_)
        {
            gzerr << "ContactPlugin requires a ContactSensor.\n";
            return;
        }

        // // Connect to the sensor update event.
        this->impl_->update_connection_ = this->impl_->parentSensor_->ConnectUpdated(
            std::bind(&GazeboRosContact::OnUpdate, this));

        this->impl_->parentSensor_->SetActive(true);
    }

    void GazeboRosContact::OnUpdate()
    {
        msgs::Contacts contacts;
        contacts = this->impl_->parentSensor_->Contacts();

        for (int i = 0; i < contacts.contact_size(); i++)
        {
            auto c2 = contacts.contact(i).collision2()->name();
            if (strstr(c2.c_str(), this->impl_->ground_.c_str()))
            {
                continue;
            }
            gzmsg << "Collision between ["
                  << contacts.contact(i).collision1() << "]"
                  << "[" << contacts.contact(i).collision2() << "]"
                  << std::endl;
        }
    }
    GZ_REGISTER_SENSOR_PLUGIN(GazeboRosContact)
}