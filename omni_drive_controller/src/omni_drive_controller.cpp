#include <chrono> 
#include <cmath>
#include <exception>
#include <utility>
#include <vector>
#include <queue>
#include <memory>


#include "controller_interface/controller_interface.hpp"
//#include "controller_interface/controller_interface_base.hpp"
//#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
//#include "tf2/LinearMath/Quaternion.h"

#include "omni_drive_controller/omni_drive_controller.hpp"

namespace omni_drive_controller
{
    using namespace std::chrono_literals;

    OmniDriveController::OmniDriveController()
        : controller_interface::ControllerInterface()
        , vel_cmd_sub_(nullptr)
        , vel_cmd_ptr_(nullptr) {}
    
    controller_interface::CallbackReturn OmniDriveController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }
    
    // controller command interface assignment
    controller_interface::InterfaceConfiguration OmniDriveController::command_interface_configuration() const
    {
       
        std::vector<std::string> conf_names;
        
        RCLCPP_INFO(get_node()->get_logger(), "Configure OmniDriveController\n");
        for(const auto &joint_name: rim_names_)
        {
            conf_names.push_back(joint_name+ "/" + hardware_interface::HW_IF_VELOCITY);

        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};

    }

    // controller state interface assignment
    controller_interface::InterfaceConfiguration OmniDriveController::state_interface_configuration() const
    {
        std::vector<std::string> state_interfaces_config;
        
        for(const auto &joint_name: rim_names_)
        {
            state_interfaces_config.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);

        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config};
    }

    // controller getting configured
    controller_interface::CallbackReturn OmniDriveController::on_configure(const rclcpp_lifecycle
    ::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Configure OmniDriveController\n");
        rim_names_ = get_node()->get_parameter("rim_names").as_string_array();
        if(rim_names_.size() != 4)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "What the fuck!\n");
            return controller_interface::CallbackReturn::ERROR;
        }
        if(rim_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "seriously!!\n");
            return controller_interface::CallbackReturn::ERROR;
        }
        rim_radius_ = get_node()->get_parameter("rim_radius").as_double();
        wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();

        // creating a subcriber to subcriber to Twist topic
        vel_cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", rclcpp::SystemDefaultsQoS(), [this](const geometry_msgs::msg::TwistStamped::SharedPtr twist)
        {
            vel_cmd_ptr_.writeFromNonRT(twist);
        });

        return controller_interface::CallbackReturn::SUCCESS;

    }

    controller_interface::CallbackReturn OmniDriveController::on_activate(const rclcpp_lifecycle::State &)
    {
        if(rim_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "myre!!\n");
            return controller_interface::CallbackReturn::ERROR;
        }

        registered_rim_handles_.reserve(rim_names_.size());
        {
            for(const auto &rim_name: rim_names_)
            {
                const auto state_handle = std::find_if(
                    state_interfaces_.cbegin(), state_interfaces_.cend(), [&rim_name](const auto &interface)
                    {
                        return interface.get_name() == rim_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;

                    });

                if(state_handle == state_interfaces_.cend())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Unable to get joint state handle forr %s", rim_name.c_str());
                    return controller_interface::CallbackReturn::ERROR;
                }

                const auto command_handle = std::find_if(
                    command_interfaces_.begin(), command_interfaces_.end(), [&rim_name](const auto &interface)
                    {
                        return interface.get_name() == rim_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                    });
                
                if(command_handle == command_interfaces_.end())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Unable to get joint command handle for %s", rim_name.c_str());
                    return controller_interface::CallbackReturn::ERROR;
                }

                registered_rim_handles_.emplace_back(RimHandle{std::ref(*state_handle), std::ref(*command_handle)});

                RCLCPP_INFO(get_node()->get_logger(),"Got command interface: %s", command_handle->get_name().c_str());
                RCLCPP_INFO(get_node()->get_logger(),"Got state interface: %s", state_handle->get_name().c_str());
       
            }

            subscriber_is_active_ = true;
            RCLCPP_INFO(get_node()->get_logger(),"Subcriber and publisher are active now\n");
            return controller_interface::CallbackReturn::SUCCESS;
        }

    }

    // stop sending commands to the subscriber by making it inactive
    controller_interface::CallbackReturn OmniDriveController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(), "Called on_deactivate\n");
        subscriber_is_active_ = false;
        return controller_interface::CallbackReturn::SUCCESS;

    }

    // cleanup phase
    controller_interface::CallbackReturn OmniDriveController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_cleanup\n");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // if errors occur what to do ?
    controller_interface::CallbackReturn OmniDriveController::on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_error\n");
        return controller_interface::CallbackReturn::SUCCESS; 
    }

    // shutdown phase of the controllers
    controller_interface::CallbackReturn OmniDriveController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_shutdown\n");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // our main function say controlling is looped here in return_type update function
    controller_interface::return_type OmniDriveController::update(const rclcpp::Time &, const rclcpp::Duration &)
    {
        
        RCLCPP_INFO(get_node()->get_logger(), "Entered the update phase: working?\n");
        // to get previous velocity command 
        auto vel_cmd = vel_cmd_ptr_.readFromRT();
        if(!vel_cmd || !(*vel_cmd))
        {
            return controller_interface::return_type::OK;
        }

        const auto twist = (*vel_cmd)->twist;
        // inverse kinematics of the 4-omni-wheel-robot
        double v_x_des = twist.linear.x;
        double v_y_des = twist.linear.y;
        double omega_des = twist.angular.z;

        std::vector<double> rim_velocity;
        rim_velocity[0] = -v_y_des + (wheel_separation_/2)*omega_des;
        rim_velocity[1] = v_x_des + (wheel_separation_/2)*omega_des;
        rim_velocity[2] = v_y_des + (wheel_separation_/2)*omega_des;
        rim_velocity[3] = -v_x_des + (wheel_separation_/2)*omega_des;

        return controller_interface::return_type::OK;
    }
    OmniDriveController::~OmniDriveController() {}

}

#include <class_loader/register_macro.hpp>

CLASS_LOADER_REGISTER_CLASS(
    omni_drive_controller::OmniDriveController, controller_interface::ControllerInterface)



