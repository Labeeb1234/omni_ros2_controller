#ifndef OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_
#define OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <utility>


#include "controller_interface/controller_interface.hpp"
//#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.h"


// optional but recommended
//#include "omni_drive_controller/visibility_control.h"

namespace omni_drive_controller
{
class OmniDriveController : public controller_interface::ControllerInterface
{
  public:
    OmniDriveController();
      controller_interface::CallbackReturn on_init() override;
      controller_interface::InterfaceConfiguration command_interface_configuration() const override;
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;
      controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
      controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
      controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
      controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
      controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
      controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    ~OmniDriveController(); 

    protected:
      struct RimHandle
      {
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
        

      };

      // vectors for rim names, handles and joints
      std::vector<std::string> rim_names_;
      std::vector<RimHandle> registered_rim_handles_;
      std::vector<std::string> rim_joint_names_;

      // robot physical parameters variables declared
      double rim_radius_;
      double wheel_separation_;

      bool subscriber_is_active_ = false;
      // declaring variables/pointers for subscriptions
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_sub_;
      realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::TwistStamped>> vel_cmd_ptr_;



};

}  

#endif 