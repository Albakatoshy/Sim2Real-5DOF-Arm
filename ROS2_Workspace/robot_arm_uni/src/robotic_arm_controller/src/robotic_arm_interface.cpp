    #include "robotic_arm_controller/robotic_arm_interface.hpp"
    #include <hardware_interface/types/hardware_interface_type_values.hpp>
    #include <pluginlib/class_list_macros.hpp>
    #include <rclcpp/rclcpp.hpp>
    #include <cmath>

    namespace robotic_arm_controller
    {

    RoboticArmInterface::RoboticArmInterface()
    {
    }

    RoboticArmInterface::~RoboticArmInterface()
    {
        if (arduino_.IsOpen())
        {
            try {
                arduino_.Close();
            } 
            catch (...) {
                RCLCPP_ERROR(rclcpp::get_logger("RoboticArmInterface"), "Error closing serial port!");
            }
        }
    }

    CallbackReturn RoboticArmInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
    {
        if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        try {
            port_ = info_.hardware_parameters.at("port");
        } catch (const std::out_of_range& e) {
            RCLCPP_ERROR(rclcpp::get_logger("RoboticArmInterface"), "No 'port' provided!");
            return CallbackReturn::ERROR;
        }

        // resize() not reserve() — creates elements so size() > 0
        position_states_.resize(info_.joints.size(), 0.0);
        velocity_states_.resize(info_.joints.size(), 0.0);
        position_commands_.resize(info_.joints.size(), 0.0);
        prev_position_commands_.resize(info_.joints.size(), 0.0);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RoboticArmInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
        }

        return state_interfaces;
    }
    std::vector<hardware_interface::CommandInterface> RoboticArmInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // Tell ros2_control where to write the target joint commands
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn RoboticArmInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Activating hardware interface and connecting to Arduino...");

        try {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200); // Ensure this matches your Arduino code
            RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Waiting for Arduino to reset...");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Arduino ready!");
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("RoboticArmInterface"), "Failed to open serial port: %s", port_.c_str());
            return CallbackReturn::ERROR;
        }

        // Reset commands and states to 0 upon activation
        std::fill(position_states_.begin(), position_states_.end(), 0.0);
        std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
        std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
        std::fill(prev_position_commands_.begin(), prev_position_commands_.end(), 0.0);

        RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Hardware interface activated successfully!");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RoboticArmInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Deactivating hardware interface...");

        if (arduino_.IsOpen())
        {
            try {
                arduino_.Close();
            } catch (...) {
                RCLCPP_ERROR(rclcpp::get_logger("RoboticArmInterface"), "Error closing serial port!");
                return CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("RoboticArmInterface"), "Hardware interface deactivated successfully!");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RoboticArmInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // For a basic setup (open loop), we simply assume the arm reached the commanded position:
        position_states_ = position_commands_;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RoboticArmInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Check if the command has actually changed to avoid spamming the serial port
        if (position_commands_ == prev_position_commands_)
        {
            return hardware_interface::return_type::OK;
        }

        std::string msg = "";
        
        // Loop through all joints defined in your URDF (e.g., 6 joints)
        for (size_t i = 0; i < position_commands_.size(); i++)
        {
            // 1. Convert radians from MoveIt to degrees
            double degrees = position_commands_[i] * (180.0 / M_PI);
            
            // 2. Add 90 so that 0 radians (MoveIt Home) equals 90 degrees (Servo Home)
            int target_angle = std::round(degrees + 90.0);
            
            // 3. SAFETY LIMITS: Constrain the angle between 0 and 180
            // This prevents MoveIt from sending a command that strips your servo gears
            if (target_angle < 0) target_angle = 0;
            if (target_angle > 180) target_angle = 180;

            // 4. Add the formatted integer to the message string
            msg += std::to_string(target_angle);
            
            // Add a comma delimiter if it's not the last joint
            if (i < position_commands_.size() - 1) {
                msg += ",";
            }
        }
        
        // Terminate the message with a newline so the Arduino knows the message is complete
        msg += "\n"; 

        // Send the message over Serial
        try {
            arduino_.Write(msg);
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("RoboticArmInterface"), "Error writing to serial port!");
            return hardware_interface::return_type::ERROR;
        }

        // Update previous commands so we don't send duplicates
        prev_position_commands_ = position_commands_;

        return hardware_interface::return_type::OK;
    }

    } // namespace robotic_arm_controller

    // This macro makes the interface visible to the ros2_control plugin loader
    PLUGINLIB_EXPORT_CLASS(
        robotic_arm_controller::RoboticArmInterface,
        hardware_interface::SystemInterface 
    )