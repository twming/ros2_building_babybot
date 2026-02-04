#include "babybot_firmware/babybot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace babybot_firmware
{
    BabybotInterface::BabybotInterface()
    {

    }

    BabybotInterface::~BabybotInterface()
    {
        /*
        Babybot Destructor - if arduino port is opened, then close it
        */
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("BabybotInterface"),"Something wen wrong while closing the connection with port "<< port_);
            }

        }
    }

    /*
    Initialize serial port to Arduino
    */
    CallbackReturn BabybotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }
        try
        {
            port_=info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("BabybotInterface"),"No Serial Port Provided! Aborting");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        last_run_ = rclcpp::Clock().now();

        return CallbackReturn::SUCCESS;
        
    }

    /*
    position state and velocity state
    */
    std::vector<hardware_interface::StateInterface>BabybotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
                hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }
        return state_interfaces;
    }

    /*
    the command_interface 
    */
    std::vector<hardware_interface::CommandInterface> BabybotInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }
        return command_interfaces;
         
    }

    /*
    initialize commands, positions and velocity, set the serial baudrate 
    */
    CallbackReturn BabybotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("BabybotInterface"), "Starting robot hardware...");
        velocity_commands_ = {0.0, 0.0};
        position_states_ = {0.0, 0.0};
        velocity_states_ = {0.0, 0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("BabybotInterface"),"Something wen wrong while closing the connection with port "<< port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("BabybotInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }

    /*
    deactivate serial port 
    */
    CallbackReturn BabybotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        
        RCLCPP_INFO(rclcpp::get_logger("BabybotInterface"), "Stopping robot hardware...");
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("BabyotInterface"),"Something wen wrong while closing the connection with port "<< port_);
                return CallbackReturn::FAILURE;
            }

        }
    }

    /*
    read the wheel encoder reading, store in the velocity, position arrays 
    */
    hardware_interface::return_type BabybotInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        if(arduino_.IsDataAvailable())
        {
            std::string message;
            arduino_.ReadLine(message);
            
            // 1. Basic length check to avoid substr out-of-bounds
            if (message.size() < 4) return hardware_interface::return_type::OK;

            auto dt = (rclcpp::Clock().now() - last_run_).seconds();
            std::stringstream ss(message);
            std::string res;

            while (std::getline(ss, res, ','))
            {
                // 2. Ensure the individual segment is long enough (e.g., "rp1.0")
                if (res.size() < 3) continue;

                try {
                    int multiplier = (res.at(1) == 'p') ? 1 : -1;
                    double val = std::stod(res.substr(2)); // substr(2) goes to end of string

                    if(res.at(0) == 'r')
                    {
                        velocity_states_.at(0) = multiplier * val;
                        position_states_.at(0) += velocity_states_.at(0) * dt;
                    }
                    else if (res.at(0) == 'l')
                    {
                        velocity_states_.at(1) = multiplier * val;
                        position_states_.at(1) += velocity_states_.at(1) * dt;    
                    }
                }
                catch (const std::invalid_argument& e) {
                    //RCLCPP_WARN(rclcpp::get_logger("BabybotInterface"), "Malformed serial data: %s", res.c_str());
                    continue; // Skip this segment and don't crash
                }
            }
            last_run_ = rclcpp::Clock().now();
        }
        return hardware_interface::return_type::OK; 
    }

    /*
    write the PMW to serial
    */
    hardware_interface::return_type BabybotInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        std::stringstream message_stream;
        char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
        char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";

        if ( std::abs(velocity_commands_.at(0)) < 10.0 )
        {
            compensate_zeros_right = "0";
        }
        else
        {
            compensate_zeros_right = "";
        }

        if ( std::abs(velocity_commands_.at(1)) < 10.0 )
        {
            compensate_zeros_left = "0";
        }
        else
        {
            compensate_zeros_left = "";
        }

        message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) <<
            ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << ",";

        try
        {
            arduino_.Write(message_stream.str());
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("BabybotInterface"),"Something went wrong while sending the message" << 
                message_stream.str() << " on the port " << port_ );
            return hardware_interface::return_type::ERROR;
        }
        
        return hardware_interface::return_type::OK;
    }
        
}

/*
export the babybot_firmware plugin 
*/
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(babybot_firmware::BabybotInterface, hardware_interface::SystemInterface)