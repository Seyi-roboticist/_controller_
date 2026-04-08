/**
 * @file plc_hardware_interface.cpp
 * @author Seyi R. Afolayan
 * @brief ros2_control SystemInterface bridging controllers to PLC-governed robots
 *
 * @date 2025
 * @copyright MIT License
 */

#include "plc_ros2_bridge/plc_hardware_interface.hpp"
#include "plc_ros2_bridge/ethernet_ip_interface.hpp"
#include "plc_ros2_bridge/opcua_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace plc_ros2_bridge
{
    PLCHardwareInterface::PLCHardwareInterface()
    {
        logger_ = std::make_shared<rclcpp::Logger>(
            rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.plc_ros2_bridge"));
        clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    }

    PLCHardwareInterface::~PLCHardwareInterface()
    {
        if (plc_interface_ && plc_interface_->isConnected())
        {
            plc_interface_->disconnect();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    hardware_interface::CallbackReturn PLCHardwareInterface::on_init(
        const hardware_interface::HardwareInfo &hardware_info)
    {
        if (hardware_interface::SystemInterface::on_init(hardware_info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Parse hardware parameters from URDF
        auto param_it = info_.hardware_parameters.find("protocol");
        if (param_it != info_.hardware_parameters.end())
        {
            protocol_ = param_it->second;
        }
        else
        {
            protocol_ = "ethernet_ip";  // Default to EtherNet/IP
        }

        // Collect all PLC-specific parameters
        for (const auto &[key, value] : info_.hardware_parameters)
        {
            plc_params_[key] = value;
        }

        // Get tag mapping and safety config file paths
        param_it = info_.hardware_parameters.find("tag_mapping_file");
        if (param_it != info_.hardware_parameters.end())
        {
            tag_mapping_file_ = param_it->second;
        }

        param_it = info_.hardware_parameters.find("safety_config_file");
        if (param_it != info_.hardware_parameters.end())
        {
            safety_config_file_ = param_it->second;
        }

        // Extract joint names from the hardware info
        for (const auto &joint : info_.joints)
        {
            joint_names_.push_back(joint.name);
        }

        // Resize state and command vectors
        size_t n_joints = joint_names_.size();
        hw_positions_.resize(n_joints, 0.0);
        hw_velocities_.resize(n_joints, 0.0);
        hw_efforts_.resize(n_joints, 0.0);
        hw_cmd_positions_.resize(n_joints, 0.0);
        hw_cmd_velocities_.resize(n_joints, 0.0);
        hw_cmd_efforts_.resize(n_joints, 0.0);

        RCLCPP_INFO(get_logger(), "PLC Hardware Interface initialized");
        RCLCPP_INFO(get_logger(), "  Protocol: %s", protocol_.c_str());
        RCLCPP_INFO(get_logger(), "  Joints: %zu", n_joints);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PLCHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Configuring PLC Hardware Interface...");

        // Create the appropriate PLC interface
        plc_interface_ = createPLCInterface(protocol_);
        if (!plc_interface_)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create PLC interface for protocol: %s",
                         protocol_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Configure the PLC interface with collected parameters
        if (!plc_interface_->configure(plc_params_))
        {
            RCLCPP_ERROR(get_logger(), "Failed to configure PLC interface");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Load tag mappings if configured
        if (!tag_mapping_file_.empty())
        {
            if (!tag_registry_.loadFromYAML(tag_mapping_file_))
            {
                RCLCPP_WARN(get_logger(), "Failed to load tag mappings from: %s",
                            tag_mapping_file_.c_str());
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Loaded %zu tag mappings", tag_registry_.size());
            }
        }

        RCLCPP_INFO(get_logger(), "PLC Hardware Interface configured successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PLCHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Activating PLC Hardware Interface...");

        // Connect to PLC
        if (!plc_interface_->connect())
        {
            auto diag = plc_interface_->getDiagnostics();
            RCLCPP_ERROR(get_logger(), "Failed to connect to PLC: %s",
                         diag.last_error_message.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Connected to PLC via %s",
                     plc_interface_->getProtocolName().c_str());

        // Validate that all required tags exist in the PLC
        auto all_tags = tag_registry_.getAllPLCTagNames();
        if (!all_tags.empty())
        {
            auto validation = plc_interface_->validateTags(all_tags);
            for (const auto &[tag, exists] : validation)
            {
                if (!exists)
                {
                    RCLCPP_ERROR(get_logger(), "Required PLC tag not found: %s", tag.c_str());
                    plc_interface_->disconnect();
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
            RCLCPP_INFO(get_logger(), "All %zu PLC tags validated", all_tags.size());
        }

        // Initialize command values to zero
        std::fill(hw_cmd_velocities_.begin(), hw_cmd_velocities_.end(), 0.0);
        std::fill(hw_cmd_positions_.begin(), hw_cmd_positions_.end(), 0.0);
        std::fill(hw_cmd_efforts_.begin(), hw_cmd_efforts_.end(), 0.0);

        RCLCPP_INFO(get_logger(), "PLC Hardware Interface activated successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PLCHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating PLC Hardware Interface...");

        // Zero all command outputs before disconnecting
        auto cmd_tags = tag_registry_.getCommandTags();
        if (!cmd_tags.empty() && plc_interface_->isConnected())
        {
            std::unordered_map<std::string, PLCValue> zero_commands;
            for (const auto *mapping : cmd_tags)
            {
                zero_commands[mapping->plc_tag] = 0.0f;
            }
            plc_interface_->writeTags(zero_commands);
        }

        // Disconnect
        if (plc_interface_)
        {
            plc_interface_->disconnect();
        }

        RCLCPP_INFO(get_logger(), "PLC Hardware Interface deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Interface Export
    // ─────────────────────────────────────────────────────────────────────────

    std::vector<hardware_interface::StateInterface> PLCHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            interfaces.emplace_back(hardware_interface::StateInterface(
                joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            interfaces.emplace_back(hardware_interface::StateInterface(
                joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
            interfaces.emplace_back(hardware_interface::StateInterface(
                joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
        }

        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> PLCHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            // Export command interfaces based on what's configured in URDF
            for (const auto &joint : info_.joints)
            {
                if (joint.name != joint_names_[i]) continue;

                for (const auto &cmd_iface : joint.command_interfaces)
                {
                    if (cmd_iface.name == hardware_interface::HW_IF_POSITION)
                    {
                        interfaces.emplace_back(hardware_interface::CommandInterface(
                            joint_names_[i], hardware_interface::HW_IF_POSITION,
                            &hw_cmd_positions_[i]));
                    }
                    else if (cmd_iface.name == hardware_interface::HW_IF_VELOCITY)
                    {
                        interfaces.emplace_back(hardware_interface::CommandInterface(
                            joint_names_[i], hardware_interface::HW_IF_VELOCITY,
                            &hw_cmd_velocities_[i]));
                    }
                    else if (cmd_iface.name == hardware_interface::HW_IF_EFFORT)
                    {
                        interfaces.emplace_back(hardware_interface::CommandInterface(
                            joint_names_[i], hardware_interface::HW_IF_EFFORT,
                            &hw_cmd_efforts_[i]));
                    }
                }
            }
        }

        return interfaces;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Cyclic I/O
    // ─────────────────────────────────────────────────────────────────────────

    hardware_interface::return_type PLCHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!plc_interface_ || !plc_interface_->isConnected())
        {
            return hardware_interface::return_type::ERROR;
        }

        // Batch-read all feedback tags
        auto feedback_tags = tag_registry_.getFeedbackTags();
        if (feedback_tags.empty())
        {
            return hardware_interface::return_type::OK;
        }

        std::vector<std::string> tag_names;
        for (const auto *mapping : feedback_tags)
        {
            tag_names.push_back(mapping->plc_tag);
        }

        std::unordered_map<std::string, PLCValue> values;
        auto results = plc_interface_->readTags(tag_names, values);

        // Update state interfaces with read values
        for (const auto *mapping : feedback_tags)
        {
            auto result_it = results.find(mapping->plc_tag);
            if (result_it != results.end() && result_it->second == IOResult::SUCCESS)
            {
                auto value_it = values.find(mapping->plc_tag);
                if (value_it != values.end())
                {
                    // Extract numeric value and apply inverse transform
                    double plc_val = std::visit([](auto &&v) -> double
                    {
                        return static_cast<double>(v);
                    }, value_it->second);

                    double ros2_val = mapping->fromPLC(plc_val);

                    // Route to appropriate state vector
                    for (size_t i = 0; i < joint_names_.size(); ++i)
                    {
                        if (joint_names_[i] == mapping->joint_name)
                        {
                            if (mapping->interface_type == "position")
                                hw_positions_[i] = ros2_val;
                            else if (mapping->interface_type == "velocity")
                                hw_velocities_[i] = ros2_val;
                            else if (mapping->interface_type == "effort")
                                hw_efforts_[i] = ros2_val;
                            break;
                        }
                    }
                }
            }
        }

        // Feed watchdog if safety validator is active
        if (safety_validator_)
        {
            safety_validator_->feedWatchdog();
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PLCHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!plc_interface_ || !plc_interface_->isConnected())
        {
            return hardware_interface::return_type::ERROR;
        }

        // Validate commands through safety layer if present
        if (safety_validator_)
        {
            if (!safety_validator_->validateJointVelocities(joint_names_, hw_cmd_velocities_))
            {
                RCLCPP_WARN(get_logger(), "Joint velocities blocked by safety validator");
                return hardware_interface::return_type::OK;  // Don't return ERROR — just skip the write
            }
        }

        // Build batch write from command interfaces
        auto cmd_tags = tag_registry_.getCommandTags();
        if (cmd_tags.empty())
        {
            return hardware_interface::return_type::OK;
        }

        std::unordered_map<std::string, PLCValue> tag_values;

        for (const auto *mapping : cmd_tags)
        {
            double ros2_val = 0.0;

            // Find the matching joint and interface
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                if (joint_names_[i] == mapping->joint_name)
                {
                    if (mapping->interface_type == "velocity")
                        ros2_val = hw_cmd_velocities_[i];
                    else if (mapping->interface_type == "position")
                        ros2_val = hw_cmd_positions_[i];
                    else if (mapping->interface_type == "effort")
                        ros2_val = hw_cmd_efforts_[i];
                    break;
                }
            }

            // Apply forward transform and package as PLCValue
            double plc_val = mapping->toPLC(ros2_val);
            tag_values[mapping->plc_tag] = static_cast<float>(plc_val);  // Most PLCs use REAL (float)
        }

        // Batch write
        auto results = plc_interface_->writeTags(tag_values);

        // Check for failures
        for (const auto &[tag, result] : results)
        {
            if (result != IOResult::SUCCESS)
            {
                RCLCPP_WARN(get_logger(), "Failed to write tag %s: %d",
                            tag.c_str(), static_cast<int>(result));
            }
        }

        return hardware_interface::return_type::OK;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Factory Methods
    // ─────────────────────────────────────────────────────────────────────────

    PLCInterface::UniquePtr PLCHardwareInterface::createPLCInterface(const std::string &protocol)
    {
        if (protocol == "ethernet_ip" || protocol == "EtherNet/IP")
        {
            RCLCPP_INFO(get_logger(), "Creating EtherNet/IP interface");
            return std::make_unique<EtherNetIPInterface>();
        }
        else if (protocol == "opcua" || protocol == "OPC UA" || protocol == "opc_ua")
        {
            RCLCPP_INFO(get_logger(), "Creating OPC UA interface");
            return std::make_unique<OPCUAInterface>();
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Unknown protocol: %s", protocol.c_str());
            return nullptr;
        }
    }

    SafetyValidator::SharedPtr PLCHardwareInterface::createSafetyValidator()
    {
        // Safety validator implementation would be instantiated here
        // For now, return nullptr (no safety validation)
        return nullptr;
    }

}  // namespace plc_ros2_bridge

PLUGINLIB_EXPORT_CLASS(plc_ros2_bridge::PLCHardwareInterface, hardware_interface::SystemInterface)
