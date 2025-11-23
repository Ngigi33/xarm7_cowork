#include <rclcpp/rclcpp.hpp>
#include "xarm_custom_interfaces/srv/trigger.hpp"
#include "xarm_custom_interfaces/msg/task_status.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstdlib>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

using Trigger = xarm_custom_interfaces::srv::Trigger;

class SequenceRunner : public rclcpp::Node
{
public:
    SequenceRunner() : Node("sequence_runner")
    {
        //
        // --- Setup publisher ---
        status_pub_ = this->create_publisher<xarm_custom_interfaces::msg::TaskStatus>("task_status", 10);

        //
        // --- Define paths for your Python tasks + venv activation ---
        std::string base_path = "/home/vmlabs/xarm7_cowork/dev_ws/src";
        std::string tasks_path = base_path + "/tasks";
        std::string venv_activate = "source " + base_path + "/xarm_env/bin/activate";

        //
        // --- Lambda to build the correct execution command ---
        auto make_cmd = [&](const std::string &script)
        {
            return "bash -c \"cd " + tasks_path +
                   " && " + venv_activate +
                   " && python " + script + "\"";
        };

        //
        // --- Build sequences ---
        main_sequence_scripts_ = {
            make_cmd("pinion_gear_task.py"),
            make_cmd("pinion_spindle_task.py"),
            make_cmd("spur_spindle_task.py"),
            make_cmd("adhesive_applier_task.py"),
            make_cmd("manual_mode_changer.py")};

        post_manual_sequence_scripts_ = {
            make_cmd("adhesive_returner_task.py"),
            make_cmd("cover_plate_task.py"),
            make_cmd("manual_mode_changer.py")};

        //
        // --- Create services ---
        main_service_ = this->create_service<Trigger>(
            "start_main_sequence",
            std::bind(&SequenceRunner::startMainSequence, this,
                      std::placeholders::_1, std::placeholders::_2));

        auto_service_ = this->create_service<Trigger>(
            "start_auto_sequence",
            std::bind(&SequenceRunner::startAutoSequence, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Sequence Runner Node is ready.");
    }

private:
    rclcpp::Publisher<xarm_custom_interfaces::msg::TaskStatus>::SharedPtr status_pub_;

    rclcpp::Service<Trigger>::SharedPtr main_service_;
    rclcpp::Service<Trigger>::SharedPtr auto_service_;

    std::vector<std::string> main_sequence_scripts_;
    std::vector<std::string> post_manual_sequence_scripts_;

    int task_counter_ = 0; // keeps track across all sequences

    // --- Helper to publish status ---
    void publish_status(const std::string &task, const std::string &state, float progress = 0.0, int task_id = 0)
    {
        xarm_custom_interfaces::msg::TaskStatus msg;
        msg.task_name = task;
        msg.state = state;       // "Starting", "In-progress", "Finished", "Error"
        msg.progress = progress; // 0.0-100.0
        msg.task_id = task_id;
        msg.timestamp = this->now();
        status_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Task: %s | State: %s | Progress: %.1f%%",
                    task.c_str(), state.c_str(), progress);
    }

    // -----------------------------------------------------
    // MAIN SEQUENCE
    // -----------------------------------------------------
    void startMainSequence(
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
    {
        if (!request->run)
        {
            response->success = false;
            response->message = "Request 'run' was false, sequence not started.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting Main Sequence...");

        int task_counter_ = 0;
        for (const auto &cmd : main_sequence_scripts_)
        {
            std::string script_name = cmd.substr(cmd.find("python ") + 7);

            // Starting
            publish_status(script_name, "Starting", 0.0, task_counter_);

            // Optional: simulate progress updates before running (or if you can read actual progress from Python script)
            publish_status(script_name, "In-progress", 50.0, task_counter_);

            // Run the Python script
            int ret = system(cmd.c_str());

            if (ret != 0)
            {
                publish_status(script_name, "Error", 0.0, task_counter_);
                response->success = false;
                response->message = "Failed to run: " + script_name;
                return;
            }

            // Finished
            publish_status(script_name, "Finished", 100.0, task_counter_);

            task_counter_++;
        }

        response->success = true;
        response->message = "Main sequence completed successfully.";

        //
        // // Automatically start post/manual sequence in a separate thread
        // std::thread([this]()
        //             { startPostManualSequence(); })
        //     .detach();
    }

    // -----------------------------------------------------
    // AUTO SEQUENCE (manual switch)
    // -----------------------------------------------------
    void startAutoSequence(
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
    {
        if (!request->run)
        {
            response->success = false;
            response->message = "Request 'run' was false, sequence not started.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting Post-Manual Sequence...");

        int task_counter_ = 0;
        for (const auto &cmd : post_manual_sequence_scripts_)
        {
            std::string script_name = cmd.substr(cmd.find("python ") + 7);

            // Starting
            publish_status(script_name, "Starting", 0.0, task_counter_);

            // Optional: simulate progress updates before running (or if you can read actual progress from Python script)
            publish_status(script_name, "In-progress", 50.0, task_counter_);

            // Run the Python script
            int ret = system(cmd.c_str());

            if (ret != 0)
            {
                publish_status(script_name, "Error", 0.0, task_counter_);
                response->success = false;
                response->message = "Failed to run: " + script_name;
                return;
            }

            // Finished
            publish_status(script_name, "Finished", 100.0, task_counter_);

            task_counter_++;
        }

        response->success = true;
        response->message = "Post-manual sequence completed successfully.";
    }

    // -----------------------------------------------------
    // INTERNAL AUTO START
    // -----------------------------------------------------
    void startPostManualSequence()
    {
        RCLCPP_INFO(this->get_logger(), "Automatically triggering Post-Manual Sequence...");

        for (const auto &cmd : post_manual_sequence_scripts_)
        {
            std::string script_name = cmd.substr(cmd.find("python ") + 7);

            publish_status(script_name, "Starting", 0.0, task_counter_++);
            // int ret = system(cmd.c_str());
            publish_status(script_name, "Finished", 100.0, task_counter_ - 1);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SequenceRunner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include "xarm_custom_interfaces/srv/trigger.hpp"
// #include <cstdlib>
// #include <vector>
// #include <string>
// #include <thread>

// using Trigger = xarm_custom_interfaces::srv::Trigger;

// class SequenceRunner : public rclcpp::Node
// {
// public:
//     SequenceRunner() : Node("sequence_runner")
//     {
//         main_service_ = this->create_service<Trigger>(
//             "start_main_sequence",
//             std::bind(&SequenceRunner::startMainSequence, this,
//                       std::placeholders::_1, std::placeholders::_2));

//         auto_service_ = this->create_service<Trigger>(
//             "start_auto_sequence",
//             std::bind(&SequenceRunner::startAutoSequence, this,
//                       std::placeholders::_1, std::placeholders::_2));

//         RCLCPP_INFO(this->get_logger(), "Sequence Runner Node is ready.");
//     }

// private:
//     rclcpp::Service<Trigger>::SharedPtr main_service_;
//     rclcpp::Service<Trigger>::SharedPtr auto_service_;

//     std::string make_cmd(const std::string &script)
//     {
//         return "python /home/vmlabs/xarm7_cowork/dev_ws/src/xarm_sequence_runner/scripts/" + script;
//     }

//     std::vector<std::string> main_sequence_scripts_ = {
//         make_cmd("pinion_gear_task.py"),
//         make_cmd("pinion_spindle_task.py"),
//         make_cmd("spur_spindle_task.py"),
//         make_cmd("adhesive_applier_task.py"),
//         make_cmd("manual_mode_changer.py")};

//     std::vector<std::string> post_manual_sequence_scripts_ = {
//         make_cmd("adhesive_returner_task.py"),
//         make_cmd("cover_plate_task.py"),
//         make_cmd("manual_mode_changer.py")};

//     void startMainSequence(const std::shared_ptr<Trigger::Request> request,
//                            std::shared_ptr<Trigger::Response> response)
//     {
//         if (!request->run)
//         {
//             response->success = false;
//             response->message = "Request 'run' was false.";
//             return;
//         }

//         RCLCPP_INFO(this->get_logger(), "Starting Main Sequence...");

//         for (const auto &script : main_sequence_scripts_)
//         {
//             int ret = system(script.c_str());
//             if (ret != 0)
//             {
//                 response->success = false;
//                 response->message = "Failed to run: " + script;
//                 RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
//                 return;
//             }
//         }

//         response->success = true;
//         response->message = "Main sequence completed.";

//         std::thread([this]() { startPostManualSequence(); }).detach();
//     }

//     void startAutoSequence(const std::shared_ptr<Trigger::Request> request,
//                            std::shared_ptr<Trigger::Response> response)
//     {
//         if (!request->run)
//         {
//             response->success = false;
//             response->message = "Request 'run' was false.";
//             return;
//         }

//         RCLCPP_INFO(this->get_logger(), "Starting Post-Manual Sequence...");
//         for (const auto &script : post_manual_sequence_scripts_)
//         {
//             int ret = system(script.c_str());
//             if (ret != 0)
//             {
//                 response->success = false;
//                 response->message = "Failed to run: " + script;
//                 return;
//             }
//         }

//         response->success = true;
//         response->message = "Post-manual sequence completed.";
//     }

//     void startPostManualSequence()
//     {
//         RCLCPP_INFO(this->get_logger(), "Auto-triggering Post-Manual Sequence...");
//         for (const auto &script : post_manual_sequence_scripts_)
//         {
//             system(script.c_str());
//         }
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SequenceRunner>());
//     rclcpp::shutdown();
//     return 0;
// }
