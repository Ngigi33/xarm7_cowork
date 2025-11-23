#include <rclcpp/rclcpp.hpp>
#include "xarm_custom_interfaces/srv/trigger.hpp"
#include "xarm_custom_interfaces/msg/task_status.hpp"
#include <cstdlib>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <ctime>

using Trigger = xarm_custom_interfaces::srv::Trigger;

class SequenceRunner : public rclcpp::Node
{
public:
    SequenceRunner() : Node("sequence_runner")
    {
        // --- Publisher for task status ---
        status_pub_ = this->create_publisher<xarm_custom_interfaces::msg::TaskStatus>("task_status", 10);

        // --- CSV file setup (append mode) ---
        csv_file_.open("/home/vmlabs/xarm7_cowork/dev_ws/task_log.csv", std::ios::out | std::ios::app);
        if (csv_file_.tellp() == 0) // If file is empty, write header
        {
            csv_file_ << "Task Name,Start Time,End Time,Duration(s),Idle Time(s)\n";
            csv_file_.flush();
        }

        // --- Define paths for Python tasks + virtual environment ---
        std::string base_path = "/home/vmlabs/xarm7_cowork/dev_ws/src";
        std::string tasks_path = base_path + "/tasks";
        std::string venv_activate = "source " + base_path + "/xarm_env/bin/activate";

        auto make_cmd = [&](const std::string &script) {
            return "bash -c \"cd " + tasks_path +
                   " && " + venv_activate +
                   " && python " + script + "\"";
        };

        // --- Define main and post-manual sequences ---
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

        // --- Create services ---
        main_service_ = this->create_service<Trigger>(
            "start_task_sequence",
            std::bind(&SequenceRunner::startMainSequence, this,
                      std::placeholders::_1, std::placeholders::_2));

        auto_service_ = this->create_service<Trigger>(
            "start_auto_sequence",
            std::bind(&SequenceRunner::startAutoSequence, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Sequence Runner Node is ready.");
    }

    ~SequenceRunner()
    {
        if (csv_file_.is_open())
            csv_file_.close();
    }

private:
    rclcpp::Publisher<xarm_custom_interfaces::msg::TaskStatus>::SharedPtr status_pub_;
    rclcpp::Service<Trigger>::SharedPtr main_service_;
    rclcpp::Service<Trigger>::SharedPtr auto_service_;

    std::vector<std::string> main_sequence_scripts_;
    std::vector<std::string> post_manual_sequence_scripts_;

    int task_counter_ = 0;
    std::ofstream csv_file_;
    std::chrono::system_clock::time_point last_task_end_;

    // --- Helper: publish task status ---
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

    // --- Convert time_point to human-readable string ---
    std::string time_to_string(const std::chrono::system_clock::time_point &tp)
    {
        std::time_t t = std::chrono::system_clock::to_time_t(tp);
        std::tm tm = *std::localtime(&t);
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }

    // -----------------------------------------------------
    // MAIN SEQUENCE
    // -----------------------------------------------------
    void startMainSequence(const std::shared_ptr<Trigger::Request> request,
                           std::shared_ptr<Trigger::Response> response)
    {
        if (!request->run)
        {
            response->success = false;
            response->message = "Request 'run' was false, sequence not started.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting Main Sequence...");

        for (const auto &cmd : main_sequence_scripts_)
        {
            std::string script_name = cmd.substr(cmd.find("python ") + 7);

            auto start_time = std::chrono::system_clock::now();

            double idle_time = 0.0;
            if (last_task_end_.time_since_epoch().count() > 0)
            {
                idle_time = std::chrono::duration<double>(start_time - last_task_end_).count();
            }

            publish_status(script_name, "Starting", 0.0, task_counter_);
            publish_status(script_name, "In-progress", 50.0, task_counter_);

            int ret = system(cmd.c_str());

            auto end_time = std::chrono::system_clock::now();
            last_task_end_ = end_time;

            double duration = std::chrono::duration<double>(end_time - start_time).count();

            if (ret != 0)
            {
                publish_status(script_name, "Error", 0.0, task_counter_);
                response->success = false;
                response->message = "Failed to run: " + script_name;
                return;
            }

            publish_status(script_name, "Finished", 100.0, task_counter_);

            // Write CSV & flush
            csv_file_ << script_name << ","
                      << time_to_string(start_time) << ","
                      << time_to_string(end_time) << ","
                      << std::fixed << std::setprecision(2) << duration << ","
                      << idle_time << "\n";
            csv_file_.flush();

            task_counter_++;
        }

        response->success = true;
        response->message = "Main sequence completed successfully.";

        std::thread([this]() { startPostManualSequence(); }).detach();
    }

    // -----------------------------------------------------
    // POST-MANUAL SEQUENCE (manual trigger)
    // -----------------------------------------------------
    void startAutoSequence(const std::shared_ptr<Trigger::Request> request,
                           std::shared_ptr<Trigger::Response> response)
    {
        if (!request->run)
        {
            response->success = false;
            response->message = "Request 'run' was false, sequence not started.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting Post-Manual Sequence...");

        for (const auto &cmd : post_manual_sequence_scripts_)
        {
            std::string script_name = cmd.substr(cmd.find("python ") + 7);

            auto start_time = std::chrono::system_clock::now();

            double idle_time = 0.0;
            if (last_task_end_.time_since_epoch().count() > 0)
            {
                idle_time = std::chrono::duration<double>(start_time - last_task_end_).count();
            }

            publish_status(script_name, "Starting", 0.0, task_counter_);
            publish_status(script_name, "In-progress", 50.0, task_counter_);

            int ret = system(cmd.c_str());

            auto end_time = std::chrono::system_clock::now();
            last_task_end_ = end_time;

            double duration = std::chrono::duration<double>(end_time - start_time).count();

            if (ret != 0)
            {
                publish_status(script_name, "Error", 0.0, task_counter_);
                response->success = false;
                response->message = "Failed to run: " + script_name;
                return;
            }

            publish_status(script_name, "Finished", 100.0, task_counter_);

            csv_file_ << script_name << ","
                      << time_to_string(start_time) << ","
                      << time_to_string(end_time) << ","
                      << std::fixed << std::setprecision(2) << duration << ","
                      << idle_time << "\n";
            csv_file_.flush();

            task_counter_++;
        }

        response->success = true;
        response->message = "Post-manual sequence completed successfully.";
    }

    // -----------------------------------------------------
    // INTERNAL AUTO START FOR POST-MANUAL SEQUENCE
    // -----------------------------------------------------
    void startPostManualSequence()
    {
        RCLCPP_INFO(this->get_logger(), "Automatically triggering Post-Manual Sequence...");

        for (const auto &cmd : post_manual_sequence_scripts_)
        {
            std::string script_name = cmd.substr(cmd.find("python ") + 7);

            auto start_time = std::chrono::system_clock::now();

            double idle_time = 0.0;
            if (last_task_end_.time_since_epoch().count() > 0)
            {
                idle_time = std::chrono::duration<double>(start_time - last_task_end_).count();
            }

            publish_status(script_name, "Starting", 0.0, task_counter_);

            // int ret = system(cmd.c_str());

            auto end_time = std::chrono::system_clock::now();
            last_task_end_ = end_time;

            double duration = std::chrono::duration<double>(end_time - start_time).count();

            publish_status(script_name, "Finished", 100.0, task_counter_);

            csv_file_ << script_name << ","
                      << time_to_string(start_time) << ","
                      << time_to_string(end_time) << ","
                      << std::fixed << std::setprecision(2) << duration << ","
                      << idle_time << "\n";
            csv_file_.flush();

            task_counter_++;
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
// #include "xarm_custom_interfaces/msg/task_status.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <cstdlib>
// #include <vector>
// #include <string>
// #include <thread>
// #include <chrono>

// using Trigger = xarm_custom_interfaces::srv::Trigger;

// class SequenceRunner : public rclcpp::Node
// {
// public:
//     SequenceRunner() : Node("sequence_runner")
//     {
//         //
//         // --- Setup publisher ---
//         status_pub_ = this->create_publisher<xarm_custom_interfaces::msg::TaskStatus>("task_status", 10);

//         //
//         // --- Define paths for your Python tasks + venv activation ---
//         std::string base_path = "/home/vmlabs/xarm7_cowork/dev_ws/src";
//         std::string tasks_path = base_path + "/tasks";
//         std::string venv_activate = "source " + base_path + "/xarm_env/bin/activate";

//         //
//         // --- Lambda to build the correct execution command ---
//         auto make_cmd = [&](const std::string &script)
//         {
//             return "bash -c \"cd " + tasks_path +
//                    " && " + venv_activate +
//                    " && python " + script + "\"";
//         };

//         //
//         // --- Build sequences ---
//         main_sequence_scripts_ = {
//             make_cmd("pinion_gear_task.py"),
//             make_cmd("pinion_spindle_task.py"),
//             make_cmd("spur_spindle_task.py"),
//             make_cmd("adhesive_applier_task.py"),
//             make_cmd("manual_mode_changer.py")};

//         post_manual_sequence_scripts_ = {
//             make_cmd("adhesive_returner_task.py"),
//             make_cmd("cover_plate_task.py"),
//             make_cmd("manual_mode_changer.py")};

//         //
//         // --- Create services ---
//         main_service_ = this->create_service<Trigger>(
//             "start_task_sequence",
//             std::bind(&SequenceRunner::startMainSequence, this,
//                       std::placeholders::_1, std::placeholders::_2));

//         auto_service_ = this->create_service<Trigger>(
//             "start_auto_sequence",
//             std::bind(&SequenceRunner::startAutoSequence, this,
//                       std::placeholders::_1, std::placeholders::_2));

//         RCLCPP_INFO(this->get_logger(), "Sequence Runner Node is ready.");
//     }

// private:
//     rclcpp::Publisher<xarm_custom_interfaces::msg::TaskStatus>::SharedPtr status_pub_;

//     rclcpp::Service<Trigger>::SharedPtr main_service_;
//     rclcpp::Service<Trigger>::SharedPtr auto_service_;

//     std::vector<std::string> main_sequence_scripts_;
//     std::vector<std::string> post_manual_sequence_scripts_;

//     int task_counter_ = 0; // keeps track across all sequences

//     // --- Helper to publish status ---
//     void publish_status(const std::string &task, const std::string &state, float progress = 0.0, int task_id = 0)
//     {
//         xarm_custom_interfaces::msg::TaskStatus msg;
//         msg.task_name = task;
//         msg.state = state;       // "Starting", "In-progress", "Finished", "Error"
//         msg.progress = progress; // 0.0-100.0
//         msg.task_id = task_id;
//         msg.timestamp = this->now();
//         status_pub_->publish(msg);

//         RCLCPP_INFO(this->get_logger(), "Task: %s | State: %s | Progress: %.1f%%",
//                     task.c_str(), state.c_str(), progress);
//     }

//     // -----------------------------------------------------
//     // MAIN SEQUENCE
//     // -----------------------------------------------------
//     void startMainSequence(
//         const std::shared_ptr<Trigger::Request> request,
//         std::shared_ptr<Trigger::Response> response)
//     {
//         if (!request->run)
//         {
//             response->success = false;
//             response->message = "Request 'run' was false, sequence not started.";
//             return;
//         }

//         RCLCPP_INFO(this->get_logger(), "Starting Main Sequence...");

//         int task_counter_ = 0;
//         for (const auto &cmd : main_sequence_scripts_)
//         {
//             std::string script_name = cmd.substr(cmd.find("python ") + 7);

//             // Starting
//             publish_status(script_name, "Starting", 0.0, task_counter_);

//             // Optional: simulate progress updates before running (or if you can read actual progress from Python script)
//             publish_status(script_name, "In-progress", 50.0, task_counter_);

//             // Run the Python script
//             int ret = system(cmd.c_str());

//             if (ret != 0)
//             {
//                 publish_status(script_name, "Error", 0.0, task_counter_);
//                 response->success = false;
//                 response->message = "Failed to run: " + script_name;
//                 return;
//             }

//             // Finished
//             publish_status(script_name, "Finished", 100.0, task_counter_);

//             task_counter_++;
//         }

//         response->success = true;
//         response->message = "Main sequence completed successfully.";

//         //
//         // // Automatically start post/manual sequence in a separate thread
//         // std::thread([this]()
//         //             { startPostManualSequence(); })
//         //     .detach();
//     }

//     // -----------------------------------------------------
//     // AUTO SEQUENCE (manual switch)
//     // -----------------------------------------------------
//     void startAutoSequence(
//         const std::shared_ptr<Trigger::Request> request,
//         std::shared_ptr<Trigger::Response> response)
//     {
//         if (!request->run)
//         {
//             response->success = false;
//             response->message = "Request 'run' was false, sequence not started.";
//             return;
//         }

//         RCLCPP_INFO(this->get_logger(), "Starting Post-Manual Sequence...");

//         int task_counter_ = 0;
//         for (const auto &cmd : post_manual_sequence_scripts_)
//         {
//             std::string script_name = cmd.substr(cmd.find("python ") + 7);

//             // Starting
//             publish_status(script_name, "Starting", 0.0, task_counter_);

//             // Optional: simulate progress updates before running (or if you can read actual progress from Python script)
//             publish_status(script_name, "In-progress", 50.0, task_counter_);

//             // Run the Python script
//             int ret = system(cmd.c_str());

//             if (ret != 0)
//             {
//                 publish_status(script_name, "Error", 0.0, task_counter_);
//                 response->success = false;
//                 response->message = "Failed to run: " + script_name;
//                 return;
//             }

//             // Finished
//             publish_status(script_name, "Finished", 100.0, task_counter_);

//             task_counter_++;
//         }

//         response->success = true;
//         response->message = "Post-manual sequence completed successfully.";
//     }

//     // -----------------------------------------------------
//     // INTERNAL AUTO START
//     // -----------------------------------------------------
//     void startPostManualSequence()
//     {
//         RCLCPP_INFO(this->get_logger(), "Automatically triggering Post-Manual Sequence...");

//         for (const auto &cmd : post_manual_sequence_scripts_)
//         {
//             std::string script_name = cmd.substr(cmd.find("python ") + 7);

//             publish_status(script_name, "Starting", 0.0, task_counter_++);
//             // int ret = system(cmd.c_str());
//             publish_status(script_name, "Finished", 100.0, task_counter_ - 1);
//         }
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<SequenceRunner>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
