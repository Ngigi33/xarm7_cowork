// #include <rclcpp/rclcpp.hpp>
// #include "xarm_msgs/msg/stop_msg.hpp"

// class StopMotion : public rclcpp::Node
// {
// public:
//     StopMotion() : Node("stop_motion_node")
//     {
//         stop_subscriber_ = this->create_subscription<xarm_msgs::msg::StopMsg>(
//             "stop_motion", 10,
//             std::bind(&StopMotion::stopMotionCallback, this, std::placeholders::_1));
//         RCLCPP_INFO(this->get_logger(), "Stop Motion Subscriber is running...");
//     }

// private:
//     rclcpp::Subscription<xarm_msgs::msg::StopMsg>::SharedPtr stop_subscriber_;
//     bool prev_state_ = false;
//     std::string make_cmd(const std::string &script)
//     {
//         return "bash -c \"cd " + tasks_path_ +
//                " && " + venv_activate_ +
//                " && python " + script + "\"";
//     }
//     void stopMotionCallback(const xarm_msgs::msg::StopMsg::SharedPtr msg)
//     {
//         // Trigger only on rising edge: false -> true
//         if (msg->stop_flag && !prev_state_)
//         {
//             RCLCPP_WARN(this->get_logger(), "STOP button PRESSED!");

//             std::string cmd = make_cmd("stop_motion.py");
//             int ret = system(cmd.c_str());

//             if (ret != 0)
//                 RCLCPP_ERROR(this->get_logger(), "Failed to run open_gripper_final_task.py");
//             else
//                 RCLCPP_INFO(this->get_logger(), "Finished open_gripper_final_task.py")
//         }

//         // Save state for next comparison
//         prev_state_ = msg->stop_flag;
//     }
// }

// int
// main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<StopMotion>());
//     rclcpp::shutdown();
//     return 0;
// }
#include <rclcpp/rclcpp.hpp>
#include "xarm_msgs/msg/stop_msg.hpp"
#include "xarm_msgs/srv/trigger.hpp"
#include <cstdlib>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sys/wait.h>
#include <memory>
#include <atomic>

using Trigger = xarm_msgs::srv::Trigger;

using namespace std::chrono_literals;
using std::placeholders::_1;

class StopMotion : public rclcpp::Node, public std::enable_shared_from_this<StopMotion>
{
public:
    StopMotion() : Node("stop_motion_node")
    {
        std::string base_path = "/home/vmlabs/xarm7_cowork/dev_ws/src";
        tasks_path_ = base_path + "/tasks";
        venv_activate_ = "source " + base_path + "/xarm_env/bin/activate";

        stop_subscriber_ = this->create_subscription<xarm_msgs::msg::StopMsg>(
            "stop_motion",
            10,
            std::bind(&StopMotion::stopMotionCallback, this, _1));

        stop_publisher_ = this->create_publisher<xarm_msgs::msg::StopMsg>(
            "stop_motion",
            10);

        xarm_msgs::msg::StopMsg msg;   // create the message object
        msg.stop_flag = false;         // set the boolean field
        stop_publisher_->publish(msg); // publish

        RCLCPP_INFO(this->get_logger(), "STOP node (pub + sub) running...");
    }

private:
    std::string tasks_path_;
    std::string venv_activate_;

    std::string make_cmd(const std::string &script)
    {
        // Use bash -lc so that `source` works and environment lasts for the chained commands.
        return "bash -lc \"cd '" + tasks_path_ + "' && " + venv_activate_ + " && python3 '" + script + "'\"";
    }

    void run_script_async(const std::string &script, const std::string &descr)
    {
        // Launch the script in a detached thread. Avoid capturing shared_from_this
        // because that can throw if the object isn't owned as expected. Use a
        // static logger instead.
        std::string cmd = make_cmd(script);
        std::thread([cmd, descr]() {
            auto logger = rclcpp::get_logger("stop_motion_node");
            RCLCPP_INFO(logger, "Running %s", descr.c_str());
            int ret = system(cmd.c_str());
            if (ret == -1)
            {
                RCLCPP_ERROR(logger, "system() failed when running %s", descr.c_str());
            }
            else if (WIFEXITED(ret))
            {
                int code = WEXITSTATUS(ret);
                if (code == 0)
                    RCLCPP_INFO(logger, "%s finished successfully (exit %d)", descr.c_str(), code);
                else
                    RCLCPP_ERROR(logger, "%s exited with code %d", descr.c_str(), code);
            }
            else
            {
                RCLCPP_ERROR(logger, "%s terminated abnormally", descr.c_str());
            }
        })
            .detach();
    }

    void kill_processes_by_pattern(const std::string &pattern)
    {
        // Safer approach: parse `ps` output and kill only processes whose
        // full command line exactly ends with the given pattern. This avoids
        // broad pkill -f matches that may capture unrelated processes.
        FILE *fp = popen("ps -eo pid=,args=", "r");
        if (!fp)
        {
            RCLCPP_ERROR(this->get_logger(), "failed to run ps for pattern: %s", pattern.c_str());
            return;
        }

        char *line = nullptr;
        size_t len = 0;
        ssize_t read;
        std::vector<int> killed;
        while ((read = getline(&line, &len, fp)) != -1)
        {
            std::string s(line);
            // Trim leading spaces
            size_t pos = s.find_first_not_of(' ');
            if (pos == std::string::npos)
                continue;
            s = s.substr(pos);
            // split into pid and args
            size_t sep = s.find(' ');
            if (sep == std::string::npos)
                continue;
            std::string pid_str = s.substr(0, sep);
            std::string args = s.substr(sep + 1);
            // check if args ends with pattern (allow optional whitespace/newline)
            // Trim trailing whitespace from args
            while (!args.empty() && isspace((unsigned char)args.back()))
                args.pop_back();
            if (args.size() >= pattern.size() && args.compare(args.size() - pattern.size(), pattern.size(), pattern) == 0)
            {
                // matched exact tail; kill this pid
                int pid = atoi(pid_str.c_str());
                if (pid > 1)
                {
                    if (kill(pid, SIGKILL) == 0)
                    {
                        killed.push_back(pid);
                        RCLCPP_INFO(this->get_logger(), "killed pid %d for pattern %s", pid, pattern.c_str());
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "failed to kill pid %d for pattern %s", pid, pattern.c_str());
                    }
                }
            }
        }
        if (line)
            free(line);
        pclose(fp);

        if (killed.empty())
            RCLCPP_INFO(this->get_logger(), "no processes found matching exact pattern: %s", pattern.c_str());
    }

    void stopMotionCallback(const xarm_msgs::msg::StopMsg::SharedPtr msg)
    {
        // Rising edge detect
        if (msg->stop_flag && !prev_state_)
        {
            RCLCPP_WARN(this->get_logger(), "STOP triggered!");

            // Kill any running task sequencer processes that match the commands used
            // in the stop script, to ensure the sequencer is not running.
            kill_processes_by_pattern("ros2 run xarm_sequence_runner task_sequencer");
            // Also try a broader match as a fallback
            // kill_processes_by_pattern("task_sequencer");

            // Run stop motion script asynchronously so we don't block the callback/executor
            run_script_async("stop_motion.py", "stop_motion.py");
            // Cancel old timer if it exists
            if (reset_timer_)
            {
                reset_timer_->cancel();
            }

            // Create a ONE-SHOT timer
            reset_timer_ = this->create_wall_timer(2s, std::bind(&StopMotion::publishFalse, this));
        }

        prev_state_ = msg->stop_flag;
    }

    void publishFalse()
    {
        xarm_msgs::msg::StopMsg msg;
        msg.stop_flag = false;

        stop_publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Auto reset -> published FALSE");

        // Stop the timer so it only runs once
        reset_timer_->cancel();
    }
    // reset service handled by process_launcher node now

    std::atomic_bool prev_state_{false};

    rclcpp::Subscription<xarm_msgs::msg::StopMsg>::SharedPtr stop_subscriber_;
    rclcpp::Publisher<xarm_msgs::msg::StopMsg>::SharedPtr stop_publisher_;
    rclcpp::TimerBase::SharedPtr reset_timer_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StopMotion>());
    rclcpp::shutdown();
    return 0;
}