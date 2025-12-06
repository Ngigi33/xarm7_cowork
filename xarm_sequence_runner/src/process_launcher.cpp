#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <iostream>
#include <mutex>
#include "xarm_msgs/srv/trigger.hpp"

using Trigger = xarm_msgs::srv::Trigger;

class ProcessLauncher : public rclcpp::Node
{
public:
    ProcessLauncher()
        : Node("process_launcher")
    {
        std::string workspace = "/home/vmlabs/xarm7_cowork/dev_ws";
        commands_.push_back("cd " + workspace + " && source install/setup.bash && exec ros2 run xarm_sequence_runner task_sequencer");
        commands_.push_back("cd " + workspace + " && source install/setup.bash && exec ros2 run xarm_sequence_runner emergency_stop_subscriber");

        RCLCPP_INFO(this->get_logger(), "ProcessLauncher starting %zu commands", commands_.size());

        for (size_t i = 0; i < commands_.size(); ++i)
        {
            launch(i);
        }

        // Create reset service so other nodes can request the sequencer to start
        reset_service_ = this->create_service<Trigger>(
            "reset",
            std::bind(&ProcessLauncher::on_reset, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~ProcessLauncher()
    {
        // Ask children to terminate
        for (pid_t p : pids_)
        {
            if (p > 1)
            {
                kill(p, SIGTERM);
            }
        }
    }

private:
    void on_reset(const std::shared_ptr<Trigger::Request> request,
                  std::shared_ptr<Trigger::Response> response)
    {
        // If run==true request, ensure the task_sequencer (index 0) is running
        if (!request->run)
        {
            response->success = false;
            response->message = "run flag not set";
            return;
        }

        std::lock_guard<std::mutex> lk(pids_mutex_);
        pid_t existing = (pids_.size() > 0) ? pids_[0] : 0;
        if (existing > 1)
        {
            // check if process is alive
            if (kill(existing, 0) == 0)
            {
                response->success = true;
                response->message = "Task sequencer already running";
                return;
            }
        }

        // spawn the sequencer (command index 0)
        pid_t pid = fork();
        if (pid == -1)
        {
            response->success = false;
            response->message = "fork failed";
            return;
        }
        if (pid == 0)
        {
            // child
            setsid();
            execl("/bin/bash", "bash", "-lc", commands_[0].c_str(), (char *)NULL);
            _exit(127);
        }
        // parent
        if (pids_.size() <= 0)
            pids_.resize(1, 0);
        pids_[0] = pid;
        response->success = true;
        response->message = "Task sequencer started";
        RCLCPP_INFO(this->get_logger(), "reset service started sequencer pid=%d", pid);
    }
    void launch(size_t index)
    {
        std::thread([this, index]() {
            const std::string &cmd = commands_[index];

            pid_t pid = fork();
            if (pid == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "fork() failed for command %zu", index);
                return;
            }

            if (pid == 0)
            {
                // child
                // create new session so child is detached
                setsid();
                // execute bash -lc "cmd"
                execl("/bin/bash", "bash", "-lc", cmd.c_str(), (char *)NULL);
                // if execl returns, it's an error
                _exit(127);
            }

            // parent
            RCLCPP_INFO(this->get_logger(), "Launched command[%zu] pid=%d", index, pid);
            // store pid
            if (pids_.size() <= index)
                pids_.resize(index + 1, 0);
            pids_[index] = pid;

            // wait for child to exit and log
            int status = 0;
            if (waitpid(pid, &status, 0) == pid)
            {
                if (WIFEXITED(status))
                {
                    int code = WEXITSTATUS(status);
                    RCLCPP_INFO(this->get_logger(), "command[%zu] pid=%d exited with %d", index, pid, code);
                }
                else if (WIFSIGNALED(status))
                {
                    int sig = WTERMSIG(status);
                    RCLCPP_INFO(this->get_logger(), "command[%zu] pid=%d killed by signal %d", index, pid, sig);
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "waitpid failed for pid %d", pid);
            }
        })
            .detach();
    }

    std::vector<std::string> commands_;
    std::vector<pid_t> pids_;
    std::mutex pids_mutex_;
    rclcpp::Service<Trigger>::SharedPtr reset_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProcessLauncher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
