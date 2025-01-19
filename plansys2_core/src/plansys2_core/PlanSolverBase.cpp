// Copyright 2025 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>

#include <filesystem>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <thread>
#include <sstream>

#include "plansys2_core/PlanSolverBase.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace plansys2
{

char**
PlanSolverBase::tokenize(const std::string & command)
{
  std::vector<std::string> tokens;
  std::istringstream stream(command);
  std::string token;

  while (stream >> std::quoted(token)) {
    tokens.push_back(token);
  }

  char** argv = new char*[tokens.size() + 2];

  for (size_t i = 0; i < tokens.size(); ++i) {
    argv[i] = new char[tokens[i].size() + 1];
    std::strcpy(argv[i], tokens[i].c_str());
  }

  argv[tokens.size()] = nullptr; // Null-terminate the array

  return argv;
}

bool 
PlanSolverBase::execute_planner(const std::string & command,
  const rclcpp::Duration & solver_timeout, const std::string & plan_path)
{
  cancel_requested_ = false;
  bool child_finish = false;

  int pipe_fd[2];
  pipe(pipe_fd);

  auto start = lc_node_->now();

  pid_t pid = fork();

  if (pid == 0) {
    close(pipe_fd[0]);
    dup2(pipe_fd[1], STDOUT_FILENO);
    close(pipe_fd[1]);

    auto cmd_tokens = tokenize(command);
    execvp(cmd_tokens[0], cmd_tokens);

    exit(EXIT_FAILURE);
  } else {
    std::thread monitor_thread([&]() {
      while (!cancel_requested_ && !child_finish &&
        lc_node_->now() - start < solver_timeout)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      kill(pid, SIGKILL);
    });

    close(pipe_fd[1]);
    int output_fd = open(plan_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (output_fd == -1) {
      RCLCPP_ERROR(lc_node_->get_logger(), "Open failed");
      close(pipe_fd[0]);
    
      child_finish = true;
      monitor_thread.join();
      return false;
    }

    char buffer[4096];
    ssize_t bytes_read;
    while ((bytes_read = read(pipe_fd[0], buffer, sizeof(buffer))) > 0) {
      write(output_fd, buffer, bytes_read);
    }

    close(pipe_fd[0]);
    close(output_fd);
  
    int status;
    waitpid(pid, &status, 0);

    child_finish = true;
    monitor_thread.join();

    if (WIFSIGNALED(status) && WTERMSIG(status) == SIGKILL) {
      RCLCPP_DEBUG(lc_node_->get_logger(), "Child process was terminated by cancel request.");
      return false;
    } else if (WIFEXITED(status)) {
      RCLCPP_DEBUG_STREAM(
        lc_node_->get_logger(), "Child process exited with status: " << WEXITSTATUS(status));
      
      if (WEXITSTATUS(status) != 0) {return false;};
    }
  }
  return true;
}


}  // namespace plansys2

