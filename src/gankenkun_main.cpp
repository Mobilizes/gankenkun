// Copyright (c) 2025 ICHIRO ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "gankenkun/gankenkun.hpp"

int main(int argc, char * argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (args.size() < 2) {
    std::cerr << "Missing config path!" << std::endl;

    return 0;
  }

  const std::string & path = args[1];

  auto node = std::make_shared<rclcpp::Node>("gankenkun");
  auto gankenkun_node = std::make_shared<gankenkun::GankenkunNode>(node);

  auto walking_manager = std::make_shared<gankenkun::WalkingManager>();
  walking_manager->load_config(path);

  gankenkun_node->set_walking_manager(walking_manager);
  gankenkun_node->run_config_service(path);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();

  return 0;
}