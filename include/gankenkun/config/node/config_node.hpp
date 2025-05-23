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

#ifndef GANKENKUN__CONFIG__NODE__CONFIG_NODE_HPP_
#define GANKENKUN__CONFIG__NODE__CONFIG_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "gankenkun/walking/node/walking_manager.hpp"
#include "gankenkun_interfaces/srv/get_config.hpp"
#include "gankenkun_interfaces/srv/update_config.hpp"

namespace gankenkun
{

class ConfigNode
{
public:
  using GetConfig = gankenkun_interfaces::srv::GetConfig;
  using UpdateConfig = gankenkun_interfaces::srv::UpdateConfig;

  ConfigNode(const rclcpp::Node::SharedPtr & node, const std::shared_ptr<WalkingManager> & walking_manager, const std::string & path);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<WalkingManager> walking_manager;

  rclcpp::Service<GetConfig>::SharedPtr get_config_server;
  rclcpp::Service<UpdateConfig>::SharedPtr update_config_server;

};
  
} // namespace gankenkun

#endif  // GANKENKUN__CONFIG__NODE__CONFIG_NODE_HPP_
