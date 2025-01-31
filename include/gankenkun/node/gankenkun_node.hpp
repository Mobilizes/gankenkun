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

#ifndef GANKENKUN__NODE__GANKENKUN_NODE_HPP_
#define GANKENKUN__NODE__GANKENKUN_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "gankenkun/config/node/config_node.hpp"
#include "gankenkun/walking/node/walking_node.hpp"

namespace gankenkun
{

class GankenkunNode
{
public:
  GankenkunNode(const rclcpp::Node::SharedPtr & node);

  void set_walking_manager(const std::shared_ptr<WalkingManager> & walking_manager);

  void run_config_service(const std::string & path);

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<WalkingNode> walking_node;
  std::shared_ptr<ConfigNode> config_node;

  rclcpp::TimerBase::SharedPtr node_timer;
};

}  // namespace gankenkun

#endif  // GANKENKUN__NODE__GANKENKUN_NODE_HPP_
