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

#include "gankenkun/node/gankenkun_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace gankenkun
{

GankenkunNode::GankenkunNode(const rclcpp::Node::SharedPtr & node)
: node(node), walking_manager(nullptr), walking_node(nullptr), config_node(nullptr)
{
  node_timer = node->create_wall_timer(8ms, [this]() {
    if (this->walking_manager && walking_node) {
      this->walking_manager->process();
      this->walking_node->update();
    }
  });
}

void GankenkunNode::set_walking_manager(const std::shared_ptr<WalkingManager> & walking_manager)
{
  this->walking_manager = walking_manager;
  walking_node = std::make_shared<WalkingNode>(node, walking_manager);
}

void GankenkunNode::run_config_service(const std::string & path)
{
  config_node = std::make_shared<ConfigNode>(node, walking_manager, path);
}

}  // namespace gankenkun
