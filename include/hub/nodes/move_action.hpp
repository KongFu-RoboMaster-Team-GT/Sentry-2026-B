#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

class MoveAction : public BT::StatefulActionNode {
public:
    // 正確的 V4 構造函數簽名
    MoveAction(const std::string& name, 
               const BT::NodeConfig& config, 
               rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), // 傳遞 config 給基類
      node_(node_ptr)
    {
    }

    // 必須實現這三個虛函數 (如果繼承自 StatefulActionNode)
    BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }
    BT::NodeStatus onRunning() override { return BT::NodeStatus::SUCCESS; }
    void onHalted() override { }

    static BT::PortsList providedPorts() { return {}; }

private:
    rclcpp::Node::SharedPtr node_;
};