#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "hub/nodes/move_action.hpp"

// 獲取 XML 文件路徑的輔助函數（基於 ament_index）
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_node");

    BT::BehaviorTreeFactory factory;

    // 註冊
    factory.registerBuilder<MoveAction>(
        "MoveAction",
        [node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveAction>(name, config, node);
        }
    );

    // 讀取 XML
    std::string pkg_share = ament_index_cpp::get_package_share_directory("hub");
    std::string xml_file = pkg_share + "/bt_xml/main_tree.xml";
    
    auto tree = factory.createTreeFromFile(xml_file);

    // 循環 Tick
    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        tree.tickOnce();
        rclcpp::spin_some(node); // 處理 ROS 回調
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}