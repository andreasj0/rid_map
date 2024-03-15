#include <iostream>
#include <map>
#include <variant>
#include <string>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Define a type alias for the variant
using values = std::variant<std::string, int, double>;

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("publisher_node");

    // Map with string as key and variant(int, string, double) as values
    std::map<std::string, values> rid_data;

    // Populate rid_data as in your original code

    // Iterate through map and return as JSON format
    std::ostringstream oss;
    oss << "{\n";
    for (auto it = rid_data.begin(); it != rid_data.end(); ++it) {
        oss << "  \"" << it->first << "\": ";
        std::visit([&](auto&& arg) {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, std::string>) {
                oss << "\"" << arg << "\"";
            } else {
                oss << arg;
            }
        }, it->second);
        if(std::next(it) != rid_data.end())
            oss << ",\n";
        else
            oss << "\n";
    }
    oss << "}\n";

    // Print the map as a JSON format
    std::string rid_output = oss.str();
    std::cout << rid_output << std::endl;

    // Create a ROS 2 publisher
    auto publisher = node->create_publisher<std_msgs::msg::String>("rid_output", 10);

    // Publish the string
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = rid_output;
    publisher->publish(*msg);

    rclcpp::shutdown();
    return 0;
}
