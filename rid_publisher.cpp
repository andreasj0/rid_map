#include <iostream>
#include <map>
#include <variant>
#include <string>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Define a type alias for the variant
using values = std::variant<std::string, int, double>;

class RIDPublisher : public rclcpp::Node {
public:
    RIDPublisher() : Node("rid_publisher"), count_(0) {
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("rid_output", 10);

        // Timer to publish message every second
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                        std::bind(&RIDPublisher::publishMessage, this));
    }

private:
    void publishMessage() {
        // Map with string as key and variant(int, string, double) as values
        std::map<std::string, values> rid_data;

        // LocationMSG
        rid_data["track_direction"] = 3.14; // float
        rid_data["ground_speed"] = 3.14; // float
        rid_data["vertical_speed"] = 3.14; // float
        rid_data["latitude"] = 3.14; // float
        rid_data["longitude"] = 3.14; // float
        rid_data["geodetic_altitude"] = 3.14; // float
        rid_data["height"] = 3.14; // float
        rid_data["time_stamp"] = 3.14; // float
        rid_data["horizontal_accuracy"] = 3.14; // float
        rid_data["vertical_accuracy"] = 3.14; // float
        rid_data["ground_speed_accuracy"] = 3.14; // float
        // rid_data["mac_address"] = "123:123:abc:abc"; Don't see why we need two mac addresses when they are the same in both msgs

        // BasicIdMSG
        rid_data["UA_type"] = 7; // int
        rid_data["UAS_ID"] = "4"; //str
        rid_data["mac_address"] = "00:20:18:61:f1:8a"; //str

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

        // Publish the map as a JSON format
        std_msgs::msg::String msg;
        msg.data = oss.str();
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published message: %s", msg.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RIDPublisher>());
    rclcpp::shutdown();
    return 0;
}
