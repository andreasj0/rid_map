#include <iostream>
#include <map>
#include <variant>
#include <string>
#include <sstream>

// Define a type alias for the variant
using values = std::variant<std::string, int, double>;

int main() {
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
    // rid_data["mac_address"] = "123:123:abc:abc"; Dont see why we need two mac addresses when they are the same in both msgs

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

    // Print the JSON string
    std::cout << oss.str() << std::endl;
    
    return 0;
}

