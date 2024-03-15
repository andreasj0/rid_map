#include <iostream>
#include <map>
#include <string>

struct valuess {
    int int_values;
    std::string string_values;
    double double_values;
};

int main() {
    std::map<std::string, valuess> rid_data;

    // Adding elements to the map
    rid_data["heading"] = {0, "Hello", 3.14};
    rid_data["mac address"] = {20, "World", 6.28};

    for (const auto& pair : rid_data) {
        std::cout << "Key: " << pair.first << ", values: " << pair.second.int_values << ", " << pair.second.string_values << ", " << pair.second.double_values << std::endl;
    }
    return 0;
}
