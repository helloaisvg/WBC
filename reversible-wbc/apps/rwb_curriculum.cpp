#include <filesystem>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std::string parse_string(int argc, char** argv, const std::string& name,
                         const std::string& fallback) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (argv[index] == name) {
            return argv[index + 1];
        }
    }
    return fallback;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string telemetry_dir =
        parse_string(argc, argv, "--telemetry",
                     "outputs/stress/telemetry");
    const std::string output_path =
        parse_string(argc, argv, "--out",
                     "outputs/stress/curriculum.csv");

    std::vector<std::filesystem::path> files;
    for (const auto& entry :
         std::filesystem::directory_iterator(telemetry_dir)) {
        if (entry.is_regular_file() &&
            entry.path().extension() == ".csv") {
            files.push_back(entry.path());
        }
    }
    if (files.empty()) {
        std::cerr << "no reversible telemetry found in "
                  << telemetry_dir << '\n';
        return 2;
    }

    const std::filesystem::path output(output_path);
    if (output.has_parent_path()) {
        std::filesystem::create_directories(output.parent_path());
    }
    std::ofstream out(output_path);
    if (!out) {
        std::cerr << "cannot write " << output_path << '\n';
        return 3;
    }
    out << "x,y,yaw,v,w,roll,payload_angle,payload_rate,joint_1,"
           "joint_2,joint_velocity_1,joint_velocity_2,payload_mass,"
           "friction,clearance,external_disturbance,requested_v,"
           "requested_w,requested_joint_1,requested_joint_2,"
           "requested_balance,event\n";

    int selected_rows = 0;
    int event_rows = 0;
    int boundary_rows = 0;
    for (const std::filesystem::path& file : files) {
        std::ifstream input(file);
        std::string line;
        if (!std::getline(input, line)) {
            continue;
        }
        const std::vector<std::string> header = split_csv(line);
        std::unordered_map<std::string, std::size_t> column;
        for (std::size_t index = 0; index < header.size(); ++index) {
            column[header[index]] = index;
        }
        std::vector<std::unordered_map<std::string, std::string>> rows;
        while (std::getline(input, line)) {
            const std::vector<std::string> values = split_csv(line);
            if (values.size() != header.size()) {
                continue;
            }
            std::unordered_map<std::string, std::string> row;
            for (std::size_t index = 0; index < header.size(); ++index) {
                row[header[index]] = values[index];
            }
            rows.push_back(std::move(row));
        }

        std::unordered_set<int> selected_steps;
        for (std::size_t index = 0; index < rows.size(); ++index) {
            const auto& row = rows[index];
            if (row.at("boundary_expanded") == "1") {
                selected_steps.insert(static_cast<int>(index));
                ++boundary_rows;
            }
            const bool failed =
                row.at("event") != "none" &&
                !row.at("event").empty();
            if (!failed) {
                continue;
            }
            ++event_rows;
            const std::size_t first =
                index > 120 ? index - 120 : 0;
            for (std::size_t preceding = first; preceding < index;
                 preceding += 4) {
                selected_steps.insert(static_cast<int>(preceding));
            }
            selected_steps.insert(static_cast<int>(index));
        }

        std::vector<int> ordered(selected_steps.begin(),
                                 selected_steps.end());
        std::sort(ordered.begin(), ordered.end());
        for (int index : ordered) {
            const auto& row = rows[static_cast<std::size_t>(index)];
            out << row.at("pre_x") << ',' << row.at("pre_y") << ','
                << row.at("pre_yaw") << ',' << row.at("pre_v") << ','
                << row.at("pre_w") << ',' << row.at("pre_roll") << ','
                << row.at("pre_payload_angle") << ','
                << row.at("payload_rate") << ','
                << row.at("joint_1") << ',' << row.at("joint_2") << ','
                << row.at("joint_velocity_1") << ','
                << row.at("joint_velocity_2") << ','
                << row.at("payload_mass") << ','
                << row.at("friction") << ','
                << row.at("clearance") << ','
                << row.at("external_disturbance") << ','
                << row.at("requested_v") << ','
                << row.at("requested_w") << ','
                << row.at("requested_joint_1") << ','
                << row.at("requested_joint_2") << ','
                << row.at("requested_balance") << ','
                << row.at("event") << '\n';
            ++selected_rows;
        }
    }

    std::cout << "curriculum rows=" << selected_rows
              << " failure_rows=" << event_rows
              << " boundary_rows=" << boundary_rows << '\n';
    std::cout << "artifact: " << output_path << '\n';
    return selected_rows > 0 ? 0 : 4;
}

