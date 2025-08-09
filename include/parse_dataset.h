#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <optional>
#include <fstream> 
#include <sstream>   
#include <algorithm>
#include <cctype>

struct Parameters {
    double Ksteer = 0, Ktraction = 0, axis_length = 0, steer_offset = 0;
};

struct JointsMaxEnc {
    std::vector<std::string> names;     // e.g., {"steering","traction_wheel"}
    std::vector<long long> values;      // e.g., {8192, 5000}
};

struct LaserExtrinsics {
    double tx = 0, ty = 0, tz = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
};

struct Record {
    double time = 0;
    uint32_t ticks_steer = 0;
    uint32_t ticks_trac  = 0;
    double model_x=0, model_y=0, model_th=0;
    double track_x=0, track_y=0, track_th=0;
};

struct Dataset {
    Parameters params;
    JointsMaxEnc joints;
    LaserExtrinsics laser;
    std::vector<Record> records;
};

static inline std::string trim(std::string s) {
    auto notspace = [](int ch){ return !std::isspace(ch); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
    return s;
}

static std::vector<std::string> splitBySpace(const std::string& s) {
    std::istringstream iss(s);
    std::vector<std::string> out;
    std::string tok;
    while (iss >> tok) out.push_back(tok);
    return out;
}

static std::vector<std::string> parseBracketedStrings(const std::string& line) {
    auto l = line.find('['), r = line.find(']');
    std::vector<std::string> out;
    if (l == std::string::npos || r == std::string::npos || r <= l) return out;
    std::string inside = line.substr(l+1, r-l-1);
    // split by delimiters , and space
    std::string token;
    std::istringstream iss(inside);
    while (std::getline(iss, token, ',')) {
        token = trim(token);
        if (!token.empty()) out.push_back(token);
    }
    // if no commas, try spaces
    if (out.size() <= 1) {
        out = splitBySpace(inside);
        for (auto& t : out) t = trim(t);
    }
    return out;
}

static std::vector<double> parseBracketedDoubles(const std::string& line) {
    std::vector<double> out;
    for (auto& s : parseBracketedStrings(line)) {
        try { out.push_back(std::stod(s)); } catch (...) {}
    }
    return out;
}

static std::optional<Record> parseRecordLine(const std::string& line);

bool loadDataset(const std::string& path, Dataset& ds);
