#include "parse_dataset.h"

static std::optional<Record> parseRecordLine(const std::string& line) {
    // expected tokens with labels: time: <d> ticks: <u> <u> model_pose: a b c tracker_pose: a b c
    Record r;
    std::istringstream iss(line);
    std::string key;
    if (!(iss >> key)) return std::nullopt;
    if (key != "time:") return std::nullopt;
    if (!(iss >> r.time)) return std::nullopt;

    if (!(iss >> key) || key != "ticks:") return std::nullopt;
    uint64_t t1=0, t2=0;
    if (!(iss >> t1 >> t2)) return std::nullopt;
    r.ticks_steer = static_cast<uint32_t>(t1);
    r.ticks_trac  = static_cast<uint32_t>(t2);

    if (!(iss >> key) || key != "model_pose:") return std::nullopt;
    if (!(iss >> r.model_x >> r.model_y >> r.model_th)) return std::nullopt;

    if (!(iss >> key) || key != "tracker_pose:") return std::nullopt;
    if (!(iss >> r.track_x >> r.track_y >> r.track_th)) return std::nullopt;

    return r;
}

bool loadDataset(const std::string& path, Dataset& ds) {
    std::ifstream f(path);
    if (!f) return false;

    std::string line;
    while (std::getline(f, line)) {
        auto raw = trim(line);
        if (raw.empty()) continue;

        if (raw[0] == '#') {
            // header lines
            // parameters list (names not strictly needed since order is defined in comment)
            if (raw.find("#parameter_values:") == 0) {
                auto vals = splitBySpace(trim(raw.substr(std::string("#parameter_values:").size())));
                // or parse bracketed if present; in your sample they are plain numbers
                if (vals.size() >= 4) {
                    try {
                        ds.params.Ksteer      = std::stod(vals[0]);
                        ds.params.Ktraction   = std::stod(vals[1]);
                        ds.params.axis_length = std::stod(vals[2]);
                        ds.params.steer_offset= std::stod(vals[3]);
                    } catch (...) {}
                }
            } else if (raw.find("#joints_max_enc_values:") == 0) {
                auto vals = splitBySpace(trim(raw.substr(std::string("#joints_max_enc_values:").size())));
                for (auto& v : vals) {
                    try { ds.joints.values.push_back(std::stoll(v)); } catch (...) {}
                }
            } else if (raw.find("#joints_max_enc:") == 0) {
                // e.g., [ steering traction_wheel ]
                auto names = parseBracketedStrings(raw);
                ds.joints.names = names;
            } else if (raw.find("#\ttranslation:") == 0 || raw.find("# translation:") == 0) {
                auto v = parseBracketedDoubles(raw);
                if (v.size() == 3) { ds.laser.tx = v[0]; ds.laser.ty = v[1]; ds.laser.tz = v[2]; }
            } else if (raw.find("#\trotation:") == 0 || raw.find("# rotation:") == 0) {
                auto v = parseBracketedDoubles(raw);
                if (v.size() == 4) { ds.laser.qx = v[0]; ds.laser.qy = v[1]; ds.laser.qz = v[2]; ds.laser.qw = v[3]; }
            }
            continue;
        }

        // data record
        if (auto rec = parseRecordLine(raw); rec.has_value()) {
            ds.records.push_back(*rec);
        }
    }
    return true;
}
