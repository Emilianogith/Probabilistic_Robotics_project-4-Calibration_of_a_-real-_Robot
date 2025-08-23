#include "utils.h"

double wrapToPi(double a) {
    a = std::remainder(a, TWO_PI);  // (-π, π]
    return a;
}

double wrapTwoPi(double a) {
    a = std::remainder(a, TWO_PI);  // (-π, π]
    if (a < 0) a += TWO_PI;         // (0, 2π]
    if (a == TWO_PI) a = 0.0;       // force half-open [0, 2π)
    return a;
}

void print_infos(Dataset& ds){
    std::cout << "Parameters:\n"
              << "  Ksteer=" << ds.params.Ksteer
              << " Ktraction=" << ds.params.Ktraction
              << " axis_length=" << ds.params.axis_length
              << " steer_offset=" << ds.params.steer_offset << "\n";

    std::cout << "Joints: ";
    for (size_t i=0;i<ds.joints.names.size();++i) {
        std::cout << (i? ", ":"") << ds.joints.names[i];
        if (i < ds.joints.values.size()) std::cout << "(" << ds.joints.values[i] << ")";
    }
    std::cout << "\n";

    std::cout << "Laser extrinsics (base->laser): t=[" << ds.laser.tx << "," << ds.laser.ty << "," << ds.laser.tz
              << "], q=[" << ds.laser.qx << "," << ds.laser.qy << "," << ds.laser.qz << "," << ds.laser.qw << "]\n";

    std::cout << "Loaded " << ds.records.size() << " records.\n";
    if (!ds.records.empty()) {
        const auto& r = ds.records.front();
        std::cout << "First record: time=" << r.time
                  << " ticks=(" << r.ticks_steer << "," << r.ticks_trac << ") "
                  << "model_pose=(" << r.model_x << "," << r.model_y << "," << r.model_th << ") "
                  << "tracker_pose=(" << r.track_x << "," << r.track_y << "," << r.track_th << ")\n";
    }


}



void get_sensor_reading(
    Dataset& ds, 
    int& index, 
    double& time, 
    int32_t& ticks_steer,
    int32_t& ticks_track, 
    Eigen::Vector2d& z){
    
    if (index < 0 || index >= static_cast<int>(ds.records.size())) {
        throw std::out_of_range("Index out of range in get_sensor_readings");
    }

    const auto& r = ds.records[index];
    const auto& prev_r = (index == 0) ? r : ds.records[index - 1];

    const uint32_t max_steer    = static_cast<uint32_t>(ds.joints.values[0]);
    const uint32_t max_traction = static_cast<uint32_t>(ds.joints.values[1]);
    
    time = r.time;
    ticks_steer = r.ticks_steer;    
    ticks_track = static_cast<int32_t>(r.ticks_trac - prev_r.ticks_trac);


    z(0) = r.track_x;
    z(1) = r.track_y;
}

void save_error(const std::string& filename, const std::vector<double>& error_log) {
    std::ofstream out(filename);
    out << "iter, error\n";
    out << std::fixed << std::setprecision(10);
    for (size_t i = 0; i < error_log.size(); ++i) {
        out << i << ", " << error_log[i] << "\n";
    }
}

void save_params(const std::string& filename, const Eigen::VectorXd& x) {
    std::ofstream out(filename);
    out << "param, value\n";
    out << std::fixed << std::setprecision(10);
    out << "x_s: " << x(0) << "\n";
    out << "y_s: " << x(1) << "\n";
    out << "K_steer: " << x(2) << "\n";
    out << "K_traction: " << x(3) << "\n";
    out << "baseline: " << x(4) << "\n";
    out << "steer_offset: " << x(5) << "\n";
    
}
