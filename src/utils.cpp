#include "utils.h"



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