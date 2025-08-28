# Probabilistic_Robotics_project-4-Calibration_of_a_-real-_Robot


## Overview

gg




## How to run the code
1. Clone the repo
2. Create the build folder inside the repo of the project and do cmake
   ```bash
   cd Probabilistic_Robotics_project-4-Calibration_of_a_-real-_Robot/
   mkdir build
   cd build
   cmake ..
   ```
   The correct structure of the repository is:
   ```bash
      Probabilistic_Robotics_project-4-Calibration_of_a_-real-_Robot/
      ├── 04-Calibration
      │   ├── dataset.txt
      │   ├── README.txt
      │   └── view_traj.py
      ├── build/              # Compiled files
      │   ├── calibration
      │   ├── calibrated_params.csv
      │   └── error_log.csv
      ├── include/            # Header files
      │   ├── parse_dataset.h
      │   ├── tricycle_model.h
      │   └── utils.h
      ├── src/                # Source code
      │   ├── calibration.cpp
      │   ├── parse_dataset.cpp
      │   ├── tricycle_model.cpp
      │   └── utils.cpp
      ├── scripts_py/            # Header files
      │   ├── requirements.txt
      │   ├── check_correction.py
      │   └── plot_error.py
      ├── CMakeLists.txt      
      └── README.md        
   ```
3. Make sure the `dataset.txt` has the same form of the one provided:
   ```bash
   #kinematic_model: ...
   #parameters: ...
   #parameter_values: ... 
   #joints_max_enc: ...
   #joints_max_enc_values: ... 
   #laser wrt base_link 
   #	translation:	...,
   #	rotation:	 ...
   time: ... ticks: ... model_pose: ... tracker_pose: ...
   ...
   ```
4. Compile and run the calibration executable with arg the `<path_to_the_dataset>`:
   ```bash
    make
    ./calibration ../04-Calibration/dataset.txt
   ```
5. This will generate in the `build` folder the files `calibrated_params.csv` and `error_log.csv`.  
   Plots can be visualized through:
   ```bash
   python ../scripts_py/check_correction.py
   python ../scripts_py/plot_error.py
   ```
