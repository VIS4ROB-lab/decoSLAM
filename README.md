# Cross-Agent Relocalization for Decentralized Collaborative SLAM

## About
This repository contains the official implementation of the paper "**Cross-Agent Relocalization for Decentralized Collaborative SLAM**" to be presented at ICRA 2023.


## Setup
This implementation has been tested using **[Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)** and the corresponding ROS distribution [**ROS Noetic Ninjemys**](http://wiki.ros.org/noetic). Please refer to the corresponding installation instructions.

Our system additionally depends on the following frameworks, which can be built and installed from source using CMake: 
- [OpenCV](https://github.com/opencv/opencv)
- [Ceres](http://ceres-solver.org/) (version 2.0 or higher)
- [OpenGV](http://laurentkneip.github.io/opengv/)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [Eigen3](https://eigen.tuxfamily.org)
- [cppzmq](https://github.com/zeromq/cppzmq)
- [glog](https://github.com/google/glog)

Finally, we make use of the [**Hyper**](https://github.com/VIS4ROB-lab/HyperSLAM) ecosystem for implementations of [sensor classes](https://github.com/VIS4ROB-lab/HyperSensors) as well as [Lie Groups](https://github.com/VIS4ROB-lab/HyperVariables). These are automatically fetched at configuration time and don't need to be installed manually.

After successfully setting up dependencies, the framework can be built using standard CMake procedure:

```
mkdir build && cd build
cmake ..
make -j8
```

## Reproducing the Paper Results
In order to reproduce the results from the EuRoC experiment of the paper, first download the rosbags for the Machine Hall sequences from the [dataset website](https://projects.asl.ethz.ch/datasets). Then, adapt the paths in the roslaunch file `evaluation/euroc/full_euroc.launch` to the location the bags are saved. Finally, launch the experiments using

```
python evaluation/euroc/launcher_euroc.py
```
This runs the five Machine Hall sequences concurrently at reduced speed. The experiment is run twice, once with map sharing among the team and once without map sharing. 

Result folders containing trajectory estimates, map data and communication data will be saved to `evaluation/euroc/configs/<configuration_name>/result_<timestamp>`. In order to get a meaningful trajectory estimate from the map sharing results, the trajectory files need to be postprocessed. For this purpose, run

```
./build/combine_poses <path_to_result_folder>
```
which produces a file `combined_poses.csv` which can be compared to ground truth using e.g. the [EVO](https://github.com/MichaelGrupp/evo) trajectory evaluation tool.

## Running a Custom Dataset
Running the following command spins up **one agent** from a config file:
```
./build/main <path_to_config_file>
```

Starting points for config files can be found under `evaluation/euroc/configs`. In order to generate your own runnable configuration, three steps need to be followed:

### 1. Extraction of NetVLAD Descriptors
We rely on pre-extracted NetVLAD descriptors (128 floats) that can be looked up at runtime. For this, we used the network from [netvlad_tf_open](https://github.com/uzh-rpg/netvlad_tf_open) to generate a .csv file containing timestamped descriptors (see e.g. `evaluation/euroc/descriptors/MH01/`) from the rosbag. This file is loaded at startup from the path set in the `descriptor_path` field of the main config.

### 2. Definition of the Sensor Setup
The sensor setup is defined from a separate config file  according to the format required by [*Hyper*Sensors](https://github.com/VIS4ROB-lab/HyperSensors) (see e.g. `evaluation/euroc/configs/euroc_stereo.yaml`). The path to the sensor configuration is set from the `sensor_config_file` field in the main config. 

Currently, only stereo setups are supported, where `cam0` is assumed to be the left camera and `cam1` the right camera. Furthermore, the `topic` field must be set to the rostopic where camera images are published. Note that in order to run multiple instances in the same machine, we prefix this field internally with an additional namespace `/agent_<id>` where `id` is the agent's id as defined in the main configuration file. **Topics exposed by your rosbag must be remapped accordingly!**

### 3. Definition of the Network
In order to spin up the peer-to-peer network, agents must load **the same network configuration file.** This file contains an entry for each agent, where it's IP is defined, along with a list of ports. You need to define as many ports per agent as there are agents in the system and the combination of port and IP needs to be unique. For an example with 5 agents, see `evaluation/euroc/configs/network_5.yaml`.

### Running the agents
Repeat step 1 from above for each sequence in the dataset. Then you can spin up all agents with the above command, possibly on different machines. Finally, run your rosbags concurrently using e.g. a `.launch` file (see `evaluation/euroc/full_euroc.launch` for an example).

## Contact
[Philipp Bänninger](mailto:baephili@ethz.ch)


## License

DecoSLAM is distributed under the [BSD-3-Clause License](LICENSE).


