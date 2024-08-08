# simulation_benchmark
Benchmark comparison for rigid-body dynamic simulators. This project is part of GSoC'24@OpenRobotics. 

- [X] [Boxes benchmark: Free-floating rigid bodies:](https://github.com/yaswanth1701/simulation_benchmark/blob/gz-sim_dev/boxes_description.ipynb)

  <img src="img/boxes/boxesSimple.gif" width="395" height="240" />
  <img src="img/boxes/boxesComplex.gif" width="395" height="240" />
  
- [ ] [Triball benchmark: Rigid bodies in contact:]()

## Steps to run benchmark:
#### Install dependencies:
```bash
pip install lz4 protobuf zstandard matplotlib numpy pandas
```
#### Install currently implemented simulator:
- [X] Gazebo Ionic:
  
      
  **Installation**
  -  Follow the Gazebo Ionic source installation up to [here](https://gazebosim.org/docs/ionic/install_ubuntu_src/#:~:text=Use%20vcstool%20to%20automatically%20retrieve%20all%20the%20Gazebo%20libraries%20sources%20from%20their%20repositories%3A).
  -  Edit `collection-ionic.yaml.
  -  In `gz-sim version: main`, replace `main` with `scepters/set_model_state_prototype`.
   
  This is how it should look:
  ```bash
  gz-sim:
  type: git
  url: https://github.com/gazebosim/gz-sim
  version: scpeters/set_model_state_prototype
  ```
  Follow rest of the installation tutorial.

#### Build and run tests:

```bash 
git clone https://github.com/yaswanth1701/simulation_benchmark.git -b gz-sim_dev
cd benchmark
git submodule update --init --recursive
mkdir build 
cmake ..
make 
make test 
```

         
  


This project is an effort to create an open-source benchmarking suite for robotics physics engines/simulators (for example: gazebo, mujoco, and drake). 

This suite offers the following features:
- Logging ([mcap](https://github.com/foxglove/mcap)/csv)
- Dynamics world generation at run-time (SDF format).
- Simulator independent post-processing script.

These features allow for a simulator-independent suite for benchmarking.

Currently implemented simulators/physics:

- 
