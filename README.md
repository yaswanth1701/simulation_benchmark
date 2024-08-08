# simulation_benchmark
Benchmark comparison for rigid-body dynamic simulators. This project is part of GSoC'24@OpenRobotics. 

This project is an effort to create an open-source benchmarking suite for robotics physics engines/simulators (for example: gazebo, mujoco, and drake). 
- [X] [Boxes benchmark: Free-floating rigid bodies:](boxes_description.ipynb)

  <img src="img/boxes/boxesSimple.gif" width="395" height="240" />
  <img src="img/boxes/boxesComplex.gif" width="395" height="240" />
  
- [ ] [Triball benchmark: Rigid bodies in contact:]()

## Steps to run benchmark:
#### Install dependencies:
```bash
pip install lz4 protobuf zstandard matplotlib numpy pandas
```
#### Install currently implemented simulators:
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
git clone https://github.com/yaswanth1701/simulation_benchmark.git
cd benchmark
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make 
make test 
```
#### Run Python notebooks for results
- [boxes_results.ipynb](https://github.com/yaswanth1701/simulation_benchmark/blob/gz-sim_dev/boxes_results.ipynb)

## Benchmark suite feature description:
#### Benchmark components:
- `Simulation` -> `mcap to csv log conversion` -> `post_processing` -> `result_plots`
#### This suite offers the following features:
- Logging ([mcap](https://github.com/foxglove/mcap) & csv formats)
- Dynamics world generation at run-time (SDF format).
- Simulator independent post-processing script.

These features allow for a simulator-independent suite for benchmarking.

#### Currently implemented simulators/physics engines:

-  [Gazebo Ionic benchmark:](https://github.com/gazebosim/gz-ionic)
      
   - [X] [DART](https://github.com/dartsim/dart)
   - [ ] [Bullet](https://github.com/bulletphysics/bullet3)
   - [X] [Bullet-Featherstone](https://github.com/bulletphysics/bullet3)

## Future works:
- Benchmarks with the model having joints and multi-body dynamics.
- Integration of Mujoco and Drake.

## Acknowledgment:
We would highly appreciate developers of open-source simulators and the robotics community suggesting or implementing new benchmark ideas using this benchmarking suite and opening a pull request for it. If anyone comes across problems while running the benchmark, please inform us by opening an issue.
