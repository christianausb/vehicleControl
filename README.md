# A tutorial on vehicle control

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/christianausb/vehicleControl/HEAD)

-- upcoming --

Tutorial on: Vehicle path following and control

This project considers example implementations and demonstrations for several aspects of car-like vehicle control, e.g., for autonomous driving purposes. The examples are implemented using model-based programming in python. 

Topics might be (to be further defined)

- Kinematic Bicycle Model (KBM) ✅ [bicycle-model](https://christianausb.github.io/vehicleControl/bicycle_model.html) 

- Lateral/Longitudinal acceleration

- Path following (basic)  ✅ [path-following control](https://christianausb.github.io/vehicleControl/path_following_control.html) 

- Path following (improved with curvature profile) ✅ [path-following control (curvature-based)](https://christianausb.github.io/vehicleControl/path_curvature_following_control.html) 

- Lateral dynamics ✅ [lateral dynamics](https://christianausb.github.io/vehicleControl/path_following_lateral_dynamics.html) 

- Path following (open-loop) ✅ [path-following open-loop control](https://christianausb.github.io/vehicleControl/path_following_open_loop_control.html) 

- Trajectory tracking control
- Frenet transformation of the KBM
- Motion planning
- Safety Functions
- Trailers


Requirements
------------

- openrtdynamics2 (tool for c++ code generation): $ pip install openrtdynamics2
- emscripten (https://emscripten.org/), the compiler command 'emcc' is required to compile the generated c++ code into web assembly.


Run the examples
----------------

Start a http-server to be able to access the *.html files, e.g.:

python3 -m http.server

