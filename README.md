# A tutorial on vehicle control

-- upcoming --

Tutorial on: Vehicle path following and control

This project considers example implementations and demonstrations for several aspects of car-like vehicle control, e.g., for autonomous driving purposes. The examples are implmented using model-based programming in python. 

Topics might be (to be further defined)
- Kinematic Bicycle Model (KBM) ✅ [path-following control](https://christianausb.github.io/vehicleControl/bicycle_model.html) 
- Path following (basic)  ✅ [path-following control](https://christianausb.github.io/vehicleControl/path_following_control.html) 
- Path following (improved with curvature profile) ✅ [path-curvature-following control](https://christianausb.github.io/vehicleControl/path_curvature_following_control.html) 
- Path following (open-loop) ✅ [path-following open-loop control](https://christianausb.github.io/vehicleControl/path_following_open_loop_control.html) 
- Trajectory tracking control
- Frenet transformation of the KBM
- Motion planning
- Lateral/Longitudinal acceleration
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

