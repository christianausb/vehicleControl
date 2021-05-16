import math
import numpy as np
from scipy import signal
import openrtdynamics2.lang as dy
import openrtdynamics2.lang.circular_buffer as cb
import matplotlib.pyplot as plt

from .path_data import *
from .geometry import *
from .vehicle_models import *
from .tracking import *
from .numpy_helper import *
from .path_following import *
from .safety import *
from .reference_algs import *


"""
A library for modelling and control of vehicles that can be descirbed by the kinematic bicycle model.

The methods are based on the paper

Klauer C., Schwabe M., and Mobalegh H., "Path Tracking Control for Urban Autonomous Driving", IFAC World Congress 2020, Berlin

It implements
- A kinematic model to describe the position of the front axle given velocity and steering
- Acceleration at the front axle
- Dynamic tracking of the closest point of the vehicle front axle to the path using a computationally efficient tracing algorithm 
- Path following control in which the steering angle is adjusted such that the front axle follows a given path. 
  The approach is independent of the driving velocity, which must be measured though and passed to the controller.
  A velocity controller for the vehicle can be implemented separately.

The implementation is based on the framework OpenRTDynamics 2 for modelling dynamic systems (Simulink-like, but text-based).
Hence, efficient c++ code can be generated.
"""

