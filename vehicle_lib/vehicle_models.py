import math
import openrtdynamics2.lang as dy


#
# vehicle models
#

def discrete_time_bicycle_model(delta, v, Ts, wheelbase, x0=0.0, y0=0.0, psi0=0.0):
    """
        Implement an ODE solver (Euler) for the kinematic bicycle model equations

        x, y           - describes the position of the front axle,
        delta          - the steering angle
        v              - the velocity measured on the front axle
        wheelbase      - the distance between front- and rear axle
        Ts             - the sampling time for the Euler integration
        psi            - the orientation of the carbody
        (x0, y0, psi0) - the initial states of the ODEs
    """

    x   = dy.signal()
    y   = dy.signal()
    psi = dy.signal()

    # bicycle model
    x_dot   = v * dy.cos( delta + psi )
    y_dot   = v * dy.sin( delta + psi )
    psi_dot = v / dy.float64(wheelbase) * dy.sin( delta )

    # integrators
    x    << dy.euler_integrator(x_dot,   Ts, x0)
    y    << dy.euler_integrator(y_dot,   Ts, y0)
    psi  << dy.euler_integrator(psi_dot, Ts, psi0)

    return x, y, psi, x_dot, y_dot, psi_dot

def compute_acceleration( v, v_dot, delta, delta_dot, psi_dot ):
    """
        Compute the acceleration at the front axle
    """

    a_lat = v_dot * dy.sin( delta ) + v * ( delta_dot + psi_dot ) * dy.cos( delta )
    a_long = None

    return a_lat, a_long


def lateral_vehicle_model(u_delta, v, v_dot, Ts, wheelbase, x0=0.0, y0=0.0, psi0=0.0, delta0=0.0, delta_disturbance=None, delta_factor=None):
    """
        Bicycle model and the acceleration at the front axle
    """

    # add malicious factor to steering
    if delta_factor is not None:
        delta_ = u_delta * delta_factor
    else:
        delta_ = u_delta 

    # add disturbance to steering
    if delta_disturbance is not None:
        delta_ = delta_ + delta_disturbance

    # saturate steering
    delta = dy.saturate(u=delta_, lower_limit=-math.pi/2.0, upper_limit=math.pi/2.0)

    # bicycle model
    x, y, psi, x_dot, y_dot, psi_dot = discrete_time_bicycle_model(delta, v, Ts, wheelbase, x0, y0, psi0)

    #
    delta_dot = dy.diff( delta, initial_state=delta ) / dy.float64(Ts)
    delta = dy.delay( delta, delta0 )

    # compute acceleration in the point (x,y) in the vehicle frame
    a_lat, a_long = compute_acceleration( v, v_dot, delta, delta_dot, psi_dot )

    return x, y, psi, delta, delta_dot, a_lat, a_long, x_dot, y_dot, psi_dot

