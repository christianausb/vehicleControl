import math
import numpy as np
from scipy import signal
import openrtdynamics2.lang as dy
import openrtdynamics2.lang.circular_buffer as cb
import matplotlib.pyplot as plt





#
# Path storage and access
#

def import_path_data(data):
    """
        Create a data structure containing the driving path 
    """
    # distance on path (D), position (X/Y), path orientation (PSI), curvature (K)
    path = {}
    path['buffer_type']   = 'dy.memory'

    path['D']   = dy.memory(datatype=dy.DataTypeFloat64(1), constant_array=data['D'] )
    path['X']   = dy.memory(datatype=dy.DataTypeFloat64(1), constant_array=data['X'] )
    path['Y']   = dy.memory(datatype=dy.DataTypeFloat64(1), constant_array=data['Y'] )
    path['PSI'] = dy.memory(datatype=dy.DataTypeFloat64(1), constant_array=data['PSI'] )
    path['K']   = dy.memory(datatype=dy.DataTypeFloat64(1), constant_array=data['K'] )

    path['samples'] = len( data['D'] )

    return path


def create_path_horizon(horizon_length : int = 100):

    # distance on path (D), position (X/Y), path orientation (PSI), curvature (K)
    path = {}
    path['buffer_type']   = 'circular_buffer'

    path['D']   = cb.new_circular_buffer_float64( horizon_length )
    path['X']   = cb.new_circular_buffer_float64( horizon_length )
    path['Y']   = cb.new_circular_buffer_float64( horizon_length )
    path['PSI'] = cb.new_circular_buffer_float64( horizon_length )
    path['K']   = cb.new_circular_buffer_float64( horizon_length )

    path['horizon_length'] = horizon_length

    return path

def append_to_path_horizon(path, path_sample):
    """
        Append one sample to the path horizon
    """

    cb.append_to_buffer( path['D'],   path_sample['d']   )
    cb.append_to_buffer( path['X'],   path_sample['x']   )
    cb.append_to_buffer( path['Y'],   path_sample['y']   )
    cb.append_to_buffer( path['PSI'], path_sample['psi'] )
    cb.append_to_buffer( path['K'],   path_sample['K']   )

def path_horizon_head_index(path):
    """
        Get the current head-index position in the horizon and the distance at the head
    """

    if path['buffer_type']  == 'dy.memory':
        head_index                     = dy.int32( path['samples'] - 1 )
        distance_at_the_end_of_horizon = dy.memory_read( memory=path['D'],   index=head_index ) 

    elif path['buffer_type']  == 'circular_buffer':
        head_index                     = cb.get_current_absolute_write_index(path['D']) - 1
        distance_at_the_end_of_horizon = cb.read_from_absolute_index(path['D'], head_index)

    return head_index, distance_at_the_end_of_horizon


def path_horizon_tail_index(path):
    """
        Get the current tail-index position in the horizon and the distance at the tail
    """

    if path['buffer_type']  == 'dy.memory':
        tail_index                       = dy.int32( 0 )
        distance_at_the_begin_of_horizon = dy.memory_read( memory=path['D'],   index=tail_index ) 

    elif path['buffer_type']  == 'circular_buffer':
        tail_index                     = cb.get_absolute_minimal_index(path['D'])
        distance_at_the_begin_of_horizon = cb.read_from_absolute_index(path['D'], tail_index)

    return tail_index, distance_at_the_begin_of_horizon






def sample_path(path, index):
    """
        Read a sample of the given path at a given array-index
    """

    if path['buffer_type']  == 'dy.memory':
        d   = dy.memory_read( memory=path['D'],   index=index ) 
        x   = dy.memory_read( memory=path['X'],   index=index ) 
        y   = dy.memory_read( memory=path['Y'],   index=index ) 
        psi = dy.memory_read( memory=path['PSI'], index=index )
        K   = dy.memory_read( memory=path['K'],   index=index )

        return d, x, y, psi, K

    elif path['buffer_type']  == 'circular_buffer':
        d   = cb.read_from_absolute_index(path['D'],   index)
        x   = cb.read_from_absolute_index(path['X'],   index)
        y   = cb.read_from_absolute_index(path['Y'],   index)
        psi = cb.read_from_absolute_index(path['PSI'], index)
        K   = cb.read_from_absolute_index(path['K'],   index)

        return d, x, y, psi, K

def sample_path_xy(path, index):
    """
        Read a sample (x,y) of the given path at a given array-index
    """

    if path['buffer_type']  == 'dy.memory':
        y   = dy.memory_read( memory=path['Y'], index=index ) 
        x   = dy.memory_read( memory=path['X'], index=index ) 

        return x, y

    elif path['buffer_type']  == 'circular_buffer':
        x   = cb.read_from_absolute_index(path['X'],   index)
        y   = cb.read_from_absolute_index(path['Y'],   index)

        return x, y

def sample_path_d(path, index):
    """
        Read a sample (d) of the given path at a given array-index
    """

    if path['buffer_type']  == 'dy.memory':
        d   = dy.memory_read( memory=path['D'], index=index ) 

        return d

    elif path['buffer_type']  == 'circular_buffer':
        d   = cb.read_from_absolute_index(path['D'],   index)

        return d



def sample_path_finite_difference(path, index):
    """
        Compute path orientation angle form x/y data only using finite differences
    """

    x1, y1 = sample_path_xy(path, index)
    x2, y2 = sample_path_xy(path, index + 1)

    Delta_x = x2 - x1
    Delta_y = y2 - y1

    psi_r = dy.atan2(Delta_y, Delta_x)
    x_r = x1
    y_r = y1

    return x_r, y_r, psi_r



def linear_interpolation(v1, v2, interpolation_factor):
    return v1 + (v2 - v1) * interpolation_factor

def sample_path_linear_interpolation(path, i_s, i_e, interpolation_factor):

    d_s, x_s, y_s, psi_s, K_s = sample_path(path, i_s)
    d_e, x_e, y_e, psi_e, K_e = sample_path(path, i_e)

    d   = linear_interpolation( d_s,   d_e,   interpolation_factor )
    x   = linear_interpolation( x_s,   x_e,   interpolation_factor )
    y   = linear_interpolation( y_s,   y_e,   interpolation_factor )
    psi = linear_interpolation( psi_s, psi_e, interpolation_factor )
    K   = linear_interpolation( K_s,   K_e,   interpolation_factor )

    return d, x, y, psi, K



def load_path_from_cvs_TUMFTM(filename : str, delimiter=';'):
    """
        read CVS data as produced by 
        https://github.com/TUMFTM/global_racetrajectory_optimization
    """
    A = np.genfromtxt(filename, delimiter=delimiter)
        
    path = {
        'D' :   A[:,0],
        'X' :   A[:,1],
        'Y' :   A[:,2],
        'PSI' : A[:,3] + np.deg2rad(90), # TUMFTM uses a slightly different definition of the angle psi
        'K' :   A[:,4]
    }
    
    if np.size(A, 1) > 5:
        path['V_R'] =  A[:,5]
        path['A_R'] =  A[:,6]
    
    return path


def plot_path(path, show_xy = True, show_curvature = True, show_steering = True):
   # time = make_time(Ts, path_x)

    if show_xy:
        plt.figure(figsize=(6,6), dpi=100)
        plt.plot( path['X'], path['Y'] )

        plt.plot( path['X'][0], path['Y'][0], 'r+', label='begin' )
        plt.plot( path['X'][-1], path['Y'][-1], 'k+', label='end' )
        plt.plot( path['X'][10], path['Y'][10], 'g+', label='begin + 10 samples' )

        plt.legend()
        plt.grid()

        plt.show()




    if show_steering:

        if 'V_DELTA' in path and 'V_DELTA_DOT' in path:
            plt.figure(figsize=(12,3), dpi=100)
            plt.plot(path['D'], np.rad2deg( path['V_DELTA']),     'r' )
            plt.plot(path['D'], np.rad2deg( path['V_DELTA_DOT']), 'k' )
            plt.plot(path['D'], np.rad2deg( path['K']),           'g' )

            plt.legend(['steering angle ($\delta$) [${}^\circ$]', 'steering rate ($\dot\delta$) [${}^\circ / s$]', 'curvature ($\kappa$) [${}^\circ / m$]'])
            plt.xlabel('distance along output path ($d$) [m]')
            plt.ylabel('steering angle  / steering rate ')
            plt.grid()

            plt.show()

    if show_curvature:
        plt.figure(figsize=(12,2), dpi=100)
        plt.plot(path['D'], np.rad2deg( path['PSI']), 'r' )

        plt.legend(['path orientation angle ($\Psi_r$)'])
        plt.xlabel('distance along output path ($d$) [m]')
        plt.ylabel('angle [${}^\circ$]')
        plt.grid()

        plt.show()


        plt.figure(figsize=(12,2), dpi=100)
        plt.plot(path['D'], np.rad2deg( path['K']),   'g' )

        plt.legend(['curvature ($\kappa = {\partial}/{\partial d} \, \Psi_r$)'])
        plt.xlabel('distance along output path ($d$) [m]')
        plt.ylabel('curvature [${}^\circ / m$]')
        plt.grid()

        plt.show()
    
