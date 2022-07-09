import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math

from vehicle_lib.numpy_helper import np_normalize_angle_mpi_to_pi, rotate_vector_2d


def simulate_odometry(x_y_psi):
    pdf_odometry_measurements = pd.DataFrame()

    # simulate odometry
    x_y_psi_delta = np.diff( x_y_psi, axis=0 )
    d_delta   = np.sqrt( x_y_psi_delta[:,0]**2 + x_y_psi_delta[:,1]**2 )
    psi_delta = x_y_psi_delta[:,2]

    d_psi_delta = np.array([ d_delta, psi_delta  ]).transpose()  # aka odemetry
    
    
    pdf_odometry_measurements["d_delta"] = d_delta
    pdf_odometry_measurements["psi_delta"] = psi_delta
    
    # define measurement noise parameters
    pdf_odometry_measurements["d_sigma"] = 0.01
    pdf_odometry_measurements["psi_sigma"] = 0.01
    
    return pdf_odometry_measurements


def simulate_odometry_and_GPS(
    raw_trace, 
    number_of_samples     : int = 100, 
    number_of_gps_samples : int = 5, 
    xy_gps_sigma  = math.sqrt( 1.0 ),
    psi_gps_sigma = math.sqrt( 0.1 ),
):
    pdf_gps_measurements = pd.DataFrame()

    n_raw = len(raw_trace['X'])

    # perform a subsampling of the ground truth to reduce the data and to 
    # model the reduced sampling rate of the odometry.
    I_subsample_raw = np.linspace(0, n_raw-1, number_of_samples, dtype=np.int32)

    x_y_psi__ = np.array( [ raw_trace['X'], raw_trace['Y'], raw_trace['PSI'] ] ).transpose()
    x_y_psi = x_y_psi__[ I_subsample_raw, : ]

    # pass-through the (sub-sampled) ground truth (used for plotting verification purposes not to solve the SLAM problem)    
    pdf_vehicle_trace_gt = pd.DataFrame()
    pdf_vehicle_trace_gt["x"]   = x_y_psi[:,0]
    pdf_vehicle_trace_gt["y"]   = x_y_psi[:,1]
    pdf_vehicle_trace_gt["psi"] = x_y_psi[:,2]

    #
    # simulate odometry
    #
    
    # compute odometry
    pdf_odometry_measurements = simulate_odometry( x_y_psi )
    
    

    #
    # simulate GPS
    #

    # perform a subsampling of the ground truth to model the reduced sampling rate of GPS.
    gps_subsample_indices = np.linspace( 1, len(x_y_psi)-1, number_of_gps_samples, dtype=np.int32 )
    
    #trace['gps_subsample_indices'] = gps_subsample_indices
    x_y_psi_GPS = x_y_psi[gps_subsample_indices] # TODO: add measurement noise  
    
    #trace['x_y_psi_GPS'] = x_y_psi_GPS
    
    np.random.seed(1)
    
    pdf_gps_measurements["x"]   = x_y_psi_GPS[:,0] + np.random.normal(scale=xy_gps_sigma,  size=len(gps_subsample_indices) )
    pdf_gps_measurements["y"]   = x_y_psi_GPS[:,1] + np.random.normal(scale=xy_gps_sigma,  size=len(gps_subsample_indices) )
    pdf_gps_measurements["psi"] = x_y_psi_GPS[:,2] + np.random.normal(scale=psi_gps_sigma, size=len(gps_subsample_indices) )
    pdf_gps_measurements["index_in_trace"] = gps_subsample_indices

    # define measurement noise parameters
    pdf_gps_measurements["x_sigma"]   = xy_gps_sigma
    pdf_gps_measurements["y_sigma"]   = xy_gps_sigma
    pdf_gps_measurements["psi_sigma"] = psi_gps_sigma
    
    return pdf_odometry_measurements, pdf_gps_measurements, pdf_vehicle_trace_gt


def sense_landmark_on_given_trace( 
    pdf_vehicle_trace_gt,
    landmark_xy, 
    field_of_view_angle=np.deg2rad(80), 
    field_of_view_max_distance=8 
):
    
    # simulate landmark detecting sensor
    delta_x = landmark_xy[0] - pdf_vehicle_trace_gt.x.to_numpy()
    delta_y = landmark_xy[1] - pdf_vehicle_trace_gt.y.to_numpy()

    bearing_angle    = np_normalize_angle_mpi_to_pi(
        np.arctan2( delta_y, delta_x ) - pdf_vehicle_trace_gt.psi.to_numpy()
    )
    bearing_distance = np.sqrt( delta_x**2 + delta_y**2 )

    I_seen_in_front_of_vehicle = np.where( 
        np.logical_and(
            bearing_angle <   field_of_view_angle, 
            bearing_angle > - field_of_view_angle
        )  
    )[0]
    
    I_tmp = np.where( bearing_distance[I_seen_in_front_of_vehicle] < field_of_view_max_distance )[0]

    I_low_distance_and_visible = I_seen_in_front_of_vehicle[I_tmp]

    #
    n_observations_per_landmark = len(I_low_distance_and_visible)
    
    return n_observations_per_landmark, I_low_distance_and_visible, bearing_angle[I_low_distance_and_visible], bearing_distance[I_low_distance_and_visible]

def simulate_landmarks_sesing(pdf_vehicle_trace_gt, pdf_landmarks_ground_truth):

    pdf_landmarks_bearing = pd.DataFrame()

    for i, row in enumerate(pdf_landmarks_ground_truth.itertuples()):

        landmark_id    = row.landmark_id
        landmark_xy_gt = [ row.x, row.y ]

        n_observations_per_landmark, I_low_distance_and_visible, bearing_angle, bearing_distance = sense_landmark_on_given_trace( 
            pdf_vehicle_trace_gt,
            landmark_xy_gt
        )

        pdf_landmark_bearing = pd.DataFrame()
        pdf_landmark_bearing["landmark_id"]                                = landmark_id*np.ones(n_observations_per_landmark, dtype=np.int64)
        pdf_landmark_bearing["index_in_trace_where_landmark_was_seen"]     = I_low_distance_and_visible
        pdf_landmark_bearing["bearing_angle"]                              = bearing_angle
        pdf_landmark_bearing["bearing_distance"]                           = bearing_distance

        pdf_landmark_bearing["bearing_angle_sigma"]                        = 0.01
        pdf_landmark_bearing["bearing_distance_sigma"]                     = 0.4

        # concatenate the data for each landmark
        if i==0:
            pdf_landmarks_bearing = pdf_landmark_bearing
        else:
            pdf_landmarks_bearing = pd.concat( ( pdf_landmarks_bearing, pdf_landmark_bearing ), axis=0  )

    return pdf_landmarks_bearing


class Plots:
    
    @classmethod
    def _compute_positions_from_which_the_landmark_is_visible(cls, pdf_landmarks_bearing, x_y_psi, landmark_id_to_show):

        # get the indices of the samples in the trace at which the landmark was seen.
        I_low_distance_and_visible = pdf_landmarks_bearing[ 
            pdf_landmarks_bearing.landmark_id == landmark_id_to_show 
        ]['index_in_trace_where_landmark_was_seen']

        # lookup the positions in the vehicle trace
        x_y_psi_visible = x_y_psi[I_low_distance_and_visible]

        return x_y_psi_visible

    @classmethod
    def _draw_beams(cls, x_y_psi_visible, landmark_xy_gt, beam_color):
        # construct beams
        x_y_visible = x_y_psi_visible[:, 0:2]
        N = x_y_psi_visible.shape[0]
        tmp  = np.tile(landmark_xy_gt, N ).reshape( (N,2)  )
        nans = np.tile([ np.nan, np.nan ], N ).reshape( (N,2)  )
        plot_data = np.column_stack( (x_y_visible, tmp, nans ) ).reshape( ( N*3,2 ) )

        plt.plot( 
            plot_data[ :,0 ], plot_data[ :,1 ], 
            color=beam_color
        )

    @classmethod
    def draw_positions_from_which_the_landmark_is_visible(cls, x_y_psi_visible, marker_style, color, landmark_id_to_show):
        plt.plot( 
            x_y_psi_visible[ :,0 ], x_y_psi_visible[ :,1 ], 
            '+', marker=marker_style,
            color=color, label="visibility of landmark "+str(landmark_id_to_show) 
        ) 

    @classmethod
    def add_plot_positions_on_trace_and_beams(cls, pdf_vehicle_trace_gt, pdf_landmarks_bearing, landmark_xy_gt, landmark_id_to_show, color, beam_color, marker_style):
        # callback for for_each_landmark
        # get ground truth vehicle trace
        x_y_psi = np.array([ 
            pdf_vehicle_trace_gt.x.to_numpy(), 
            pdf_vehicle_trace_gt.y.to_numpy(),
            pdf_vehicle_trace_gt.psi.to_numpy(),
        ]).transpose()
        
        # draw positions on trace where the landmark is visible
        x_y_psi_visible = cls._compute_positions_from_which_the_landmark_is_visible(pdf_landmarks_bearing, x_y_psi, landmark_id_to_show)
        cls.draw_positions_from_which_the_landmark_is_visible(x_y_psi_visible, marker_style, color, landmark_id_to_show)

        # construct beams
        cls._draw_beams(x_y_psi_visible, landmark_xy_gt, beam_color)

    @classmethod
    def add_plot_landmark_markers(cls, pdf_vehicle_trace_gt, pdf_landmarks_bearing, landmark_xy_gt, landmark_id_to_show, color, beam_color, marker_style):
        # callback for for_each_landmark

        plt.plot( 
            landmark_xy_gt[0], landmark_xy_gt[1], 
            '+', marker=marker_style, color=color, markersize=12,
            label="landmark "+str(landmark_id_to_show) 
        )      

    @classmethod
    def for_each_landmark(cls, pdf_vehicle_trace_gt, pdf_landmarks_bearing, pdf_landmarks_ground_truth, landmarks_to_show, lambda_fn):
        for lshow in landmarks_to_show:
            landmark_id_to_show, color, beam_color, marker_style = lshow[0], lshow[1], lshow[2], lshow[3]        
            landmark_gt = pdf_landmarks_ground_truth[ pdf_landmarks_ground_truth["landmark_id"] == landmark_id_to_show ].iloc[0]
            landmark_xy_gt = np.array([ landmark_gt.x, landmark_gt.y ])

            lambda_fn( pdf_vehicle_trace_gt, pdf_landmarks_bearing, landmark_xy_gt, landmark_id_to_show, color, beam_color, marker_style )

              
    
                
    @classmethod
    def plot_overview(
        cls,
        pdf_vehicle_trace_gt, 
        pdf_gps_measurements, 
        pdf_landmarks_ground_truth, 
        pdf_landmarks_bearing, 
        landmarks_to_show, 
        figsize=(12, 7)
    ):

        plt.figure(figsize=figsize, dpi=100)
        plt.plot( pdf_vehicle_trace_gt.x, pdf_vehicle_trace_gt.y, 'k', color="lightgrey", label="vehicle trace (ground truth)" )    
        plt.plot( pdf_gps_measurements.x, pdf_gps_measurements.y, 'o', color="grey", markersize=12, label="GPS sample" )

        cls.for_each_landmark(pdf_vehicle_trace_gt, pdf_landmarks_bearing, pdf_landmarks_ground_truth, landmarks_to_show, cls.add_plot_positions_on_trace_and_beams)
        cls.for_each_landmark(pdf_vehicle_trace_gt, pdf_landmarks_bearing, pdf_landmarks_ground_truth, landmarks_to_show, cls.add_plot_landmark_markers)

        plt.axis('equal')
        plt.grid(color='k', linestyle=':', linewidth=1)
        plt.xlabel('y [m]')
        plt.ylabel('x [m]')
        plt.legend(loc='best')
        plt.show()

        return

    