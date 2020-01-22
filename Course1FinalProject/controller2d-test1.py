#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('prev_err', 0.0)
        self.vars.create_var('prev_tm', 0.0)
        self.vars.create_var('prevc_i', 0.0)
        self.vars.create_var('integral_error_lateral', 0.0)

        u = 0
        k_p = 2.5
        k_i = 0.05
        k_d = 0.01
        kappa = 2.5
        k_s = 0.1
        c_p = 0
        c_i = 0
        c_d = 0
        prev_e = 0
        prev_t = 0
        prevc_i = 0
        alfa_1 = 0.5
        kp_1 = 0.6
        ki_1 =  0.4
        error_lateral = 0
        i_errlat = 0.0
 

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            dt = t - self.vars.prev_tm
            error = v_desired - v
            de = error - self.vars.prev_err

            c_p = k_p * error
            prevc_i = self.vars.prevc_i + error * dt

            c_d = 0

            if dt > 0:
                c_d = de/dt

            prev_t = t
            prev_e = error

            u = c_p + (k_i * prevc_i) + ( k_d * c_d)

            #v = self.vars.v_previous + (u * dt) 

            throttle_p = 0
            brake_p = 0

            if u >= 0:
                throttle_p = u
                brake_p = 0

            if u < 0:
                throttle_p = 0
                brake_p = -u              
                     
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            throttle_output = 0
            brake_output    = 0

            throttle_output = throttle_p
            brake_output = brake_p

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            #min_idx1       = 0
            #min_dist1      = float("inf")
            #for i in range(len(self._waypoints)):
            #    dist1 = np.linalg.norm(np.array([
            #             self._waypoints[i][0] - self._current_x,
            #             self._waypoints[i][1] - self._current_y]))
            #    if dist1 < min_dist1:
            #        min_dist1 = dist1
            #        min_idx1 = i
            #if min_idx1 < len(self._waypoints)-1:
            #    x_d = self._waypoints[min_idx1][0]
            #    y_d = self._waypoints[min_idx1][1]
            #else:
            #    x_d = self._waypoints[-1][0]
            #    y_d = self._waypoints[-1][1]

            #x_d = waypoints[5][0]
            #y_d = waypoints[5][1]

            #xc = x + (1.5 * np.sin(yaw))
            #yc = y + (1.5 * np.cos(yaw))

            #a = y - y_d
            #b = x_d - x
            #c = (x * y_d) - (x_d * y)

            #e = ((a*xc)+(b*yc)+c)/np.sqrt((x*x) + (y*y))
            #e = ((a*x)+(b*y)+c)/np.sqrt((a*a) + (b*b))

            #e = ((a*xc)+(b*yc)+c)/np.sqrt((a*a) + (b*b))

            #head_error = np.arctan(-a/b) - yaw
            #head_error = np.arctan2(-a,b) - yaw

            #if head_error > 180:
            #    head_error = head_error - 360

            #delta = head_error + np.arctan((kappa*e)/(k_s+v))
            #delta = head_error + np.arctan((kappa*e)/(k_s+u))
            #delta = head_error + np.arctan2((kappa*e),(k_s+v))
            #delta = head_error + np.arctan2((kappa*e),k_s+v)


            x_d = waypoints[-10][0]
            y_d = waypoints[-10][1]

            error_lateral = -(x_d - x) * np.sin(yaw) + (y_d - y)*np.cos(yaw)
            i_errlat = self.vars.integral_error_lateral + error_lateral * dt
            acceleration_l = kp_1*error_lateral + ki_1*i_errlat
            delta = 1.22 * np.tanh(alfa_1*acceleration_l)

            # Change the steer output with the lateral controller. 
            steer_output    = delta

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.prev_err = prev_e
        self.vars.prev_tm  = prev_t
        self.vars.prevc_i  = prevc_i
        self.vars.integral_error_lateral = i_errlat


