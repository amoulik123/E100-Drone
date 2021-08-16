# -*- coding: utf-8 -*-

"""

Created on Sat Mar  6 23:35:38 2021



@author: juhyeon

"""



# -*- coding: utf-8 -*-

"""

Created on Thursday Mar  25 



@author: juhyeon

ENG100-400 Drone Staff Copy [Annotated]

"""



#############################################################################



##############################################################################

############      Initial set up & Functions      ############################

############   Do not make any modification in this section. #################

##############################################################################

# Set Imports

import setup_path 

#import airsim

import time

import numpy as np

import airsimdroneracinglab as asdl

import math

import E100_functions



dt = E100_functions.dt()   

client = asdl.MultirotorClient()

E100_functions.takeoff(client)



##############################################################################

##############################################################################

##### Everything below this line is okay to change / replace  ############

##############################################################################

##############################################################################







# Initialize altitude PID values

K_P = 0.3

K_I = 0.2

K_D = 0.1

integration_term = 0

##############################################


# Initialize pitch PID values

pitch_K_P = 0.5

pitch_K_I = 0

pitch_K_D = 0.6

pitch_integration_term = 0

##############################################


# Initialize roll PID values

roll_K_P = 0.5

roll_K_I = 0

roll_K_D = 0.6

roll_integration_term = 0

##############################################



# Initialize back PID values

back_K_P = 0.5

back_K_I = 0

back_K_D = 0.6

back_integration_term = 0

##############################################



# Initialize target values 

target_alt = 10        # Used in PID implementation (Altitude)

target_front_dist = 8  # Used in PID implementation  (Front pitch)

target_right_dist = 5  # Used in PID implementation  (Right roll)

target_back_dist = 8  # Used in PID implementation  (Back pitch)

target_left_dist = 5  # Used in PID implementation  (Left roll)



desired_pitch = 0 

desired_roll = 0 



throttle = 0.5  

##############################################





# Initialize flags

alt_flag = 1

pitch_flag = 1

roll_flag = 1



altitude_sensor_flag = 1

Lidar_sensor_flag = 1



# Add additional flags  - For conditional

back_flag = 1

negpitch_flag = 0

pospitch_flag = 1

##############################################





desired_yaw = 0 # Set desired Yaw to 0. 



start = time.time() # Get time at the start of the program







# While loop that will continuously loop throughout the duration of your code

while True:

    

    # If throttle value does not equal a number, exit the loop - terminating all control.

    if np.isnan(throttle) == 1:

        break;

    



    # Some implementations will land the quadcopter after a pre-determined period of time. 

    # This is illistrated by using an IF statement 

    

    #now = time.time()

    #if now - start > 300:   # You should change the condition to land.

     #   E100_functions.land(client)

      #  break

    

    

    # Control signal 

    E100_functions.set_quadcopter(client,desired_roll,desired_pitch,desired_yaw,throttle)

   

    

   

    ##############################################################

    ###############   Get sensor readings   #####################

    ##############################################################

    

    # Get Altitude, Pitch, Roll, and Yaw readings - get two values and get average of the two to smooth out any anomalous values

    

    # Flag will always be set first time around

    if altitude_sensor_flag == 1:

        altitude_n1 = E100_functions.get_altitude(client) # Get altitude

    else:

        altitude_n1 = altitude_n0 # If flag is set to 0, use previously stored altitude value

        

        

        

    if Lidar_sensor_flag == 1:

        roll_n1, pitch_n1, yaw_n1 = E100_functions.get_orientation(client) # Get pitch, roll, and yaw information 

    else:

        roll_n1 = roll_n0 # If flag is set to 0, use previously stored values for pitch, roll, and yaw.

        pitch_n1 = pitch_n0  

        yaw_n1 = yaw_n0

    

    # Irregardless of altitude flags, get the altitude of the quadcopter and store it into altitude_n0 variable.            

    altitude_n0 = E100_functions.get_altitude(client) 

    

    # Use average altitude as final altitude

    altitude = (altitude_n0 + altitude_n1)/2

    

    roll_n0, pitch_n0, yaw_n0 = E100_functions.get_orientation(client) # Get pitch, roll, and yaw information irregardless of flags

    

    # Use average values as final values

    roll = (roll_n0+roll_n1)/2

    pitch = (pitch_n0+pitch_n1)/2

    yaw = (yaw_n0+yaw_n1)/2

    

    

    # Get LIDAR readings

    front, right, left, back = E100_functions.get_lidars(client)

    # Get 5th LIDAR readings 
    
    points5 = E100_functions.get_lidarData_points(client.getLidarData(lidar_name= "LidarSensor5"))
    up = np.linalg.norm(np.mean(points5, axis=0))
    print('up = ', up)
    
    # Get 6th LIDAR readings 
    
    points6 = E100_functions.get_lidarData_points(client.getLidarData(lidar_name= "LidarSensor6"))
    down = np.linalg.norm(np.mean(points6, axis=0))
    print('down = ', down)
    
    # Get GPS readings 
    gps_data = client.getGpsData()

    # Output sensor readings to the terminal

    print('Roll = ',roll,'Pitch = ',pitch,'Yaw =', yaw) #(unit: degrees)

    print('F = ',front,'R = ',right,'L = ',left,'B = ',back)

    print('Alt = ',altitude)
    
    print('GPS Latitude = ', gps_data.gnss.geo_point.latitude, "GPS Longitude", gps_data.gnss.geo_point.longitude)
    
    

    ######################################################################

    ######################################################################

    ######################################################################

    ######################################################################

    

    # STUDENTS: You do not need to change the altitude, pitch, or roll PID implementations (3 blocks of code below). 

    # However, if you believe that your implementation would be better than what is provided, please feel free to change the code. 

    

    ##############################################################

    ###############   Altitude hold (PID controller)   #####################

    ##############################################################

    if alt_flag == 1:

        error_old = target_alt-altitude # Get error value between target and actual value

        alt_flag = 0

    else:

        error_old = error    # Set old error equal to error

        

    # Set values for error, integration and differential terms 

    error = target_alt-altitude

    integration_term += error*dt # Integral term accrues error overtime. Add the error

    differential_term = (error - error_old)/dt   # Derivative term gets the difference of the errors over time

    

    # If the Contrl signal is negative, set throttle to zero

    if K_P*error + K_I*integration_term + K_D*differential_term < 0:

        throttle = 0

    else:

        throttle = np.sqrt(K_P*error + K_I*integration_term + K_D*differential_term) # Convert PID transfer function to a throttle by taking the square root (square root due to property of fluids)

    

    if throttle >= 1:

        throttle = 1 # Cap throttle value at 1

    print('Throttle = ',throttle)

    ##############################################################

    ##############################################################

       

    

    

    

    ##############################################################

    ###############   Set Front Pitch (PID controller)   #####################

    ##############################################################    

    if pospitch_flag == 1:

        

        if pitch_flag == 1:

            pitch_error_old = target_front_dist-front # Get error value between target and actual value

            pitch_flag = 0

        else:

            pitch_error_old = pitch_error    # Set old error equal to error

    



       # Set values for error, integration, and differential terms

        pitch_error = target_front_dist-front

        pitch_integration_term += pitch_error*dt

        pitch_differential_term = (pitch_error - pitch_error_old)/dt

    

    # Sum kp, ki, and kd terms to generate a control signal

        desired_pitch = pitch_K_P*pitch_error + pitch_K_I*pitch_integration_term + pitch_K_D*pitch_differential_term

    

    # Cap control signal to the bounds of the hyperbolic tangent function. Very elegant. 

        desired_pitch = math.degrees(round(0.15*np.tanh(desired_pitch),8)) 

    ##############################################################

    ##############################################################

        

    

    

    

    ##############################################################

    ###############   Set Right Roll (PID controller)   #####################

    ##############################################################    

    if roll_flag == 1:

        roll_error_old = right - target_right_dist # Get error value between target and actual value

        roll_flag = 0

    else:

        roll_error_old = roll_error     # Set old error equal to error

        

    # Set values for error, integration, and differential terms    

    roll_error=  right - target_right_dist

    roll_integration_term += roll_error*dt

    roll_differential_term = (roll_error - roll_error_old)/dt 

    

     # Sum kp, ki, and kd terms to generate a control signal

    desired_roll = roll_K_P*roll_error + roll_K_I*roll_integration_term + roll_K_D*roll_differential_term

    # Cap control signal to the bounds of the hyperbolic tangent function. Very elegant.

    desired_roll = math.degrees(0.2*np.tanh(0.5*desired_roll))

    ##############################################################

    ##############################################################

    ##############################################################

    ##############################################################

    

    

    desired_yaw = 0



    if front <= target_front_dist and right <= target_right_dist:

        pitch_flag = 0

        pospitch_flag = 0

        negpitch_flag = 1

        desired_roll = 0



    if back <= target_back_dist and right <= target_right_dist:

        pitch_flag = 1

        pospitch_flag = 1

        negpitch_flag = 0

        desired_roll = 0



    ##############################################################

    ###############   Set Back Pitch (PID controller)   #####################

    ##############################################################    

    if negpitch_flag == 1:

    

        if back_flag == 1:

            pitch_error_old = target_back_dist-back # Get error value between target and actual value

            back_flag = 0

        else:

            pitch_error_old = pitch_error    # Set old error equal to error

    



    # Set values for error, integration, and differential terms

        pitch_error= target_back_dist-back

        back_integration_term += pitch_error*dt

        back_differential_term = (pitch_error - pitch_error_old)/dt

    

    # Sum kp, ki, and kd terms to generate a control signal

        desired_pitch = back_K_P*pitch_error + back_K_I*back_integration_term + back_K_D*back_differential_term

    

    # Cap control signal to the bounds of the hyperbolic tangent function. Very elegant. 

        desired_pitch = -math.degrees(round(0.15*np.tanh(desired_pitch),8)) 


    # add timer instead to land 
    if front <= target_front_dist and right <= target_right_dist and left <= target_left_dist:

        E100_functions.land(client)

        break



###################______ADD CODE BELOW ____###########################

# Code below should be your control scheme. Control schemes can incorporate LIDAR Values,

# additional variables (flags), timers, additional PID controllers, etc. When finished, be sure to land your drone. 



# With the current pitch front PID and roll right PID the quadcopter will fly 

# forwards until the end of the hallway - satisfying the forward target distance requirement 

# The right roll PID will then take over and it will roll to the right until the end of the second hallway - 

# satisfying the right target distance requirement. 



# In other words, without any code provided below, the drone will fly in an 'L' shape and then simply hover there.  

























