#imports from Q+A video imports
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
 
    def __init__(self, *args, **kwargs):
        # TODO: Implement
         #all below from Q+A video
            #video never mentioend the4 kwags eror below
        #self.yaw_controller = YawController(wheel_base,steer_ratio, 0.1, max_lat_accel,max_steer_angle)
        self.yaw_controller = YawController(wheel_base = kwargs['wheel_base'],
                                            steer_ratio = kwargs['steer_ratio'],
                                            min_speed =  kwargs['min_speed'],
                                            max_lat_accel = kwargs['max_lat_accel'],
                                            max_steer_angle = kwargs['max_steer_angle'])
     
   ##error
#File "/home/workspace/CarND-Capstone/ros/src/twist_controller/twist_controller.py", line 10, in __init__
#    self.yaw_controller = YawController(wheel_base,steer_ratio, 0.1, max_lat_accel,max_steer_angle)
#NameError: global name 'YawController' is not defined
    
    
        kp = 0.3
        ki = 0.1 # had k1 here mistake
        kd = 0. # ?? this is how it is in Q+A
        mn = 0. # Minimum throttle value ### ?? this is how it is in Q+A
        mx = 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, mn, mx)
        
        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = 0.02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.vehicle_mass = kwargs['vehicle_mass'] #maybe not best way I could take all the kwargs together at start of thsi section might be easier to read
        self.fuel_capacity = kwargs['fuel_capacity'] #same as above all kwargs added here
        self.brake_deadband = kwargs['brake_deadband'] 
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        
        self.last_time = rospy.get_time()
        #all above from Q+A video
       # pass ## old line

    #def control(self, *args, **kwargs):# old code
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel): #from Q+A video
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
        #return 1., 0., 0.#old code
        return 0., 0., 0.
    
        current_vel = self.vel_lpf.filt(current_vel)
        
        #rospy.logwarn("Angular vel: {0}", format(angular_vel))
        #rospy.logwarn("Target velocity: {0}".format (linear_vel))
        #rospy.logwarn("Target angular velocity: {0} \n" .format(angluar_vel)
        #rospy.logwarn("Current velocity : {0} " .format(current_vel))
        # rospy.logwarn("Filtered velocity: {0} " .format(self.vel_lpf.get()))
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)#Q+A video suggest steering damping here
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0
        
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400 # N*m - to hold the car in place if we were stopped at a light. Acceleration - 1m/s^2
            
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m
            
           
        return throttle, brake, steering