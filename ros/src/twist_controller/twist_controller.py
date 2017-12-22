import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

#costum parameters for pid controller
PID_P_VEL=0.5
PID_I_VEL=0.001
PID_D_VEL=0.1

PID_P_ACC=0.5
PID_I_ACC=0.05
PID_D_ACC=0.1

MIN_SPEED=.5
LPF_ACC_TAU=0.2

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass=kwargs['vehicle_mass']
        self.fuel_capacity=kwargs['fuel_capacity']
        self.brake_deadband=kwargs['brake_deadband']
        self.decel_limit=kwargs['decel_limit']
        self.accel_limit=kwargs['accel_limit']
        self.wheel_radius=kwargs['wheel_radius']
        self.wheel_base=kwargs['wheel_base']
        self.steer_ratio=kwargs['steer_ratio']
        self.max_lat_accel=kwargs['max_lat_accel']
        self.max_steer_angle=kwargs['max_steer_angle']

        self.brake_torque_const=(self.vehicle_mass+self.fuel_capacity*GAS_DENSITY)*self.wheel_radius

        self.twist=None
        self.current_velocity=0.0
        self.past_velocity=0.0
        self.current_acc=0.0

        self.lowpass_filter_acc=LowPassFilter(LPF_ACC_TAU,1.0/50)
        self.pid_vel=PID(PID_P_VEL,PID_I_VEL,PID_D_VEL,self.decel_limit,self.accel_limit)
        self.pid_acc=PID(PID_P_ACC,PID_I_ACC,PID_D_ACC,0.0,0.75)

        self.yaw_controller=YawController(self.wheel_base,
                                          self.steer_ratio,
                                          MIN_SPEED,
                                          self.max_lat_accel,
                                          self.max_steer_angle)
        #rospy.logerr('twist_controller.self() done')
        pass

    def control(self):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 0., 0., 0.

    def control(self,required_vel_linear,
                required_vel_angular,
                current_velocity):
        throttle,brake,steering=0.0,0.0,0.0
        velocity_error=required_vel_linear-current_velocity
        #get current acc using lowpass filter
        #rospy.logerr('twist_controller: velocity_error=%d',velocity_error)
        acc_temp=(self.past_velocity-current_velocity)*50.0
        self.past_velocity = current_velocity
        self.lowpass_filter_acc.filt(acc_temp)
        self.current_acc=self.lowpass_filter_acc.get() #at first, current_acc=0
        #rospy.logerr('twist_controller: current_acc=%d',self.current_acc)

        required_acc=self.pid_vel.step(velocity_error,1.0/50)
        #rospy.logerr('twist_controller: required_acc=%d',required_acc)

        if required_acc<0:
            throttle=0
            self.pid_acc.reset()
            if -required_acc>self.brake_deadband:
                #brake, but not over the brak deadband
                brake=-self.decel_limit*self.brake_torque_const
            else:
                brake=-required_acc*self.brake_torque_const
        else: 
            if required_acc>self.accel_limit:
                throttle=self.pid_acc.step(self.accel_limit-self.current_acc,1.0/50)
            else:
                throttle=self.pid_acc.step(required_acc-self.current_acc,1.0/50)
        steering=self.yaw_controller.get_steering(required_vel_linear,required_vel_angular,current_velocity)

        return throttle, brake, steering

    def reset(self):
        self.pid_vel.reset()
        pass
