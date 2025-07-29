from tensorflow.python.keras.backend import dtype
import numpy as np
from Constant import Constant
from Environment.Grid import Grid
from utils import manhattan_distance


class State:
    def __init__(self,distance_to_target_delivery,sensor_data,delivery_vehicle,target_delivery_location,bottom_right_screen_position,current_location_speed_limit):
        self.__distance_to_target_delivery=distance_to_target_delivery
        self.__sensor_one_data = sensor_data[0]
        self.__sensor_two_data = sensor_data[1]
        self.__sensor_three_data = sensor_data[2]
        self.__sensor_four_data = sensor_data[3]
        self.__sensor_five_data = sensor_data[4]
        # self.__sensor_six_data = sensor_data[5]
        # self.__sensor_seven_data = sensor_data[6]
        # self.__sensor_eight_data = sensor_data[7]
        self.__delivery_vehicle_velocity=delivery_vehicle.vel
        self.__delivery_vehicle_steering_angle=delivery_vehicle.angle
        self.__delivery_vehicle_location= (delivery_vehicle.x,delivery_vehicle.y)
        self.__target_delivery_location=target_delivery_location
        self.__number_of_pending_deliveries=None
        self.__bottom_right_screen_position=bottom_right_screen_position
        self.__delivery_vehicle_max_vel=delivery_vehicle.max_vel
        self.__current_location_speed_limit=current_location_speed_limit



    def scale_state_values(self):
        top_right_screen_position=(0,0)
        map_max_x_axis=self.__bottom_right_screen_position[0]
        map_max_y_axis=self.__bottom_right_screen_position[1]
        # max_manhattan_distance=(manhattan_distance(top_right_screen_position[0],top_right_screen_position[1],map_max_x_axis,map_max_y_axis))
        max_manhattan_distance=(manhattan_distance(top_right_screen_position[0],top_right_screen_position[1],map_max_x_axis,map_max_y_axis))
        distance_to_target_delivery=manhattan_distance(self.__delivery_vehicle_location[0],self.__delivery_vehicle_location[1],self.__target_delivery_location.x,self.__target_delivery_location.y)
        scaled_distance_to_target_delivery= self.transform_result_between_negative_one_and_positive_one(distance_to_target_delivery/max_manhattan_distance)
        scaled_sensor_one_data= self.transform_result_between_negative_one_and_positive_one(self.__scale_sensor_data(self.__sensor_one_data))
        scaled_sensor_two_data=  self.transform_result_between_negative_one_and_positive_one(self.__scale_sensor_data(self.__sensor_two_data))
        scaled_sensor_three_data=  self.transform_result_between_negative_one_and_positive_one(self.__scale_sensor_data(self.__sensor_three_data))
        scaled_sensor_four_data=  self.transform_result_between_negative_one_and_positive_one(self.__scale_sensor_data(self.__sensor_four_data))
        scaled_sensor_five_data=  self.transform_result_between_negative_one_and_positive_one(self.__scale_sensor_data(self.__sensor_five_data))
        # scaled_sensor_six_data=  self.transform_result_between_negative_one_and_positive_one(self.__scale_sensor_data(self.__sensor_six_data))
        # scaled_senor_seven_data=  self.transform_result_between_negative_one_and_positive_one(self.__scale_sensor_data(self.__sensor_seven_data))
        # scaled_sensor_eight_data= self.transform_result_between_negative_one_and_positive_one( self.__scale_sensor_data(self.__sensor_eight_data))
        scaled_delivery_vehicle_velocity= self.transform_result_between_negative_one_and_positive_one((self.__delivery_vehicle_velocity/self.__delivery_vehicle_max_vel))
        scaled_delivery_vehicle_steering_angle= self.transform_result_between_negative_one_and_positive_one(self.__delivery_vehicle_steering_angle/Constant.MAX_STEERING_ANGLE)
        scaled_delivery_vehicle_location_x= self.transform_result_between_negative_one_and_positive_one(self.__delivery_vehicle_location[0]/map_max_x_axis)
        scaled_delivery_vehicle_location_y = self.transform_result_between_negative_one_and_positive_one(self.__delivery_vehicle_location[1]/map_max_y_axis)
        scaled_target_delivery_location_x= self.transform_result_between_negative_one_and_positive_one(self.__target_delivery_location.x/map_max_x_axis)
        scaled_target_delivery_location_y = self.transform_result_between_negative_one_and_positive_one(self.__target_delivery_location.y/ map_max_y_axis)
        scaled_current_location_speed_limit= self.transform_result_between_negative_one_and_positive_one((self.__current_location_speed_limit/Constant.MAX_VEL))
        # self.number_of_pending_deliveries =
        return np.array([scaled_distance_to_target_delivery,scaled_sensor_one_data,scaled_sensor_two_data,scaled_sensor_three_data,
                         scaled_sensor_four_data,scaled_sensor_five_data,
                         # scaled_sensor_six_data,scaled_senor_seven_data,scaled_sensor_eight_data,
                scaled_delivery_vehicle_velocity,scaled_delivery_vehicle_steering_angle,scaled_delivery_vehicle_location_x
                ,scaled_delivery_vehicle_location_y,scaled_target_delivery_location_x,scaled_target_delivery_location_y,scaled_current_location_speed_limit],dtype=np.float32)


    def transform_result_between_negative_one_and_positive_one(self, value):
        return 2*value - 1

    def __scale_sensor_data(self,sensor_data):
        has_sensed_collision=sensor_data[2]==1
        sensor_distance_to_obstacle=sensor_data[0]

        if not has_sensed_collision:
            sensor_distance_to_obstacle=sensor_distance_to_obstacle+14

        return sensor_distance_to_obstacle/(Constant.MAX_SENSOR_DISTANCE+14)

    def get_distance_to_target_delivery(self):
        return self.__distance_to_target_delivery

    def get_sensor_one_data(self):
        return self.__sensor_one_data

    def get_sensor_two_data(self):
        return self.__sensor_two_data

    def get_sensor_three_data(self):
        return self.__sensor_three_data
    def get_sensor_four_data(self):
        return self.__sensor_four_data
    def get_sensor_five_data(self):
        return self.__sensor_five_data

    def get_delivery_vehicle_velocity(self):
        return self.__delivery_vehicle_velocity

    def get_delivery_vehicle_steering_angle(self):
        return self.__delivery_vehicle_steering_angle

    def get_delivery_vehicle_location(self):
        return self.__delivery_vehicle_location

    def get_target_delivery_location(self):
        return self.__target_delivery_location

    def get_number_of_pending_deliveries(self):
        return self.__number_of_pending_deliveries

    def get_bottom_right_screen_position(self):
        return self.__bottom_right_screen_position

    def get_delivery_vehicle_max_vel(self):
        return self.__delivery_vehicle_max_vel


