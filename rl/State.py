from Constant import Constant
from Environment.Grid import Grid
from utils import manhattan_distance


class State:
    def __init__(self,distance_to_target_delivery,sensor_data,delivery_vehicle,target_delivery_location):
        self.__distance_to_target_delivery=distance_to_target_delivery
        self.__sensor_one_data = sensor_data[0]
        self.__sensor_two_data = sensor_data[1]
        self.__sensor_three_data = sensor_data[2]
        # self.sensor_four_data = sensor_data[3]
        # self.sensor_five_data = sensor_data[4]
        # self.sensor_six_data = sensor_data[5]
        # self.sensor_seven_data = sensor_data[6]
        # self.sensor_eight_data = sensor_data[7]
        self.__delivery_vehicle_velocity=delivery_vehicle.vel
        self.__delivery_vehicle_steering_angle=delivery_vehicle.angle
        self.__delivery_vehicle_location= (delivery_vehicle.x,delivery_vehicle.y)
        self.__target_delivery_location=target_delivery_location
        self.__number_of_pending_deliveries=None



    def scale_state_values(self, bottom_right_screen_positon,vehicle_max_speed):
        top_right_screen_position=(0,0)
        map_max_x_axis=bottom_right_screen_positon[0]
        map_max_y_axis=bottom_right_screen_positon[1]
        max_manhattan_distance=(manhattan_distance(top_right_screen_position[0],top_right_screen_position[1],map_max_x_axis,map_max_y_axis))
        scaled_distance_to_target_delivery= (self.__distance_to_target_delivery/max_manhattan_distance)
        scaled_sensor_one_data= self.__scale_sensor_data(self.__sensor_one_data)
        scaled_sensor_two_data=  self.__scale_sensor_data(self.__sensor_two_data)
        scaled_sensor_three_data=  self.__scale_sensor_data(self.__sensor_three_data)
        # scaled_sensor_four_data=  self.__scale_sensor_data(self.__sensor_four_data)
        # scaled_sensor_five_data=  self.__scale_sensor_data(self.__sensor_five_data)
        # scaled_sensor_six_data=  self.__scale_sensor_data(self.__sensor_six_data)
        # scaled_senor_seven_data=  self.__scale_sensor_data(self.__sensor_seven_data)
        # scaled_sensor_eight_data= ( self.__scale_sensor_data(self.__sensor_eight_data)
        scaled_delivery_vehicle_velocity= (self.__delivery_vehicle_velocity/vehicle_max_speed)
        scaled_delivery_vehicle_steering_angle= self.__delivery_vehicle_steering_angle/Constant.MAX_STEERING_ANGLE
        scaled_delivery_vehicle_location_x= (self.__delivery_vehicle_location[0]/map_max_x_axis)
        scaled_delivery_vehicle_location_y = (self.__delivery_vehicle_location[1]/map_max_y_axis)
        scaled_target_delivery_location_x= (self.__target_delivery_location.x/map_max_x_axis)
        scaled_target_delivery_location_y = (self.__target_delivery_location.y/ map_max_y_axis)
        # self.number_of_pending_deliveries =
        return (scaled_distance_to_target_delivery,scaled_sensor_one_data,scaled_sensor_two_data,scaled_sensor_three_data,
                scaled_delivery_vehicle_velocity,scaled_delivery_vehicle_steering_angle,scaled_delivery_vehicle_location_x
                ,scaled_delivery_vehicle_location_y,scaled_target_delivery_location_x,scaled_target_delivery_location_y)

    def __scale_sensor_data(self,sensor_data):
        has_sensed_collision=sensor_data[2]==1
        sensor_distance_to_obstacle=sensor_data[0]

        if not has_sensed_collision:
            sensor_distance_to_obstacle=sensor_distance_to_obstacle+14

        return sensor_distance_to_obstacle/(Constant.MAX_SENSOR_DISTANCE+14)



