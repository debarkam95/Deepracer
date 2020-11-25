import math

optimal_race_line = [[2.88802, 0.72484, 3.79, True],
        [3.15743, 0.72111, 3.79, True],
        [3.45451, 0.7362, 3.79, True],
        [3.76021, 0.74243, 3.79, True],
        [4.08648, 0.72121, 4.13, True],
        [4.39735, 0.7168, 4.13, False],
        [4.68298, 0.70203, 2.7, True],
        [5.05939, 0.67402, 2.3, True],
        [5.43799, 0.64738, 2.17, True],
        [5.7232, 0.64736, 2.17, True],
        [5.97036, 0.64913, 1.75, True],
        [6.19929, 0.68236, 1.75, True],
        [6.41197, 0.73638, 1.39, True],
        [6.60389, 0.81629, 1.39, True],
        [6.77389, 0.9393, 1.39, True],
        [6.91299, 1.08425, 1.39, True],
        [7.02216, 1.24231, 1.39, True],
        [7.11851, 1.4065, 1.39, True],
        [7.14742, 1.59814, 1.39, True],
        [7.15311, 1.80635, 1.39, True],
        [7.10237, 2.01496, 1.39, True],
        [7.01901, 2.22716, 1.75, True],
        [6.86214, 2.41215, 1.75, True],
        [6.68157, 2.56797, 2.17, True],
        [6.47136, 2.68346, 2.17, True],
        [6.2465, 2.76801, 2.38, True],
        [6.02019, 2.82072, 2.38, True],
        [5.80406, 2.86554, 3.79, True],
        [5.58726, 2.88282, 3.79, False],
        [5.38082, 2.92968, 3.79, False],
        [5.16996, 2.97752, 3.79, False],
        [4.96516, 3.03504, 3.79, False],
        [4.73702, 3.02603, 3.79, False],
        [4.54702, 3.15759, 3.79, False],
        [4.35453, 3.2673, 3.79, False],
        [4.1333, 3.38206, 3.79, False],
        [3.99082, 3.49667, 4.13, False],
        [3.83933, 3.66225, 4.13, False],
        [3.68255, 3.79537, 2.3, True],
        [3.55069, 3.93555, 2.3, True],
        [3.41306, 4.06962, 1.39, True],
        [3.25732, 4.1955, 1.39, True],
        [3.09947, 4.31364, 1.39, True],
        [2.92423, 4.40954, 2.17, True],
        [2.74625, 4.50911, 2.38, True],
        [2.54791, 4.5726, 2.17, True],
        [2.3361, 4.62419, 2.17, True],
        [2.10861, 4.64073, 1.75, True],
        [1.86681, 4.59607, 1.75, True],
        [1.60807, 4.53097, 1.75, True],
        [1.33605, 4.38327, 1.75, True],
        [1.07528, 4.14503, 1.39, True],
        [0.89974, 3.8079, 1.75, True],
        [0.82255, 3.45037, 2.38, True],
        [0.82591, 3.11959, 2.3, True],
        [0.85533, 2.82514, 2.3, True],
        [0.89679, 2.57176, 2.3, True],
        [0.93458, 2.32689, 2.3, True],
        [0.97915, 2.10062, 2.17, True],
        [1.03003, 1.89672, 1.75, True],
        [1.0867, 1.70766, 1.75, True],
        [1.17309, 1.52518, 1.75, True],
        [1.27132, 1.35355, 1.75, True],
        [1.39425, 1.20403, 1.75, True],
        [1.51097, 1.04681, 1.75, True],
        [1.69588, 0.9478, 2.29, True],
        [1.89322, 0.85644, 2.38, True],
        [2.11723, 0.78057, 2.38, True],
        [2.3466, 0.74001, 2.38, True],
        [2.61218, 0.72626, 3.79, True]]

class Utils:

    @staticmethod
    def eucleadian_distance(x1, y1, x2, y2):
        return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5
    
    # Find the two closest racing points index
    @staticmethod
    def closest_racing_points_index(x, y):
        distances = []
        for i in range(len(optimal_race_line)):
            distance = Utils.eucleadian_distance(optimal_race_line[i][0], optimal_race_line[i][1], x, y)
            distances.append(distance)
        closest_index = distances.index(min(distances))

        distances_no_closest = distances.copy()
        distances_no_closest[closest_index] = 999
        second_closest_index = distances_no_closest.index(min(distances_no_closest))

        return [closest_index, second_closest_index]
    
    # Distance between closest racing points and the vehicle
    @staticmethod
    def optimal_race_line_distance(closest_race_point, second_closest_race_point, x, y):           
        # Calculate the distances between 2 closest racing points
        a = abs(Utils.eucleadian_distance(closest_race_point[0], closest_race_point[1],
                            second_closest_race_point[0], second_closest_race_point[1]))
        # Distances between car and closest and second closest racing point
        b = abs(Utils.eucleadian_distance(x, y,
                            closest_race_point[0], closest_race_point[1]))
    
        c = abs(Utils.eucleadian_distance(x, y,
                            second_closest_race_point[0], second_closest_race_point[1]))
    
        # Calculate distance between car and racing line (goes through 2 closest racing points)
        # try-except in case a=0
        try:
            distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                         (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
        except:
            distance = b
    
        return distance

    # Calculate which one of the closest racing points is the next one and which one the previous one
    @staticmethod
    def next_prev_racing_point(closest_race_point, second_closest_race_point, x, y, heading):
    
        # Virtually set the car more into the heading direction
        x_new = x + math.cos(math.radians(heading))
        y_new = y + math.sin(math.radians(heading))
    
        # Calculate distance from new car coords to 2 closest racing points
        distance_closest_coords = Utils.eucleadian_distance(x_new, y_new, closest_race_point[0], closest_race_point[1])
        distance_second_closest_coords = Utils.eucleadian_distance(x_new, y_new, second_closest_race_point[0], second_closest_race_point[1])
    
        if distance_closest_coords <= distance_second_closest_coords:
            next_race_point = closest_race_point
            prev_race_point = second_closest_race_point
        else:
            next_race_point = second_closest_race_point
            prev_race_point = closest_race_point
    
        return [next_race_point, prev_race_point]

    # Calculate the heading and the direction of the track difference
    @staticmethod
    def direction_heading_difference(prev_race_point, next_race_point, heading):
        
        if heading>180:
            heading = heading-360
            
        if heading<-180:
            heading = heading+360

        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
        track_direction = math.degrees(math.atan2(
            next_race_point[1] - prev_race_point[1], next_race_point[0] - prev_race_point[0]))

        # Calculate the difference between the track direction and the heading direction of the car
        direction_diff = abs(track_direction - heading)
        if direction_diff > 180:
            direction_diff = 360 - direction_diff

        return direction_diff
    
    #calculate reward factor for complete laps
    @staticmethod
    def cal_factor(curr_steps,max_steps,worst_max_steps,max_reward_per_step,factor):
        prev=worst_max_steps*max_reward_per_step
        additive_factor=factor
        for steps in range(worst_max_steps-5,curr_steps-1,-1):
            total_max_reward_steps = steps*max_reward_per_step
            fractional_adjustment = ((max_steps)/steps)
            additive_factor = factor+(abs(prev-total_max_reward_steps)/fractional_adjustment)
            prev = total_max_reward_steps+fractional_adjustment*additive_factor
        return additive_factor
    
    #PIRATES_TRACE_LOG: 
    #arg1 = distance_from_optimal_race_line, 
    #arg2 = difference_direction_heading, 
    #arg3 = difference_speed,
    #arg4 = closest_race_point,
    #arg5 = is_left_of_center,
    @staticmethod
    def logValues(arg1, arg2, arg3):
        print("PIRATES_TRACE_LOG:%f,%d,%s" %(arg1, arg2, arg3))
    
class Reward:
    def __init__(self, verbose=False):
        self.verbose = verbose
        

    def reward_function(self, params):
        # Read input parameters
        track_width = params['track_width']
        distance_from_center = params['distance_from_center']
        is_reversed = params['is_reversed']
        is_offtrack = params['is_offtrack']
        x = params['x']
        y = params['y']
        speed = params['speed']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        steering_angle = params['steering_angle']
        is_left_of_center = params['is_left_of_center']

        MAX_SPEED_DIFF = 0.0
        MAX_DIRECTION_DIFF = 10.01
        MAX_NUM_STEPS = 120
        ERROR_MARGIN = 3.0
        DISTANCE_LIMIT = 0.131
        
        # Closest race points on the optimal race line from the vehicle
        closest_race_point_idx, second_closest_race_point_idx = Utils.closest_racing_points_index(x, y)
        closest_race_point = optimal_race_line[closest_race_point_idx]
        second_closest_race_point = optimal_race_line[second_closest_race_point_idx]
        next_race_point, prev_race_point = Utils.next_prev_racing_point(closest_race_point, second_closest_race_point, x, y, heading)

        # Distance from race line
        distance_from_optimal_race_line = Utils.optimal_race_line_distance(closest_race_point, second_closest_race_point, x, y)
        
       
        
        if self.verbose == True :
            Utils.logValues(distance_from_optimal_race_line,
                            closest_race_point_idx,is_left_of_center)
            
        # Reward for closeness to race line
        if distance_from_optimal_race_line < DISTANCE_LIMIT:
            reward = (1 - distance_from_optimal_race_line / DISTANCE_LIMIT) * 2.5
        else:
            reward = 1e-2
        
        if is_left_of_center == closest_race_point[3]:
            reward += 1.2

        reward = 1.5 * reward * (speed) + 1e-2

        if distance_from_center <= 0.9*track_width and is_left_of_center and steering_angle > 10:
            reward = 0.01*reward
        elif distance_from_center <= 0.9*track_width and is_left_of_center == False and steering_angle < -10:
            reward = 0.01*reward

        
        # Reward for every 15 steps
        if steps % 15 == 0:
            new_progress = ((steps / MAX_NUM_STEPS) * 100) - ERROR_MARGIN
            if progress > new_progress :
                reward += (progress - new_progress) * 9.0
            
        
        # Punishment for obvious bad behaviour
        if is_reversed or is_offtrack :
            reward = 1e-4
          
        # Reward for completing a lap relative to number of steps used
        if progress == 100:
            factor=Utils.cal_factor(int(steps),MAX_NUM_STEPS,175,9,15)
            reward += (MAX_NUM_STEPS / steps) * factor

        return float(reward)

reward_object = Reward(True)

def reward_function(params):
    return reward_object.reward_function(params)
