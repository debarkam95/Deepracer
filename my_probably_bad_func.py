import math

def track_center(params):
    '''
    Example of rewarding the agent to follow center line
    '''
    
    # Read input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']

    # Calculate 3 markers that are increasingly further away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    # Give higher reward if the car is closer to center line and vice versa
    if distance_from_center <= marker_1:
        reward = 0
    elif distance_from_center <= marker_2:
        reward = 20
    elif distance_from_center <= marker_3:
        reward = 40
    else:
        reward = 100  # likely crashed/ close to off track

    return reward

def reward_function(params):
   
    # Read input parameters
    all_wheels_on_track = params['all_wheels_on_track']
    track_width = params['track_width']
    is_reversed = params['is_reversed']
    is_offtrack = params['is_offtrack']
    x = params['x']
    y = params['y']
    speed = params['speed']
    heading = params['heading']
    progress = params['progress']
    steps = params['steps']
    is_crashed = params['is_crashed']
    steering = params['steering_angle']

    reward = 5000

    if x > 3.0 and x < 5.0 and y < 1.2:
        reward -= math.fabs(3.5-speed)*30
        reward -= math.fabs(steering)*10
    elif x > 5.0 and x < 6.2 and y < 1.2:
        reward -= math.fabs(2.33-speed)*30
        reward -= math.fabs(steering)*10
    elif x > 6.2 and x < 8:
        reward -= track_center(params)
    



    # Punishment for obvious bad behaviour
    if is_crashed or is_reversed or all_wheels_on_track == False or difference_speed > MAX_SPEED_DIFF or difference_direction_heading > MAX_DIRECTION_DIFF:
        reward *= 0.0
      

    # Reward for completing a lap relative to number of steps used
    if progress == 100:
        factor=100
        if steps in range(0,130):
            factor+=200
        if steps in range(130,135):
            factor+=180
        if steps in range(135,140):
            factor+=140
        if steps in range(140,145):
            factor+=120
        if steps in range(145,150):
            factor+=80
        if steps in range(150,155):
            factor+=60
        reward += (MAX_NUM_STEPS / steps) * factor
    
        
    return float(reward)