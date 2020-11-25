import math

# Optimal race line coordinates calculated as per K1999 Path-Optimization Algorithm
# [x,y,optimal_speed]
optimal_race_line = [[3.00925, 0.68729, 4.0],
          [3.27277, 0.68332, 4.0],
          [3.42, 0.68337, 4.0],
          [3.63, 0.68345, 3.64591],
          [4.06891, 0.68361, 2.93049],
          [4.5, 0.68376, 2.51348],
          [4.55, 0.68378, 2.23448],
          [5.32, 0.68405, 2.02348],
          [5.49637, 0.69133, 1.83243],
          [5.73858, 0.71104, 1.66047],
          [5.9827, 0.74877, 1.5177],
          [6.20141, 0.80417, 1.44564],
          [6.39242, 0.87539, 1.44564],
          [6.5584, 0.96133, 1.44564],
          [6.70108, 1.06191, 1.3763],
          [6.82048, 1.17784, 1.3763],
          [6.91475, 1.31009, 1.33],
          [6.98133, 1.45824, 1.33],
          [7.02088, 1.61872, 1.33],
          [7.02811, 1.78954, 1.33],
          [6.99367, 1.96614, 1.33],
          [6.91438, 2.13988, 1.33],
          [6.7795, 2.2958, 1.71444],
          [6.61115, 2.43077, 1.91052],
          [6.41794, 2.54231, 2.19036],
          [6.20881, 2.63188, 2.57654],
          [5.99059, 2.70359, 3.09151],
          [5.76774, 2.76279, 2.5745],
          [5.5618, 2.81145, 2.5745],
          [5.35824, 2.86653, 2.5745],
          [5.15731, 2.92929, 2.5745],
          [4.95925, 3.0014, 2.5745],
          [4.76427, 3.08503, 2.5745],
          [4.57314, 3.18623, 2.93738],
          [4.38485, 3.30195, 3.40686],
          [4.19881, 3.42916, 3.1316],
          [4.01438, 3.56442, 2.85369],
          [3.83083, 3.7039, 2.54834],
          [3.67875, 3.81381, 2.53554],
          [3.52536, 3.91784, 2.46201],
          [3.3701, 4.0147, 2.38466],
          [3.21226, 4.10316, 2.2927],
          [3.05096, 4.18203, 2.18404],
          [2.88466, 4.2494, 2.04138],
          [2.71244, 4.3053, 1.90632],
          [2.53284, 4.34878, 1.77236],
          [2.34382, 4.37809, 1.63874],
          [2.14254, 4.39003, 1.50917],
          [1.92494, 4.37869, 1.50917],
          [1.6859, 4.33215, 1.50917],
          [1.42639, 4.23019, 1.50917],
          [1.17176, 4.05124, 1.50917],
          [0.96782, 3.78413, 1.50917],
          [0.87434, 3.43831, 2.07825],
          [0.85432, 3.09722, 2.43489],
          [0.87613, 2.81011, 2.42504],
          [0.9118, 2.57516, 2.17706],
          [0.96295, 2.30914, 1.98215],
          [1.00954, 2.10263, 1.7829],
          [1.06513, 1.90155, 1.61694],
          [1.1333, 1.70741, 1.61694],
          [1.21512, 1.52647, 1.61694],
          [1.31036, 1.36324, 1.61694],
          [1.41927, 1.21917, 1.61694],
          [1.5448, 1.09469, 1.61694],
          [1.69641, 0.9896, 1.91518],
          [1.87696, 0.89732, 2.13782],
          [2.09543, 0.81711, 2.39254],
          [2.36574, 0.75128, 2.75578],
          [2.69273, 0.70604, 3.233]]

def eucleadian_distance(x1, y1, x2, y2):
  return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

# Find the two closest racing points index
def closest_racing_points_index(x, y):
  distances = []
  for i in range(len(optimal_race_line)):
      distance = eucleadian_distance(optimal_race_line[i][0], optimal_race_line[i][1], x, y)
      distances.append(distance)

  closest_index = distances.index(min(distances))

  distances_no_closest = distances.copy()
  distances_no_closest[closest_index] = 999
  second_closest_index = distances_no_closest.index(
      min(distances_no_closest))

  return [closest_index, second_closest_index]

# Distance between closest racing points and the vehicle
def optimal_race_line_distance(closest_race_point, second_closest_race_point, x, y):           
  # Calculate the distances between 2 closest racing points
  a = abs(eucleadian_distance(closest_race_point[0], closest_race_point[1],
                        second_closest_race_point[0], second_closest_race_point[1]))

  # Distances between car and closest and second closest racing point
  b = abs(eucleadian_distance(x, y,
                        closest_race_point[0], closest_race_point[1]))

  c = abs(eucleadian_distance(x, y,
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
def next_prev_racing_point(closest_race_point, second_closest_race_point, x, y, heading):

  # Virtually set the car more into the heading direction
  x_new = x + math.cos(math.radians(heading))
  y_new = y + math.sin(math.radians(heading))

  # Calculate distance from new car coords to 2 closest racing points
  distance_closest_coords = eucleadian_distance(x_new, y_new, closest_race_point[0], closest_race_point[1])
  distance_second_closest_coords = eucleadian_distance(x_new, y_new, second_closest_race_point[0], second_closest_race_point[1])

  if distance_closest_coords <= distance_second_closest_coords:
      next_race_point = closest_race_point
      prev_race_point = second_closest_race_point
  else:
      next_race_point = second_closest_race_point
      prev_race_point = closest_race_point

  return [next_race_point, prev_race_point]

# Calculate the heading and the direction of the track difference
def direction_heading_difference(prev_race_point, next_race_point, heading):

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.degrees(math.atan2(
        next_race_point[1] - prev_race_point[1], next_race_point[0] - prev_race_point[0]))

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    return direction_diff

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

    MAX_SPEED_DIFF = 0.7
    MAX_DIRECTION_DIFF = 30.0
    MAX_NUM_STEPS = 145
    ERROR_MARGIN = 3.0
    
    
    # Closest race points on the optimal race line from the vehicle
    closest_race_point_idx, second_closest_race_point_idx = closest_racing_points_index(x, y)
    closest_race_point = optimal_race_line[closest_race_point_idx]
    second_closest_race_point = optimal_race_line[second_closest_race_point_idx]
    next_race_point, prev_race_point = next_prev_racing_point(closest_race_point, second_closest_race_point, x, y, heading)

    # Distance from race line
    distance_from_optimal_race_line = optimal_race_line_distance(closest_race_point, second_closest_race_point, x, y)
    
    # Difference in direction heading
    difference_direction_heading = direction_heading_difference(prev_race_point, next_race_point, heading)

    # Difference in speed and optimal speed
    difference_speed = abs(closest_race_point[2] - speed)
    
    print('closest_race_points: (%f, %f), (%f, %f), car_cords: (%f, %f), distance: %f, speed: %f, optimal_speed: %f,  speed_diff: %f, heading: %f, direction_heading_diff: %f' % 
      (prev_race_point[0], prev_race_point[1], next_race_point[0], next_race_point[1], x, y, distance_from_optimal_race_line,
        speed, closest_race_point[2], difference_speed, heading, difference_direction_heading))
    
    # Reward for closeness to race line
    if distance_from_optimal_race_line == 0:
        reward = 10.0
    elif distance_from_optimal_race_line <= 0.1*track_width:
        reward = 8.0
    elif distance_from_optimal_race_line <= 0.25*track_width:
        reward = 4.0
    elif distance_from_optimal_race_line <= 0.5*track_width:
        reward = 2.0
    else:
        reward = 0.1

    # Reward for closeness to optimal speed
    if difference_speed <= MAX_SPEED_DIFF:
        reward += (1 - difference_speed / MAX_SPEED_DIFF) * 5.0
    
    # Reward for closeness to direction of race line
    if difference_direction_heading <= MAX_DIRECTION_DIFF:
      reward += (1 - difference_direction_heading / MAX_DIRECTION_DIFF) * 5.0

    # Reward for every 50 steps
    new_progress = ((steps / MAX_NUM_STEPS) * 100) - ERROR_MARGIN
    if steps % 50 == 0 and progress > new_progress :
        reward += (progress - new_progress) * 20.0

      
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