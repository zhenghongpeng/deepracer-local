import math


class Reward(object):
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[0.76561, 2.79921, 1.96678, 0.05285],
[0.77433, 2.69551, 2.04772, 0.05082],
[0.78999, 2.59242, 2.02344, 0.05154],
[0.81253, 2.49023, 1.86754, 0.05603],
[0.84156, 2.3892, 1.70912, 0.0615],
[0.8767, 2.28951, 1.54708, 0.06833],
[0.91815, 2.19146, 1.54708, 0.0688],
[0.96598, 2.09538, 1.54708, 0.06938],
[1.02111, 2.00191, 1.54708, 0.07014],
[1.08484, 1.91209, 1.54708, 0.07119],
[1.15889, 1.82758, 1.54708, 0.07263],
[1.24544, 1.75117, 2.1113, 0.05468],
[1.33865, 1.67987, 2.31461, 0.0507],
[1.43751, 1.61316, 2.54907, 0.04679],
[1.54115, 1.55049, 2.74552, 0.04411],
[1.64906, 1.49149, 2.98434, 0.04121],
[1.76077, 1.43573, 3.28273, 0.03803],
[1.87582, 1.38275, 3.65349, 0.03467],
[1.9938, 1.33205, 4.0, 0.0321],
[2.11442, 1.28318, 4.0, 0.03254],
[2.23741, 1.23566, 4.0, 0.03296],
[2.36252, 1.18901, 4.0, 0.03338],
[2.48957, 1.14282, 4.0, 0.03379],
[2.61829, 1.09678, 4.0, 0.03418],
[2.74813, 1.05069, 4.0, 0.03444],
[2.87901, 1.00439, 4.0, 0.03471],
[3.00883, 0.95893, 4.0, 0.03439],
[3.13759, 0.91474, 3.70786, 0.03672],
[3.26537, 0.87216, 3.45271, 0.03901],
[3.39229, 0.83155, 3.2625, 0.04084],
[3.51845, 0.79324, 3.05706, 0.04313],
[3.644, 0.7576, 2.8793, 0.04533],
[3.76905, 0.72501, 2.73985, 0.04717],
[3.89369, 0.69584, 2.63355, 0.04861],
[4.01799, 0.6705, 2.55534, 0.04965],
[4.14202, 0.64931, 2.50212, 0.05029],
[4.26578, 0.63273, 2.47146, 0.05052],
[4.38928, 0.62124, 2.46123, 0.05039],
[4.51249, 0.61526, 2.46123, 0.05012],
[4.63537, 0.61517, 2.46123, 0.04993],
[4.75784, 0.62127, 2.46123, 0.04982],
[4.87983, 0.63381, 2.46123, 0.04983],
[5.00123, 0.65297, 2.46123, 0.04993],
[5.12192, 0.67884, 2.46951, 0.04998],
[5.24177, 0.71146, 2.49455, 0.04979],
[5.36065, 0.75081, 2.53473, 0.0494],
[5.47841, 0.7968, 2.58845, 0.04884],
[5.59491, 0.84928, 2.65416, 0.04814],
[5.71, 0.90809, 2.73025, 0.04734],
[5.82355, 0.97298, 2.81513, 0.04646],
[5.93541, 1.04372, 2.90715, 0.04553],
[6.04546, 1.12002, 3.00462, 0.04457],
[6.15358, 1.20159, 3.1058, 0.04361],
[6.25965, 1.28812, 3.20888, 0.04266],
[6.36357, 1.37931, 3.31198, 0.04174],
[6.46525, 1.47484, 3.41312, 0.04088],
[6.56458, 1.57441, 3.51023, 0.04007],
[6.66149, 1.67772, 3.60115, 0.03934],
[6.75589, 1.7845, 3.68365, 0.03869],
[6.8477, 1.89447, 3.75548, 0.03815],
[6.93683, 2.00737, 3.81443, 0.03771],
[7.02319, 2.12295, 3.81007, 0.03787],
[7.10667, 2.24099, 3.74571, 0.0386],
[7.18717, 2.36126, 3.66606, 0.03948],
[7.26455, 2.48355, 3.57385, 0.04049],
[7.33867, 2.60764, 3.47254, 0.04163],
[7.40938, 2.73335, 3.3664, 0.04284],
[7.47647, 2.86045, 3.26049, 0.04408],
[7.53974, 2.98875, 3.16069, 0.04526],
[7.59898, 3.11802, 3.07359, 0.04626],
[7.65393, 3.24804, 3.00639, 0.04695],
[7.70433, 3.37858, 2.96809, 0.04715],
[7.74994, 3.50939, 2.96524, 0.04672],
[7.7905, 3.64021, 2.95543, 0.04635],
[7.82581, 3.77081, 2.83109, 0.04778],
[7.85569, 3.90093, 2.70762, 0.04931],
[7.88009, 4.03034, 2.54041, 0.05184],
[7.89904, 4.15886, 2.3869, 0.05443],
[7.91272, 4.28634, 2.15132, 0.0596],
[7.92129, 4.41264, 1.90983, 0.06628],
[7.92448, 4.53754, 1.69719, 0.07362],
[7.92201, 4.66077, 1.49715, 0.08233],
[7.9134, 4.78198, 1.31125, 0.09267],
[7.89812, 4.90076, 1.31125, 0.09133],
[7.87505, 5.01639, 1.31125, 0.08992],
[7.84264, 5.12777, 1.31125, 0.08846],
[7.79913, 5.23341, 1.31125, 0.08713],
[7.74232, 5.33104, 1.31125, 0.08614],
[7.66965, 5.41672, 1.45908, 0.077],
[7.58549, 5.49133, 1.58898, 0.07079],
[7.4925, 5.55566, 1.73035, 0.06534],
[7.39273, 5.61058, 1.88411, 0.06045],
[7.28775, 5.65703, 2.01628, 0.05694],
[7.17864, 5.69568, 2.15434, 0.05373],
[7.06626, 5.72714, 2.23936, 0.05211],
[6.9512, 5.75169, 2.30934, 0.05095],
[6.83392, 5.76949, 2.33128, 0.05088],
[6.7148, 5.78044, 2.42181, 0.04939],
[6.59428, 5.78486, 2.46678, 0.04889],
[6.47271, 5.78281, 2.50894, 0.04846],
[6.35042, 5.77439, 2.50894, 0.04886],
[6.22768, 5.75943, 2.50966, 0.04927],
[6.1048, 5.73778, 2.53671, 0.04918],
[5.98211, 5.70948, 2.58469, 0.04872],
[5.85987, 5.67465, 2.65122, 0.04794],
[5.73836, 5.6335, 2.7374, 0.04686],
[5.61779, 5.58636, 2.84615, 0.04549],
[5.49832, 5.5336, 2.9825, 0.04379],
[5.38004, 5.47566, 3.15458, 0.04175],
[5.26298, 5.41303, 3.37571, 0.03933],
[5.14712, 5.34628, 3.6686, 0.03645],
[5.03237, 5.27601, 4.0, 0.03364],
[4.91857, 5.20286, 4.0, 0.03382],
[4.80554, 5.12752, 4.0, 0.03396],
[4.69303, 5.05072, 3.91911, 0.03476],
[4.58079, 4.97316, 3.5841, 0.03806],
[4.47307, 4.89888, 3.36393, 0.0389],
[4.36498, 4.8254, 3.22268, 0.04056],
[4.25625, 4.75338, 3.14231, 0.04151],
[4.14656, 4.68349, 3.11358, 0.04177],
[4.03567, 4.61632, 3.11358, 0.04164],
[3.92336, 4.55239, 3.11358, 0.04151],
[3.80945, 4.4921, 3.11358, 0.04139],
[3.69383, 4.43575, 3.11358, 0.04131],
[3.57643, 4.38353, 3.11358, 0.04127],
[3.45722, 4.33552, 3.13215, 0.04103],
[3.33626, 4.29169, 3.19678, 0.04025],
[3.21361, 4.2519, 3.3087, 0.03897],
[3.0894, 4.21591, 3.47218, 0.03724],
[2.96382, 4.18342, 3.69543, 0.0351],
[2.83706, 4.15407, 3.99234, 0.03259],
[2.70937, 4.12744, 4.0, 0.03261],
[2.581, 4.10309, 3.93605, 0.0332],
[2.45215, 4.08058, 3.50774, 0.03729],
[2.32301, 4.05949, 3.14663, 0.04158],
[2.19838, 4.03803, 2.81246, 0.04497],
[2.07481, 4.01518, 2.51882, 0.04989],
[1.95267, 3.99042, 2.2936, 0.05433],
[1.8323, 3.96332, 2.01593, 0.0612],
[1.71409, 3.93335, 1.78398, 0.06836],
[1.59846, 3.89995, 1.53762, 0.07828],
[1.48596, 3.86244, 1.3, 0.09122],
[1.37726, 3.82008, 1.3, 0.08974],
[1.27303, 3.77217, 1.3, 0.08824],
[1.17449, 3.71757, 1.3, 0.08666],
[1.08316, 3.65504, 1.3, 0.08514],
[1.00171, 3.58284, 1.3, 0.08373],
[0.93491, 3.49914, 1.42734, 0.07502],
[0.88109, 3.4079, 1.54454, 0.06858],
[0.83879, 3.31161, 1.64772, 0.06383],
[0.80687, 3.21193, 1.74768, 0.05989],
[0.78432, 3.11009, 1.82091, 0.05729],
[0.77047, 3.00694, 1.93014, 0.05392],
[0.76441, 2.90316, 1.96678, 0.05286]]

################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)