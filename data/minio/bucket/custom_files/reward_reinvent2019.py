import math


class Reward:
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
        racing_track = [[0.3312, 2.82902, 1.30028, 0.1125],
[0.33882, 2.68171, 1.30028, 0.11344],
[0.36236, 2.53659, 1.30028, 0.11307],
[0.40087, 2.39698, 1.30507, 0.11097],
[0.45275, 2.26508, 1.31176, 0.10805],
[0.51635, 2.142, 1.31921, 0.10502],
[0.59024, 2.02819, 1.33692, 0.10149],
[0.67311, 1.92365, 1.35733, 0.09828],
[0.76393, 1.82822, 1.38414, 0.09517],
[0.8618, 1.74165, 1.41847, 0.09212],
[0.96597, 1.66362, 1.46121, 0.08907],
[1.07578, 1.59378, 1.52061, 0.08558],
[1.19062, 1.53169, 1.59994, 0.08159],
[1.30986, 1.47678, 1.65938, 0.07912],
[1.43296, 1.42844, 1.62186, 0.08154],
[1.55936, 1.38595, 1.59191, 0.08376],
[1.68849, 1.34851, 1.56949, 0.08567],
[1.8198, 1.31519, 1.55379, 0.08719],
[1.9527, 1.28493, 1.5382, 0.08861],
[2.08658, 1.25661, 1.53483, 0.08916],
[2.22298, 1.22686, 1.53483, 0.09096],
[2.35884, 1.19553, 1.53483, 0.09084],
[2.49393, 1.16191, 1.53483, 0.0907],
[2.628, 1.12536, 1.53483, 0.09054],
[2.76085, 1.08528, 1.53483, 0.09041],
[2.89229, 1.04114, 1.53483, 0.09033],
[3.02211, 0.99244, 1.53483, 0.09034],
[3.15014, 0.93869, 1.53483, 0.09047],
[3.27618, 0.87939, 1.53483, 0.09075],
[3.40004, 0.81404, 1.53483, 0.09125],
[3.52152, 0.742, 1.53483, 0.09202],
[3.64034, 0.66262, 1.53483, 0.09311],
[3.76074, 0.59177, 1.53483, 0.09102],
[3.88233, 0.52963, 1.53483, 0.08897],
[4.00501, 0.47617, 1.53483, 0.08719],
[4.12874, 0.43135, 1.53483, 0.08574],
[4.25349, 0.39509, 1.53483, 0.08464],
[4.37923, 0.36738, 1.53483, 0.08389],
[4.50595, 0.3482, 1.53483, 0.0835],
[4.6336, 0.33768, 1.53483, 0.08345],
[4.76212, 0.33589, 1.54, 0.08346],
[4.89139, 0.34292, 1.54759, 0.08365],
[5.02124, 0.35891, 1.56493, 0.0836],
[5.15142, 0.38395, 1.58956, 0.08339],
[5.2816, 0.41807, 1.62477, 0.08283],
[5.4114, 0.4612, 1.65551, 0.08262],
[5.54035, 0.5133, 1.69861, 0.08188],
[5.66795, 0.57414, 1.74608, 0.08096],
[5.79366, 0.6434, 1.80192, 0.07965],
[5.91696, 0.7206, 1.8709, 0.07776],
[6.03741, 0.80512, 1.94201, 0.07577],
[6.15464, 0.89629, 2.02687, 0.07327],
[6.26838, 0.99337, 2.12691, 0.07031],
[6.37852, 1.09559, 2.23799, 0.06714],
[6.48505, 1.20221, 2.13052, 0.07074],
[6.58809, 1.31255, 1.96143, 0.07697],
[6.68784, 1.42601, 1.83003, 0.08255],
[6.78459, 1.54207, 1.72188, 0.08775],
[6.87867, 1.6603, 1.6428, 0.09198],
[6.97035, 1.78041, 1.57631, 0.09586],
[7.05971, 1.90227, 1.52045, 0.09938],
[7.1468, 2.02575, 1.47813, 0.10223],
[7.23169, 2.15076, 1.4419, 0.1048],
[7.31445, 2.27719, 1.4134, 0.10691],
[7.39504, 2.40502, 1.39004, 0.10871],
[7.47341, 2.53422, 1.37567, 0.10984],
[7.5495, 2.66476, 1.36693, 0.11054],
[7.62327, 2.79664, 1.36328, 0.11084],
[7.69465, 2.92982, 1.36328, 0.11084],
[7.76358, 3.06429, 1.36328, 0.11084],
[7.82997, 3.20002, 1.36328, 0.11084],
[7.89374, 3.33701, 1.36328, 0.11084],
[7.95479, 3.47524, 1.36328, 0.11084],
[8.01207, 3.61501, 1.36328, 0.1108],
[8.06441, 3.75637, 1.36328, 0.11057],
[8.11059, 3.89904, 1.36328, 0.11],
[8.14944, 4.04252, 1.36328, 0.10904],
[8.1799, 4.18616, 1.36328, 0.10771],
[8.20114, 4.32922, 1.36328, 0.10608],
[8.2126, 4.47094, 1.36328, 0.1043],
[8.21392, 4.6106, 1.36328, 0.10245],
[8.20493, 4.74752, 1.36328, 0.10065],
[8.18559, 4.88108, 1.36328, 0.09899],
[8.15597, 5.01071, 1.36328, 0.09753],
[8.1162, 5.13587, 1.36328, 0.09633],
[8.06644, 5.25605, 1.36328, 0.09541],
[8.00692, 5.37079, 1.36328, 0.09481],
[7.93789, 5.47964, 1.36328, 0.09455],
[7.85959, 5.58214, 1.36531, 0.09447],
[7.77232, 5.67786, 1.37127, 0.09446],
[7.67635, 5.76633, 1.38082, 0.09453],
[7.57197, 5.84706, 1.39066, 0.09488],
[7.45949, 5.91949, 1.40348, 0.09533],
[7.33925, 5.98304, 1.41835, 0.09589],
[7.2117, 6.03706, 1.437, 0.0964],
[7.07742, 6.0809, 1.45157, 0.09731],
[6.93719, 6.11382, 1.46888, 0.09806],
[6.79213, 6.13516, 1.48699, 0.0986],
[6.64383, 6.14441, 1.50369, 0.09881],
[6.49436, 6.14136, 1.51987, 0.09836],
[6.3459, 6.12623, 1.54239, 0.09675],
[6.20016, 6.09972, 1.56605, 0.09459],
[6.05822, 6.06266, 1.59331, 0.09207],
[5.92067, 6.01592, 1.63208, 0.08901],
[5.78774, 5.96039, 1.60394, 0.08982],
[5.65947, 5.89687, 1.56189, 0.09164],
[5.53577, 5.82614, 1.52974, 0.09315],
[5.41646, 5.74889, 1.5097, 0.09415],
[5.30127, 5.66582, 1.4917, 0.09521],
[5.18985, 5.57764, 1.48342, 0.09579],
[5.08179, 5.48502, 1.48113, 0.09609],
[4.97664, 5.38868, 1.48113, 0.09629],
[4.87391, 5.28929, 1.48113, 0.09651],
[4.77307, 5.18754, 1.48113, 0.09671],
[4.67361, 5.08412, 1.48113, 0.09688],
[4.57497, 4.97969, 1.48113, 0.09699],
[4.47987, 4.87962, 1.4601, 0.09455],
[4.38389, 4.78101, 1.4257, 0.09652],
[4.28641, 4.68486, 1.39322, 0.09827],
[4.18688, 4.59216, 1.3655, 0.09961],
[4.08481, 4.50378, 1.34119, 0.10067],
[3.97981, 4.4205, 1.32388, 0.10123],
[3.87157, 4.34304, 1.31182, 0.10146],
[3.75985, 4.27207, 1.30536, 0.10139],
[3.64453, 4.20816, 1.30188, 0.10128],
[3.52552, 4.15187, 1.30124, 0.10117],
[3.40284, 4.10371, 1.3, 0.10138],
[3.27657, 4.0642, 1.3, 0.10177],
[3.14689, 4.03382, 1.3, 0.10246],
[3.01402, 4.0131, 1.3, 0.10344],
[2.87828, 4.00255, 1.3, 0.10472],
[2.74013, 4.00268, 1.3, 0.10628],
[2.6001, 4.01393, 1.3, 0.10806],
[2.45888, 4.0368, 1.3, 0.11004],
[2.31735, 4.07166, 1.3, 0.11213],
[2.17655, 4.11882, 1.3, 0.11422],
[2.03352, 4.15406, 1.3, 0.11332],
[1.89032, 4.17652, 1.3, 0.1115],
[1.74811, 4.18603, 1.3, 0.10964],
[1.60798, 4.18259, 1.3, 0.10783],
[1.47095, 4.16632, 1.3, 0.10615],
[1.33799, 4.13745, 1.3, 0.10466],
[1.20995, 4.09635, 1.3, 0.10344],
[1.08765, 4.04344, 1.3, 0.10251],
[0.97178, 3.97921, 1.3, 0.1019],
[0.86304, 3.90417, 1.3, 0.10163],
[0.7621, 3.81879, 1.3, 0.1017],
[0.66966, 3.72352, 1.30028, 0.10209],
[0.58645, 3.61884, 1.30028, 0.10284],
[0.51326, 3.50525, 1.30028, 0.10392],
[0.45096, 3.38332, 1.30028, 0.10531],
[0.4005, 3.25369, 1.30028, 0.10698],
[0.36297, 3.11721, 1.30028, 0.10886],
[0.33953, 2.97506, 1.30028, 0.1108]]

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
