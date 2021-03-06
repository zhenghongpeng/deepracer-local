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
        racing_track = [[-8.71717, -1.36222, 1.65778, 0.1768],
[-8.58469, -1.62367, 1.65778, 0.1768],
[-8.45199, -1.88501, 1.65778, 0.17681],
[-8.31933, -2.14636, 1.65778, 0.17679],
[-8.18601, -2.40622, 1.65778, 0.17618],
[-8.04862, -2.65808, 1.65778, 0.17306],
[-7.90447, -2.89589, 1.65778, 0.16775],
[-7.75208, -3.11448, 1.65778, 0.16074],
[-7.59102, -3.30998, 1.65778, 0.15279],
[-7.42172, -3.47988, 1.65778, 0.14468],
[-7.2451, -3.6227, 1.65778, 0.13702],
[-7.06236, -3.73761, 1.65778, 0.13021],
[-6.87485, -3.82399, 1.65778, 0.12454],
[-6.68397, -3.8812, 1.65778, 0.1202],
[-6.49125, -3.90797, 1.65778, 0.11736],
[-6.29854, -3.90231, 1.65778, 0.1163],
[-6.10835, -3.86091, 2.00048, 0.0973],
[-5.92151, -3.79416, 2.13967, 0.09273],
[-5.73873, -3.70399, 2.30568, 0.0884],
[-5.56052, -3.59216, 2.5158, 0.08363],
[-5.38717, -3.46066, 2.78899, 0.07802],
[-5.2187, -3.31184, 3.16675, 0.07098],
[-5.05478, -3.14866, 3.49543, 0.06617],
[-4.90684, -2.98851, 3.47104, 0.06281],
[-4.75343, -2.83125, 3.46498, 0.0634],
[-4.59419, -2.67638, 3.46498, 0.06411],
[-4.42873, -2.52351, 3.46498, 0.06501],
[-4.25656, -2.37228, 3.46498, 0.06614],
[-4.07714, -2.22234, 3.46498, 0.06748],
[-3.88986, -2.07341, 3.46498, 0.06906],
[-3.69411, -1.92524, 3.46498, 0.07086],
[-3.48939, -1.77776, 3.46498, 0.07282],
[-3.27575, -1.63131, 3.46498, 0.07475],
[-3.05504, -1.48851, 3.46498, 0.07587],
[-2.83676, -1.35639, 3.46498, 0.07364],
[-2.62478, -1.23725, 3.46498, 0.07018],
[-2.41578, -1.12905, 3.46498, 0.06792],
[-2.20729, -1.03064, 3.46498, 0.06654],
[-1.99793, -0.94163, 3.46498, 0.06566],
[-1.78703, -0.86209, 3.46498, 0.06505],
[-1.57438, -0.79231, 3.46498, 0.06459],
[-1.36008, -0.73256, 3.46498, 0.06421],
[-1.14438, -0.68307, 3.46498, 0.06387],
[-0.92758, -0.64403, 3.46498, 0.06357],
[-0.70997, -0.61551, 3.46498, 0.06334],
[-0.49175, -0.59747, 3.47197, 0.06307],
[-0.27302, -0.58983, 3.51458, 0.06227],
[-0.05375, -0.59238, 3.57404, 0.06136],
[0.16624, -0.60486, 3.6676, 0.06008],
[0.38725, -0.62698, 3.78255, 0.05872],
[0.6097, -0.65849, 3.94111, 0.05701],
[0.83411, -0.69906, 3.91105, 0.05831],
[1.06101, -0.74834, 3.80315, 0.06105],
[1.29093, -0.80589, 3.69999, 0.06406],
[1.52427, -0.8711, 3.60056, 0.06729],
[1.76114, -0.94311, 3.49366, 0.07086],
[2.00127, -1.0207, 3.39556, 0.07432],
[2.24392, -1.10229, 3.2956, 0.07768],
[2.48794, -1.18596, 2.9898, 0.08628],
[2.76239, -1.28006, 2.72394, 0.10651],
[3.03684, -1.3726, 2.51653, 0.11509],
[3.31128, -1.46213, 2.3493, 0.12288],
[3.58576, -1.54729, 2.21109, 0.12998],
[3.86033, -1.62684, 2.09462, 0.13647],
[4.13505, -1.6997, 1.99241, 0.14265],
[4.41003, -1.76485, 1.90224, 0.14856],
[4.68536, -1.82139, 1.81821, 0.15459],
[4.96111, -1.86851, 1.74015, 0.16076],
[5.23736, -1.9055, 1.74015, 0.16017],
[5.51414, -1.93157, 1.74015, 0.15976],
[5.79145, -1.94596, 1.74015, 0.15957],
[6.06925, -1.94791, 1.74015, 0.15965],
[6.34745, -1.9366, 1.74015, 0.16001],
[6.62594, -1.91113, 1.74015, 0.1607],
[6.90452, -1.87048, 1.74015, 0.16179],
[7.18294, -1.81344, 1.74015, 0.16332],
[7.4608, -1.73848, 1.74015, 0.16538],
[7.73744, -1.64391, 1.74015, 0.16801],
[8.00861, -1.52919, 1.74015, 0.1692],
[8.25935, -1.39707, 1.74015, 0.16287],
[8.48301, -1.2498, 1.74015, 0.15389],
[8.67747, -1.08932, 1.74015, 0.14489],
[8.84211, -0.91765, 1.74015, 0.13669],
[8.97688, -0.73672, 1.74015, 0.12965],
[9.08183, -0.54838, 1.74015, 0.1239],
[9.15674, -0.35435, 1.74015, 0.11952],
[9.20092, -0.15637, 1.74015, 0.11657],
[9.21285, 0.04364, 1.74015, 0.11515],
[9.18995, 0.24326, 1.83017, 0.10978],
[9.13488, 0.43999, 1.91914, 0.10645],
[9.04912, 0.63178, 2.02189, 0.10391],
[8.93374, 0.81674, 2.13806, 0.10196],
[8.78955, 0.99302, 2.27599, 0.10006],
[8.61754, 1.15885, 2.44206, 0.09784],
[8.41909, 1.31267, 2.6478, 0.09483],
[8.19635, 1.45332, 2.9143, 0.09039],
[7.95239, 1.58036, 3.26924, 0.08413],
[7.69118, 1.69434, 3.43711, 0.08292],
[7.41737, 1.79706, 3.16842, 0.0923],
[7.13575, 1.89153, 2.98261, 0.09959],
[6.85067, 1.98169, 2.83341, 0.10553],
[6.56562, 2.07196, 2.74159, 0.10906],
[6.28054, 2.16217, 2.68505, 0.11136],
[5.99549, 2.25242, 2.67012, 0.11198],
[5.71042, 2.34264, 2.67012, 0.11198],
[5.42536, 2.43288, 2.67012, 0.11198],
[5.14034, 2.52321, 2.67012, 0.11198],
[4.85544, 2.61382, 2.67012, 0.11197],
[4.57065, 2.70468, 2.67012, 0.11196],
[4.28595, 2.79577, 2.67012, 0.11195],
[4.00137, 2.88711, 2.67012, 0.11194],
[3.7169, 2.9787, 2.67012, 0.11193],
[3.43253, 3.07054, 2.67012, 0.11191],
[3.15073, 3.16086, 2.67012, 0.11083],
[2.87935, 3.24342, 2.37259, 0.11956],
[2.62255, 3.31428, 2.06022, 0.1293],
[2.3815, 3.37159, 1.84326, 0.13442],
[2.15546, 3.41502, 1.68101, 0.13692],
[1.94293, 3.44508, 1.68101, 0.12769],
[1.74238, 3.46242, 1.68101, 0.11975],
[1.55224, 3.46785, 1.68101, 0.11316],
[1.37119, 3.46204, 1.68101, 0.10776],
[1.19798, 3.44572, 1.68101, 0.1035],
[1.03146, 3.41949, 1.68101, 0.10028],
[0.87054, 3.38393, 1.68101, 0.09804],
[0.71413, 3.3395, 1.68101, 0.09673],
[0.56109, 3.28676, 1.68101, 0.0963],
[0.41032, 3.2264, 1.68101, 0.09661],
[0.26091, 3.15939, 1.57856, 0.10374],
[0.11218, 3.08698, 1.46958, 0.11257],
[-0.03629, 3.01064, 1.38022, 0.12095],
[-0.18468, 2.932, 1.3, 0.12919],
[-0.35483, 2.84105, 1.3, 0.14841],
[-0.52566, 2.7565, 1.3, 0.14662],
[-0.6976, 2.68374, 1.3, 0.14361],
[-0.87074, 2.62744, 1.3, 0.14005],
[-1.04472, 2.59156, 1.3, 0.13665],
[-1.2187, 2.5798, 1.3, 0.13413],
[-1.39126, 2.59605, 1.3, 0.13333],
[-1.5617, 2.63527, 1.3, 0.13453],
[-1.72941, 2.69657, 1.3, 0.13736],
[-1.89367, 2.77953, 1.3, 0.14155],
[-2.05325, 2.88371, 1.3, 0.1466],
[-2.20603, 3.0078, 1.3, 0.15141],
[-2.34861, 3.1482, 1.3, 0.15392],
[-2.486, 3.26297, 1.3, 0.13771],
[-2.62521, 3.35707, 1.3, 0.12926],
[-2.7658, 3.42919, 1.3, 0.12154],
[-2.90712, 3.47859, 1.3, 0.11516],
[-3.04836, 3.50456, 1.3, 0.11046],
[-3.18844, 3.5059, 1.3, 0.10776],
[-3.32585, 3.48045, 1.3, 0.1075],
[-3.45804, 3.42403, 1.4944, 0.09618],
[-3.58423, 3.34204, 1.63918, 0.09181],
[-3.70355, 3.23576, 1.73531, 0.09208],
[-3.81554, 3.10655, 1.73531, 0.09853],
[-3.92036, 2.9568, 1.73531, 0.10534],
[-4.01921, 2.79082, 1.73531, 0.11133],
[-4.11448, 2.61541, 1.73531, 0.11503],
[-4.20916, 2.44543, 1.73531, 0.11212],
[-4.30658, 2.28365, 1.73531, 0.10883],
[-4.40814, 2.13396, 1.73531, 0.10424],
[-4.51468, 1.99891, 1.73531, 0.09913],
[-4.62655, 1.87987, 1.73531, 0.09414],
[-4.7436, 1.77734, 1.73531, 0.08967],
[-4.86548, 1.69147, 1.73531, 0.08592],
[-4.9917, 1.62224, 1.73531, 0.08296],
[-5.12169, 1.56952, 1.73531, 0.08084],
[-5.25488, 1.53317, 1.73531, 0.07956],
[-5.39069, 1.51312, 1.73572, 0.07909],
[-5.52855, 1.50941, 1.74789, 0.0789],
[-5.66789, 1.52232, 1.78192, 0.07853],
[-5.80813, 1.55212, 1.83062, 0.07832],
[-5.94865, 1.59921, 1.88966, 0.07843],
[-6.08882, 1.66427, 1.96482, 0.07865],
[-6.22794, 1.74806, 2.06327, 0.07871],
[-6.36521, 1.85135, 2.02531, 0.08482],
[-6.49982, 1.97501, 1.98298, 0.09218],
[-6.63086, 2.11976, 1.93497, 0.10091],
[-6.7575, 2.28587, 1.88747, 0.11066],
[-6.87903, 2.47305, 1.83908, 0.12135],
[-6.99517, 2.67989, 1.79179, 0.13239],
[-7.10624, 2.90393, 1.74165, 0.14358],
[-7.22151, 3.11737, 1.74165, 0.13928],
[-7.34166, 3.31774, 1.74165, 0.13415],
[-7.46726, 3.50364, 1.74165, 0.12882],
[-7.59867, 3.67449, 1.74165, 0.12375],
[-7.73617, 3.83017, 1.74165, 0.11926],
[-7.88014, 3.97077, 1.74165, 0.11554],
[-8.03107, 4.09631, 1.74165, 0.11272],
[-8.18959, 4.20679, 1.74165, 0.11094],
[-8.35648, 4.30184, 1.74165, 0.11028],
[-8.53265, 4.38068, 1.74165, 0.11081],
[-8.7189, 4.44188, 1.74165, 0.11256],
[-8.91574, 4.48305, 1.74165, 0.11547],
[-9.12254, 4.50098, 1.74165, 0.11919],
[-9.3365, 4.49185, 1.74165, 0.12296],
[-9.55156, 4.45248, 1.74165, 0.12554],
[-9.75915, 4.38206, 1.74165, 0.12586],
[-9.95082, 4.2824, 1.74165, 0.12404],
[-10.12066, 4.1568, 1.74165, 0.12129],
[-10.26516, 4.0083, 1.74165, 0.11897],
[-10.38173, 3.83904, 1.74165, 0.118],
[-10.46706, 3.65034, 1.99726, 0.10369],
[-10.52682, 3.44805, 2.1077, 0.10008],
[-10.56159, 3.23399, 2.23158, 0.09718],
[-10.57165, 3.00963, 2.37606, 0.09452],
[-10.55725, 2.77625, 2.55145, 0.09164],
[-10.51887, 2.53511, 2.76981, 0.08815],
[-10.45745, 2.28749, 3.05581, 0.08349],
[-10.37471, 2.03469, 2.91364, 0.09129],
[-10.27338, 1.77799, 2.6144, 0.10556],
[-10.15721, 1.51856, 2.38991, 0.11894],
[-10.03094, 1.2574, 2.2122, 0.13113],
[-9.9, 0.99526, 2.06667, 0.14178],
[-9.76927, 0.73303, 1.94613, 0.15057],
[-9.63861, 0.47073, 1.83605, 0.1596],
[-9.50769, 0.20855, 1.74226, 0.1682],
[-9.37652, -0.05352, 1.65778, 0.17678],
[-9.2451, -0.31548, 1.65778, 0.17678],
[-9.11346, -0.57732, 1.65778, 0.17679],
[-8.98159, -0.83906, 1.65778, 0.17679],
[-8.84949, -1.10069, 1.65778, 0.1768]]

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
