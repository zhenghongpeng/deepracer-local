import math
import numpy as np


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
        
        # Object Avoidance
        def ob_function(params):
            '''
            Example of rewarding the agent to stay inside two borders
            and penalizing getting too close to the objects in front
            '''

            all_wheels_on_track = params['all_wheels_on_track']
            distance_from_center = params['distance_from_center']
            track_width = params['track_width']
            objects_distance = params['objects_distance']
            _, next_object_index = params['closest_objects']
            objects_left_of_center = params['objects_left_of_center']
            is_left_of_center = params['is_left_of_center']

            # Initialize reward with a small number but not zero
            # because zero means off-track or crashed
            reward = 1e-3

            # Penalize if the agent is too close to the next object
            reward_avoid = 1.0

            # Distance to the next object
            distance_closest_object = objects_distance[next_object_index]
            # Decide if the agent and the next object is on the same lane
            is_same_lane = objects_left_of_center[next_object_index] == is_left_of_center
            
            if is_same_lane:
                if 0.5 <= distance_closest_object < 0.8: 
                    reward_avoid *= 0.5
                elif 0.3 <= distance_closest_object < 0.5:
                    reward_avoid *= 0.2
                elif distance_closest_object < 0.3:
                    reward_avoid = 1e-3 # Likely crashed

            # Calculate reward by putting different weights on 
            # the two aspects above
            reward += 1.0 * reward_avoid

            return reward

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = np.array([[5.69112, -3.48531, 4.0, 0.03756],
[5.54088, -3.484, 4.0, 0.03756],
[5.39063, -3.4827, 4.0, 0.03756],
[5.24039, -3.48138, 4.0, 0.03756],
[5.09015, -3.48007, 4.0, 0.03756],
[4.93991, -3.47876, 4.0, 0.03756],
[4.78967, -3.47744, 4.0, 0.03756],
[4.63943, -3.47613, 4.0, 0.03756],
[4.48918, -3.47481, 4.0, 0.03756],
[4.33894, -3.47349, 4.0, 0.03756],
[4.1887, -3.47217, 4.0, 0.03756],
[4.03846, -3.47086, 4.0, 0.03756],
[3.88822, -3.46955, 4.0, 0.03756],
[3.73798, -3.46825, 4.0, 0.03756],
[3.58773, -3.46697, 4.0, 0.03756],
[3.43752, -3.46563, 4.0, 0.03756],
[3.28736, -3.46418, 4.0, 0.03754],
[3.13727, -3.46258, 4.0, 0.03752],
[2.98725, -3.46083, 4.0, 0.03751],
[2.83739, -3.45876, 4.0, 0.03747],
[2.68778, -3.4562, 4.0, 0.03741],
[2.53851, -3.45295, 4.0, 0.03733],
[2.38969, -3.44881, 4.0, 0.03722],
[2.24145, -3.44358, 4.0, 0.03708],
[2.09391, -3.43702, 4.0, 0.03692],
[1.9472, -3.42891, 4.0, 0.03673],
[1.80148, -3.41901, 3.94201, 0.03705],
[1.65691, -3.40701, 3.76929, 0.03849],
[1.51364, -3.3927, 3.62439, 0.03973],
[1.3718, -3.37587, 3.50589, 0.04074],
[1.23156, -3.35627, 3.41654, 0.04145],
[1.09305, -3.33371, 3.35826, 0.04179],
[0.95642, -3.30797, 3.33281, 0.04172],
[0.82183, -3.27884, 3.33281, 0.04132],
[0.68939, -3.24615, 3.33281, 0.04093],
[0.55922, -3.20978, 3.33281, 0.04055],
[0.4314, -3.16963, 3.33281, 0.0402],
[0.30598, -3.12569, 3.33281, 0.03988],
[0.18294, -3.07798, 3.34297, 0.03947],
[0.06225, -3.0266, 3.39355, 0.03865],
[-0.05617, -2.97174, 3.49256, 0.03737],
[-0.17249, -2.91363, 3.65169, 0.03561],
[-0.28687, -2.85259, 3.88447, 0.03338],
[-0.39956, -2.78897, 4.0, 0.03235],
[-0.51079, -2.72317, 4.0, 0.03231],
[-0.62076, -2.65547, 4.0, 0.03228],
[-0.72956, -2.58603, 3.74962, 0.03442],
[-0.83723, -2.51494, 3.55757, 0.03627],
[-0.94367, -2.44243, 3.39659, 0.03792],
[-1.05101, -2.37192, 3.22683, 0.0398],
[-1.1594, -2.30368, 3.06471, 0.04179],
[-1.26892, -2.23801, 2.92754, 0.04362],
[-1.37969, -2.1753, 2.82274, 0.04509],
[-1.49178, -2.11589, 2.74753, 0.04617],
[-1.60522, -2.0601, 2.69754, 0.04686],
[-1.72006, -2.00832, 2.67038, 0.04718],
[-1.83634, -1.96098, 2.66545, 0.0471],
[-1.95407, -1.91851, 2.66545, 0.04696],
[-2.07323, -1.88127, 2.66545, 0.04684],
[-2.19374, -1.84957, 2.66545, 0.04675],
[-2.3155, -1.82362, 2.66545, 0.04671],
[-2.4384, -1.80357, 2.66545, 0.04672],
[-2.56229, -1.78947, 2.68336, 0.04647],
[-2.68703, -1.7813, 2.72571, 0.04586],
[-2.81246, -1.77894, 2.79537, 0.04488],
[-2.93845, -1.78217, 2.89709, 0.0435],
[-3.06485, -1.79067, 3.03873, 0.04169],
[-3.19155, -1.80405, 3.23425, 0.03939],
[-3.31845, -1.82179, 3.51093, 0.0365],
[-3.44548, -1.8433, 3.91556, 0.0329],
[-3.57257, -1.86787, 4.0, 0.03236],
[-3.6997, -1.89484, 4.0, 0.03249],
[-3.82684, -1.92357, 3.96376, 0.03288],
[-3.95398, -1.95357, 3.50264, 0.0373],
[-4.07768, -1.98199, 3.15517, 0.04023],
[-4.20141, -2.00914, 2.77395, 0.04566],
[-4.32517, -2.03465, 2.40487, 0.05255],
[-4.44898, -2.05783, 2.11509, 0.05955],
[-4.57283, -2.07821, 2.11509, 0.05934],
[-4.69673, -2.09501, 2.11509, 0.05911],
[-4.82066, -2.10747, 2.11509, 0.05889],
[-4.94461, -2.11437, 2.11509, 0.05869],
[-5.06848, -2.11391, 2.11509, 0.05857],
[-5.1921, -2.10398, 2.28831, 0.05419],
[-5.31531, -2.08589, 2.46128, 0.0506],
[-5.43803, -2.06066, 2.57801, 0.0486],
[-5.56017, -2.02883, 2.69215, 0.04689],
[-5.68167, -1.9908, 2.79175, 0.0456],
[-5.80245, -1.9469, 2.81171, 0.0457],
[-5.92238, -1.89708, 2.95783, 0.04391],
[-6.04141, -1.84177, 2.99018, 0.0439],
[-6.15943, -1.78096, 3.03817, 0.0437],
[-6.27628, -1.71472, 3.13138, 0.0429],
[-6.39188, -1.64324, 3.25103, 0.0418],
[-6.50612, -1.56684, 3.3836, 0.04062],
[-6.61891, -1.48583, 3.52222, 0.03943],
[-6.7302, -1.40054, 3.66301, 0.03828],
[-6.8399, -1.31131, 3.80324, 0.03718],
[-6.94796, -1.21842, 3.94055, 0.03616],
[-7.05434, -1.12219, 4.0, 0.03586],
[-7.15897, -1.02286, 4.0, 0.03607],
[-7.26183, -0.92069, 4.0, 0.03624],
[-7.36286, -0.81592, 4.0, 0.03639],
[-7.46204, -0.70873, 4.0, 0.03651],
[-7.55932, -0.59932, 4.0, 0.0366],
[-7.65468, -0.48786, 4.0, 0.03667],
[-7.74806, -0.3745, 4.0, 0.03672],
[-7.83943, -0.25938, 4.0, 0.03674],
[-7.92874, -0.14263, 4.0, 0.03675],
[-8.01593, -0.02437, 4.0, 0.03673],
[-8.10094, 0.09528, 4.0, 0.03669],
[-8.18369, 0.21621, 4.0, 0.03663],
[-8.2641, 0.33832, 4.0, 0.03655],
[-8.34206, 0.4615, 4.0, 0.03644],
[-8.41746, 0.58565, 3.95037, 0.03677],
[-8.49015, 0.71066, 3.79547, 0.0381],
[-8.55999, 0.83643, 3.63803, 0.03954],
[-8.6268, 0.96285, 3.48093, 0.04108],
[-8.69038, 1.08981, 3.3267, 0.04268],
[-8.75051, 1.2172, 3.17752, 0.04433],
[-8.80696, 1.34491, 3.03517, 0.046],
[-8.85947, 1.47282, 2.90106, 0.04766],
[-8.90777, 1.60079, 2.77629, 0.04927],
[-8.95156, 1.72872, 2.66164, 0.0508],
[-8.99056, 1.85644, 2.55765, 0.05221],
[-9.02445, 1.98382, 2.46464, 0.05348],
[-9.05292, 2.11068, 2.38275, 0.05457],
[-9.07567, 2.23686, 2.31194, 0.05546],
[-9.09241, 2.36215, 2.25198, 0.05613],
[-9.10284, 2.48634, 2.20246, 0.05659],
[-9.10672, 2.6092, 2.16281, 0.05683],
[-9.10382, 2.73046, 2.13222, 0.05689],
[-9.09393, 2.84984, 2.10972, 0.05678],
[-9.07692, 2.96705, 2.09411, 0.05655],
[-9.05267, 3.08174, 2.08405, 0.05625],
[-9.02114, 3.19358, 2.07805, 0.05592],
[-8.98234, 3.30219, 2.07457, 0.0556],
[-8.93632, 3.40719, 2.07214, 0.05533],
[-8.88323, 3.50818, 2.06953, 0.05513],
[-8.82325, 3.60475, 2.06597, 0.05502],
[-8.75665, 3.69648, 2.06151, 0.05499],
[-8.68375, 3.78296, 2.0573, 0.05498],
[-8.60494, 3.86377, 2.05592, 0.0549],
[-8.52064, 3.9385, 2.05592, 0.0548],
[-8.43135, 4.00677, 2.05592, 0.05467],
[-8.33759, 4.06821, 2.05592, 0.05452],
[-8.23993, 4.12249, 2.05592, 0.05435],
[-8.13894, 4.16936, 2.05592, 0.05415],
[-8.03523, 4.20863, 2.06151, 0.0538],
[-7.92936, 4.24022, 2.07957, 0.05313],
[-7.8219, 4.2642, 2.11551, 0.05205],
[-7.71336, 4.28078, 1.95003, 0.05631],
[-7.60418, 4.29036, 1.77735, 0.06166],
[-7.49476, 4.29345, 1.59495, 0.06863],
[-7.38539, 4.29027, 1.41986, 0.07706],
[-7.27639, 4.28035, 1.41986, 0.07708],
[-7.16817, 4.26312, 1.41986, 0.07718],
[-7.06133, 4.23724, 1.41986, 0.07742],
[-6.95683, 4.20097, 1.41986, 0.07791],
[-6.85624, 4.15176, 1.41986, 0.07887],
[-6.76238, 4.08611, 2.01428, 0.05687],
[-6.6726, 4.01219, 2.25059, 0.05167],
[-6.58642, 3.93145, 2.51979, 0.04687],
[-6.50333, 3.84511, 2.86799, 0.04178],
[-6.42277, 3.75432, 3.36158, 0.03611],
[-6.34415, 3.66019, 4.0, 0.03066],
[-6.2668, 3.5639, 4.0, 0.03088],
[-6.19008, 3.46653, 3.82904, 0.03237],
[-6.11097, 3.36771, 3.50868, 0.03608],
[-6.03102, 3.26997, 3.23649, 0.03902],
[-5.95011, 3.17371, 2.90499, 0.04328],
[-5.86812, 3.07941, 2.63215, 0.04747],
[-5.78496, 2.98752, 2.37866, 0.0521],
[-5.70053, 2.89849, 2.10315, 0.05834],
[-5.61473, 2.81282, 1.87746, 0.06458],
[-5.52743, 2.73104, 1.66645, 0.07178],
[-5.43842, 2.65391, 1.47364, 0.07992],
[-5.34749, 2.58225, 1.3, 0.08906],
[-5.25441, 2.517, 1.3, 0.08744],
[-5.15888, 2.45959, 1.3, 0.08574],
[-5.06065, 2.41158, 1.3, 0.0841],
[-4.95953, 2.37505, 1.3, 0.0827],
[-4.85552, 2.35281, 1.3, 0.08182],
[-4.74913, 2.34893, 1.53174, 0.0695],
[-4.64199, 2.35856, 1.69363, 0.06352],
[-4.53473, 2.37963, 1.87532, 0.05829],
[-4.42772, 2.41049, 2.06085, 0.05404],
[-4.32115, 2.44986, 2.26911, 0.05007],
[-4.21514, 2.49663, 2.51253, 0.04611],
[-4.10973, 2.54972, 2.81416, 0.04194],
[-4.00491, 2.6081, 3.20098, 0.03748],
[-3.9006, 2.67072, 3.80869, 0.03194],
[-3.7967, 2.73644, 4.0, 0.03074],
[-3.69307, 2.80422, 3.6111, 0.03429],
[-3.58959, 2.87301, 3.20563, 0.03877],
[-3.48374, 2.94195, 2.82194, 0.04476],
[-3.37748, 3.00935, 2.52585, 0.04982],
[-3.27069, 3.07467, 2.22337, 0.0563],
[-3.16323, 3.13736, 1.98571, 0.06265],
[-3.05497, 3.19677, 1.62211, 0.07613],
[-2.94574, 3.2521, 1.62211, 0.07548],
[-2.83535, 3.30226, 1.62211, 0.07475],
[-2.72368, 3.34614, 1.62211, 0.07397],
[-2.61057, 3.38208, 1.62211, 0.07316],
[-2.49604, 3.40828, 1.62211, 0.07243],
[-2.38027, 3.42015, 1.77534, 0.06555],
[-2.26427, 3.42018, 1.92584, 0.06023],
[-2.14855, 3.41017, 2.05146, 0.05662],
[-2.03344, 3.39129, 2.13906, 0.05453],
[-1.91919, 3.36414, 2.28431, 0.05141],
[-1.80594, 3.32964, 2.3999, 0.04933],
[-1.69381, 3.28836, 2.47291, 0.04832],
[-1.58293, 3.24057, 2.54526, 0.04744],
[-1.47343, 3.18648, 2.64399, 0.04619],
[-1.36537, 3.12641, 2.69956, 0.0458],
[-1.25888, 3.06046, 2.74718, 0.04559],
[-1.1541, 2.98867, 2.82021, 0.04504],
[-1.05111, 2.91118, 2.94146, 0.04382],
[-0.94998, 2.8283, 3.10784, 0.04207],
[-0.85067, 2.74046, 3.31746, 0.03997],
[-0.75311, 2.64816, 3.5749, 0.03757],
[-0.6572, 2.55196, 3.88603, 0.03496],
[-0.56277, 2.4524, 4.0, 0.0343],
[-0.46964, 2.35009, 4.0, 0.03459],
[-0.37773, 2.2453, 4.0, 0.03485],
[-0.28699, 2.13833, 4.0, 0.03507],
[-0.19744, 2.02933, 3.86264, 0.03652],
[-0.10933, 1.91859, 3.64453, 0.03883],
[-0.02027, 1.81057, 3.43787, 0.04072],
[0.07012, 1.70511, 3.26634, 0.04252],
[0.162, 1.60238, 3.10922, 0.04433],
[0.25551, 1.50259, 2.97322, 0.046],
[0.35077, 1.40594, 2.85938, 0.04746],
[0.44793, 1.31275, 2.76552, 0.04868],
[0.54715, 1.22338, 2.68987, 0.04964],
[0.64856, 1.13818, 2.63062, 0.05035],
[0.75226, 1.05751, 2.58565, 0.05081],
[0.85834, 0.98175, 2.55264, 0.05107],
[0.96685, 0.91123, 2.52934, 0.05116],
[1.07778, 0.84627, 2.51366, 0.05114],
[1.19108, 0.78717, 2.50375, 0.05104],
[1.30665, 0.73418, 2.4981, 0.0509],
[1.42434, 0.68751, 2.49568, 0.05073],
[1.54395, 0.64735, 2.49568, 0.05056],
[1.66524, 0.61384, 2.49568, 0.05042],
[1.78793, 0.5871, 2.49568, 0.05031],
[1.9117, 0.56722, 2.49568, 0.05023],
[2.03623, 0.55424, 2.49568, 0.05017],
[2.16115, 0.5482, 2.49602, 0.05011],
[2.28612, 0.54908, 2.49925, 0.05],
[2.41079, 0.55686, 2.5062, 0.04984],
[2.53481, 0.57148, 2.51829, 0.04959],
[2.65788, 0.59281, 2.53756, 0.04922],
[2.77971, 0.62073, 2.56662, 0.0487],
[2.90008, 0.65504, 2.60865, 0.04798],
[3.01879, 0.69551, 2.66747, 0.04702],
[3.13571, 0.74185, 2.74778, 0.04577],
[3.25077, 0.7937, 2.85572, 0.04419],
[3.36396, 0.85067, 3.00005, 0.04224],
[3.47535, 0.91227, 3.1947, 0.03984],
[3.58504, 0.97798, 3.46228, 0.03693],
[3.69322, 1.04721, 3.83461, 0.03349],
[3.80011, 1.11932, 4.0, 0.03223],
[3.90592, 1.19364, 4.0, 0.03233],
[4.01067, 1.26949, 3.77012, 0.0343],
[4.11409, 1.34596, 3.24622, 0.03962],
[4.21557, 1.42208, 2.84614, 0.04457],
[4.31574, 1.49664, 2.49454, 0.05006],
[4.41641, 1.57002, 2.13929, 0.05823],
[4.51788, 1.6415, 1.84703, 0.0672],
[4.62054, 1.71017, 1.84703, 0.06687],
[4.72477, 1.77513, 1.84703, 0.06649],
[4.83097, 1.83531, 1.84703, 0.06609],
[4.93968, 1.88938, 1.84703, 0.06573],
[5.05168, 1.9352, 1.84703, 0.06551],
[5.16789, 1.96991, 2.0604, 0.05886],
[5.28738, 1.9955, 2.23154, 0.05476],
[5.40962, 2.01309, 2.38489, 0.05178],
[5.53427, 2.0234, 2.62664, 0.04762],
[5.66091, 2.02746, 2.82989, 0.04477],
[5.78927, 2.02594, 3.03307, 0.04232],
[5.91915, 2.01935, 3.20841, 0.04053],
[6.0504, 2.00806, 3.37411, 0.03904],
[6.1829, 1.99236, 3.53557, 0.03774],
[6.31654, 1.97248, 3.70503, 0.03647],
[6.45124, 1.94867, 3.89074, 0.03516],
[6.58688, 1.92116, 4.0, 0.0346],
[6.72338, 1.89022, 4.0, 0.03499],
[6.86064, 1.85612, 4.0, 0.03536],
[6.99856, 1.81918, 4.0, 0.0357],
[7.13706, 1.77966, 4.0, 0.03601],
[7.27605, 1.73787, 4.0, 0.03628],
[7.41544, 1.69411, 4.0, 0.03652],
[7.55516, 1.64867, 4.0, 0.03673],
[7.69514, 1.60181, 4.0, 0.0369],
[7.83533, 1.55377, 4.0, 0.03705],
[7.97569, 1.50476, 4.0, 0.03717],
[8.11618, 1.45497, 4.0, 0.03726],
[8.25676, 1.40455, 4.0, 0.03734],
[8.39741, 1.35368, 4.0, 0.03739],
[8.5381, 1.3025, 4.0, 0.03743],
[8.67882, 1.25114, 4.0, 0.03745],
[8.81956, 1.19958, 4.0, 0.03747],
[8.96033, 1.14788, 4.0, 0.03749],
[9.1011, 1.09604, 4.0, 0.0375],
[9.2419, 1.04406, 4.0, 0.03752],
[9.3827, 0.99199, 4.0, 0.03753],
[9.52352, 0.93978, 4.0, 0.03755],
[9.66435, 0.88743, 4.0, 0.03756],
[9.80519, 0.8351, 4.0, 0.03756],
[9.94602, 0.78276, 4.0, 0.03756],
[10.0867, 0.73023, 4.0, 0.03754],
[10.22706, 0.67733, 4.0, 0.0375],
[10.36694, 0.62384, 4.0, 0.03744],
[10.50612, 0.56954, 4.0, 0.03735],
[10.6444, 0.51421, 4.0, 0.03723],
[10.78156, 0.45764, 4.0, 0.03709],
[10.91728, 0.39954, 3.86997, 0.03815],
[11.05143, 0.3398, 3.6758, 0.03995],
[11.1837, 0.27819, 3.49847, 0.04171],
[11.31386, 0.21454, 3.33326, 0.04347],
[11.44163, 0.14867, 3.17914, 0.04522],
[11.5667, 0.08039, 3.03666, 0.04692],
[11.68877, 0.00954, 2.90529, 0.04858],
[11.80751, -0.06401, 2.78393, 0.05017],
[11.9226, -0.14039, 2.67165, 0.0517],
[12.03368, -0.2197, 2.56781, 0.05316],
[12.1404, -0.30203, 2.47208, 0.05452],
[12.24236, -0.38743, 2.38434, 0.05578],
[12.3392, -0.47594, 2.30459, 0.05692],
[12.43052, -0.56752, 2.2329, 0.05792],
[12.51592, -0.66214, 2.16924, 0.05876],
[12.59501, -0.75971, 2.11351, 0.05943],
[12.66741, -0.86008, 2.06545, 0.05992],
[12.73272, -0.96306, 2.02477, 0.06023],
[12.79059, -1.06843, 1.99119, 0.06037],
[12.84067, -1.1759, 1.96453, 0.06035],
[12.88265, -1.28514, 1.94468, 0.06018],
[12.91626, -1.39577, 1.93166, 0.05986],
[12.94127, -1.50737, 1.92562, 0.05939],
[12.95751, -1.61947, 1.92562, 0.05882],
[12.96483, -1.7316, 1.92562, 0.05835],
[12.96315, -1.84325, 1.92562, 0.05798],
[12.95246, -1.95389, 1.92562, 0.05772],
[12.93276, -2.063, 1.92562, 0.05758],
[12.90415, -2.17007, 1.92677, 0.05752],
[12.86674, -2.27459, 1.93543, 0.05736],
[12.82071, -2.3761, 1.95194, 0.0571],
[12.76628, -2.47413, 1.97669, 0.05672],
[12.70373, -2.56828, 2.01008, 0.05624],
[12.63336, -2.6582, 2.05251, 0.05563],
[12.5555, -2.74357, 2.10437, 0.05491],
[12.47053, -2.82414, 2.16605, 0.05406],
[12.37884, -2.89972, 2.23792, 0.0531],
[12.28085, -2.97017, 2.3204, 0.05201],
[12.17698, -3.03542, 2.4139, 0.05082],
[12.06765, -3.09547, 2.51894, 0.04951],
[11.95332, -3.15035, 2.63611, 0.04811],
[11.8344, -3.20016, 2.76613, 0.04661],
[11.71133, -3.24505, 2.90989, 0.04502],
[11.5845, -3.28521, 3.06847, 0.04335],
[11.45432, -3.32088, 3.2432, 0.04162],
[11.32116, -3.35229, 3.43573, 0.03982],
[11.18536, -3.37975, 3.648, 0.03798],
[11.04725, -3.40354, 3.8824, 0.0361],
[10.90714, -3.42396, 4.0, 0.0354],
[10.7653, -3.44133, 4.0, 0.03573],
[10.62197, -3.45595, 4.0, 0.03602],
[10.47738, -3.46812, 4.0, 0.03628],
[10.33172, -3.47811, 4.0, 0.0365],
[10.18517, -3.48621, 4.0, 0.03669],
[10.03787, -3.49266, 4.0, 0.03686],
[9.88997, -3.4977, 4.0, 0.037],
[9.74156, -3.50152, 4.0, 0.03711],
[9.59274, -3.50432, 4.0, 0.03721],
[9.44359, -3.50627, 4.0, 0.03729],
[9.29419, -3.50752, 4.0, 0.03735],
[9.14457, -3.5082, 4.0, 0.0374],
[8.9948, -3.50841, 4.0, 0.03744],
[8.84491, -3.50828, 4.0, 0.03747],
[8.69496, -3.50799, 4.0, 0.03749],
[8.54497, -3.50755, 4.0, 0.0375],
[8.39495, -3.50701, 4.0, 0.03751],
[8.24488, -3.50633, 4.0, 0.03752],
[8.09478, -3.50553, 4.0, 0.03753],
[7.94464, -3.50459, 4.0, 0.03754],
[7.79447, -3.50353, 4.0, 0.03754],
[7.64426, -3.50235, 4.0, 0.03755],
[7.49402, -3.50104, 4.0, 0.03756],
[7.34378, -3.49973, 4.0, 0.03756],
[7.19353, -3.49842, 4.0, 0.03756],
[7.04329, -3.49711, 4.0, 0.03756],
[6.89305, -3.4958, 4.0, 0.03756],
[6.74281, -3.49448, 4.0, 0.03756],
[6.59257, -3.49317, 4.0, 0.03756],
[6.44233, -3.49186, 4.0, 0.03756],
[6.29208, -3.49055, 4.0, 0.03756],
[6.14184, -3.48924, 4.0, 0.03756],
[5.9916, -3.48793, 4.0, 0.03756],
[5.84136, -3.48662, 4.0, 0.03756]])

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
        
        ## reward object avoidance

        reward +=ob_function(params)

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