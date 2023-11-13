# -*- coding: utf-8 -*-

# need to start from scratch. I will attempt to make the intersection then
# the roundabout

import numpy as np
import random as rand
import matplotlib.pyplot as plt

VMAX = 5
LENGTH = 100
#CARS = 150
#CARS_GIVE_WAY = -1
ITERATIONS = 5000
ROUND_ABOUT_LEN = 30
VMAX_ROUND_ABOUT = 3
CARS_FIXED = 0
flow_rate = []
car_density = []
INTERSECTION = int(LENGTH/2)
OVERLAPS = [int(0), int(ROUND_ABOUT_LEN/2), [int(3*ROUND_ABOUT_LEN/4), int(1*ROUND_ABOUT_LEN/4)]]
VMAX_EXIT = 4
def acceleration(velocity, distance):
    """
    Uses the velocity of the car under investigation and compares it to the
    distance to the car in front of it. If the distance is bigger than the
    1 plus the velocity then the car accelerates by 1

    """
    #print(distance)
    #print(velocity)
    if distance > velocity + 1:
        if velocity < VMAX:
            velocity = velocity + 1
            #print("huh")
    return velocity

def deceleration(velocity, distance):
    """
    Uses the velocity of the car under investigation and compares it to the
    distance to the car in front of it. If the distance is equal or less
    than the velocity (in magnitude) then the car's velocity will be decreased
    to the distance subtraced by 1
    """
    if distance <= velocity:
        velocity = (distance) - 1
    return velocity

def random(vehicle_v, v_max = VMAX):
    """
    According to some probability the car under investigation can decelerate
    by 1 if its velocity is not less than 1
    """
    probability = rand.uniform(0,1)
    if probability < 0.3:
        if vehicle_v > 0:
            vehicle_v -= 1
    #     else:
    #         return vehicle_v
    # else:
    #     return vehicle_v
    return vehicle_v




def intersection_check_outside_new(i, current_car_position, current_car_velocity,
                                   next_road, current_road, all_car_positions, j):
    entrance = OVERLAPS[i]
    if current_car_position == INTERSECTION:
        for k in range(VMAX + 1):
            if current_road[2][0, entrance - k] >= 0:
                if entrance - k + current_road[2][0, entrance - k] >= entrance:
                    current_car_velocity = 0
                    next_road[i][current_car_position] = current_car_velocity
                    return next_road

        for v in range(VMAX):
            if current_road[2][0, entrance + v + 1] >= 0:
                if v == 0:
                    current_car_velocity = 0
                    next_road[i][current_car_position] = current_car_velocity
                    return next_road
                elif v > VMAX_ROUND_ABOUT - 1:
                    current_car_velocity = VMAX_ROUND_ABOUT
                    next_road[2][0, entrance + current_car_velocity - 1] = current_car_velocity
                    next_road[2][1, entrance + current_car_velocity - 1] = i
                    #print("guh")
                    return next_road
                else:
                    current_car_velocity = v + 1
                    next_road[2][0, entrance + current_car_velocity - 1] = current_car_velocity
                    next_road[2][1, entrance + current_car_velocity - 1] = i
                    #print("guh")
                    return next_road

        current_car_velocity = VMAX_ROUND_ABOUT
        next_road[2][0, entrance + current_car_velocity - 1] = current_car_velocity
        next_road[2][1, entrance + current_car_velocity - 1] = i
        #print("guh")
        return next_road

    else:
        if j == len(all_car_positions) - 1:
            current_car_velocity = INTERSECTION - current_car_position
            next_road[i][current_car_velocity + current_car_position] = current_car_velocity
            return next_road
        elif all_car_positions[j + 1] > INTERSECTION:
            current_car_velocity = INTERSECTION - current_car_position
            next_road[i][current_car_velocity + current_car_position] = current_car_velocity
            return next_road
        else:
            return False

def intersection_check_inside_new(i, current_car_position, current_car_velocity, next_road,
                                  current_road, all_car_positions, j):
    straight_road = current_road[i][1, current_car_position]
    exit_index = OVERLAPS[i][straight_road]
    #print(current_car_position)
    if current_car_position == exit_index - 1:
        for v in range(VMAX):
            if current_road[straight_road][INTERSECTION + v + 1] >= 0:
                if v == 0:
                    current_car_velocity = 0
                    next_road[i][0, current_car_position] = current_car_velocity
                    next_road[i][1, current_car_position] = straight_road
                    return next_road
                elif v > VMAX_EXIT - 1:
                    current_car_velocity = VMAX_EXIT
                    next_road[straight_road][INTERSECTION + current_car_velocity - 1] = current_car_velocity
                    #print("buh")
                    return next_road
                else:
                    current_car_velocity = v + 1
                    next_road[straight_road][INTERSECTION + current_car_velocity - 1] = current_car_velocity
                    #print("buh")
                    return next_road
        current_car_velocity = VMAX_EXIT
        next_road[straight_road][INTERSECTION + current_car_velocity - 1] = current_car_velocity
        #print("buh")
        return next_road

    else:
        if j == len(all_car_positions) - 1:
            current_car_velocity = exit_index - current_car_position - 1
            next_road[i][0, current_car_position + current_car_velocity] = current_car_velocity
            next_road[i][1, current_car_position + current_car_velocity] = straight_road
            return next_road
        elif all_car_positions[j + 1] >= exit_index:
            current_car_velocity = exit_index - current_car_position - 1
            next_road[i][0, current_car_position + current_car_velocity] = current_car_velocity
            next_road[i][1, current_car_position + current_car_velocity] = straight_road
            return next_road
        else:
            return False



def standard_road_rules(next_car_position, current_car_position, current_car_velocity, next_road, i, cars_passing):

    #print(current_car_velocity)
    #print(next_car_position)
    #print(current_car_position)
    distance_to_next_car = next_car_position - current_car_position
    #print(distance_to_next_car)
    #print("")
    #current_car_velocity = current_road[i, current_car_position]

    current_car_velocity = acceleration(current_car_velocity, distance_to_next_car)
    #print(current_car_velocity)
    current_car_velocity = deceleration(current_car_velocity, distance_to_next_car)
    #print(current_car_velocity)
    current_car_velocity = random(current_car_velocity)
    future_car_position = current_car_velocity + current_car_position
    #if current_car_position < LENGTH:
    #print(current_car_velocity)
    #print("")
    if future_car_position >= LENGTH:
        future_car_position -= LENGTH
        next_road[future_car_position] = current_car_velocity
        #current_road[i, current_car_position] = current_car_velocity
        cars_passing += 1
        # if i == 1:
        #         cars_passing += 1
        # else:
        #         pass
    else:
        next_road[future_car_position] = current_car_velocity
    #current_road[i, current_car_position] = current_car_velocity
    return next_road, cars_passing

def standard_roundabout_rules(next_car_position, current_car_position, current_car_velocity, next_road, i, current_road):


    distance_to_next_car = next_car_position - current_car_position
    #current_car_velocity = current_road[i, current_car_position]

    current_car_velocity = acceleration(current_car_velocity, distance_to_next_car)
    current_car_velocity = deceleration(current_car_velocity, distance_to_next_car)
    current_car_velocity = random(current_car_velocity)
    future_car_position = current_car_velocity + current_car_position

    if future_car_position >= ROUND_ABOUT_LEN:
        #print(future_car_position)
        future_car_position -= ROUND_ABOUT_LEN
        #print(future_car_position)
        next_road[0, future_car_position] = current_car_velocity
        #print("guh")
        #print(current_car_position)
        next_road[1, future_car_position] = current_road[1, current_car_position]

        #current_road[i, current_car_position] = current_car_velocity

    else:
        next_road[0, future_car_position] = current_car_velocity
        #print(next_road[1, 5])
        next_road[1, future_car_position] = current_road[1, current_car_position]
        #current_road[i, current_car_position] = current_car_velocity
    return next_road


def car():
    """
    generates a random velocity and a random position on the road if the
    function is called. This basically defines a car
    """
    velocity = rand.randint(0, VMAX)
    position = rand.randint(0, LENGTH -1)

    return (position, velocity)

def car2():
    """
    generates a random velocity and a random position on the road if the
    function is called. This basically defines a car
    """
    velocity = rand.randint(0, VMAX)
    position = rand.randint(0, ROUND_ABOUT_LEN -1)

    return (position, velocity)
def road(size, length):
    """
    Creates a 2D numpy array where each row represents a road. It uses the
    car() function to generate random cars and places them on each road in turn
    """

    if size == 0:
        road = np.full((length), -1, dtype=int)
        cars = CARS
    elif size == 1:
        road = np.full((length), -1, dtype=int)
        cars = CARS_FIXED
    else:
        road = np.full((size, length), -1, dtype=int)
        cars = CARS_TOTAL
    for i in range(cars):

        while True:
            if size == 2:
                position, velocity = car2()

                if road[0, position] > -1:
                    continue
                else:
                    road[0, position] = velocity
                    road[1, position] = rand.randint(0, 1)
                    break
            else:
                position, velocity = car()

                if road[position] > -1:
                    continue
                else:
                    road[position] = velocity
                    break

    return road





def simulation(current_road):

    #print(current_road[1][INTERSECTION - 10:INTERSECTION + 5])

    #next_road = np.full((2, LENGTH), -1, dtype=int)
    next_road = np.array([np.full((LENGTH), -1, dtype=int), np.full((LENGTH), -1, dtype=int),
                          np.full((2, ROUND_ABOUT_LEN), -1, dtype=int)], dtype=object)
    cars_passing = 0

    #print(current_road[2])

    for i in range(len(current_road)):

        if i < 2:
            all_car_positions = np.where(current_road[i] > -1)[0]
            #print(all_car_positions)

        else:
            all_car_positions = np.where(current_road[i][0] > -1)[0]
            #print(all_car_positions)

        #print((all_car_positions))
        #print(*current_road[0])
        #print("")
        for j in range(len(all_car_positions)):


            current_car_position = all_car_positions[j]
            #print(current_car_position)
            if i < 2:
                #print((all_car_positions))
                if j == len(all_car_positions) - 1:
                    next_car_position = all_car_positions[0] + LENGTH
                    #print(next_car_position)
                else:
                    next_car_position = all_car_positions[j + 1]
                    #print(next_car_position)
            else:
                if j == len(all_car_positions) - 1:
                    next_car_position = all_car_positions[0] + ROUND_ABOUT_LEN
                else:
                    next_car_position = all_car_positions[j + 1]

            if i < 2:
                current_car_velocity = current_road[i][current_car_position]



                if current_car_position <= INTERSECTION:
                    if current_car_position + current_car_velocity >= INTERSECTION:

                        test = intersection_check_outside_new(i, current_car_position,
                                                                   current_car_velocity,
                                                                   next_road, current_road,
                                                                   all_car_positions, j)
                        if type(test) == bool:
                            #print(next_car_position)
                            next_road[i], cars_passing = standard_road_rules(next_car_position,
                                                               current_car_position,
                                                               current_car_velocity, next_road[i],
                                                               i, cars_passing)
                        else:
                            next_road = test

                    else:
                        if next_car_position > INTERSECTION:
                            next_car_position = INTERSECTION + 1
                            #print(next_car_position)
                        #print(next_car_position)
                        next_road[i], cars_passing = standard_road_rules(next_car_position,
                                                           current_car_position,
                                                           current_car_velocity, next_road[i],
                                                           i, cars_passing)


                else:


                    next_road[i], cars_passing = standard_road_rules(next_car_position,
                                                       current_car_position,
                                                       current_car_velocity, next_road[i],
                                                       i, cars_passing)




            else:
                #next_road_part = next_road[i][0]
                current_car_velocity = current_road[i][0, current_car_position]
                #index_exit = OVERLAPS[i][current_road[2][1, current_car_position]]
                if current_car_position <= OVERLAPS[i][current_road[2][1, current_car_position]] - 1:

                    if current_car_position + current_car_velocity >= OVERLAPS[i][current_road[2][1, current_car_position]] - 1:
                        test = intersection_check_inside_new(i, current_car_position,
                                                                  current_car_velocity,
                                                                  next_road, current_road,
                                                                  all_car_positions, j)
                        if type(test) == bool:
                            next_road[i] = standard_roundabout_rules(next_car_position,
                                                                     current_car_position,
                                                                     current_car_velocity,
                                                                     next_road[i], i, current_road[i])
                        else:
                            next_road = test
                    else:
                        #print(i)
                        #print(next_road[i][1, 5])
                        next_road[i] = standard_roundabout_rules(next_car_position,
                                                                 current_car_position,
                                                                 current_car_velocity,
                                                                 next_road[i], i, current_road[i])


                else:


                    next_road[i] = standard_roundabout_rules(next_car_position,
                                                             current_car_position,
                                                             current_car_velocity,
                                                             next_road[i], i, current_road[i])


    return(next_road, cars_passing)






CARS_TOTAL = 0
CARS = -1
CARS_FIXED = -1
for m in range(100):
    #CARS_TOTAL += 2
    CARS += 1
    CARS_FIXED += 1
    if m >= 100 - ROUND_ABOUT_LEN:
        CARS_TOTAL += 1
    #while CARS > 100:
    #CARS += 2
    #CARS_FIXED += 1
    #CARS_GIVE_WAY += 1
    cars_passing_m = 0
    for k in range(3):
        if k == 0:
            road1 = road(k, LENGTH)
        elif k == 1:
            road2 = road(k, LENGTH)
        else:
            road3 = road(k, ROUND_ABOUT_LEN)

    current_road = np.array([road1, road2, road3], dtype=object)
    #current_road = road()
    #print(*current_road)
    #CARS += 10
    for n in range(int(ITERATIONS)):
        current_road, count = simulation(current_road)
        #print(*current_road[0])
        #print("-------")
        cars_passing_m += count
        #print(cars_passing_m)
        #print(*current_road[0])
    flow_rate.append(cars_passing_m/ITERATIONS)
    car_density.append((CARS + CARS_FIXED + CARS_TOTAL)/(2*LENGTH + ROUND_ABOUT_LEN))

fig = plt.figure()

axis = fig.add_subplot(111)
axis.set_title("roundabout flow rate diagram")
axis.set_xlabel("car density")
axis.set_ylabel("flow rate")
axis.grid()
axis.scatter(car_density, flow_rate, color='red', marker='x')
plt.savefig("intersection_flowtrate_plot_new_30_Roundabout_1_forreport", dpi=600)
plt.show()
