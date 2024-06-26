import random
import json
import math
import sys
import time
import threading
import concurrent.futures
import gc
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from munkres import Munkres, print_matrix, make_cost_matrix,  DISALLOWED
import sys
from getDegradation import *

##create robot, chargin station and task object
class Robot:
    def __init__(self, original_index, k):
        self.original_index = original_index
        self.arrival_period = k
        self.battery_status = "new"
        self.cur_loc = (random.randint(0, total_area), random.randint(0, total_area))
        self.state_of_charge = (round(random.uniform(Ebat, Ebat), 4)) ##to have values upto two decimal points
        self.initial_deg = 0
        self.status = "idle"
        self.task_end_time = None
        self.allocated_task = None
        self.charge_start_time = None
        self.allocated_CS = None
        self. CS_reaching_time = None
        self.task_slope = None
        self.charge_counter = 0
        

    def __str__(self):
        return f"Robot {self.original_index}: cur_loc={self.cur_loc}, task_end_time={self.task_end_time}, charge_start_time={self.charge_start_time}, CS_reaching_time={self.CS_reaching_time}, allocated_CS ={self.allocated_CS}, allocated_task ={self.allocated_task}, state_of_charge={self.state_of_charge}, status={self.status}, bat_status ={self.battery_status}, charge_counter ={self.charge_counter}"

class ChargingStation:
    def __init__(self, CS_location, original_index):
        self.original_index = original_index
        self.CS_location = CS_location
        self.CS_status = "free"
        self.charge_start_time = None
        self.robot_charging = None
        self.robot_waiting_to_charge = -1
        self.cs_slope = round(random.uniform(1, 4) , 2)

    def __str__(self):
        return f"ChargingStation {self.original_index}: CS_location={self.CS_location}, CS_status={self.CS_status}, charge_start_time = {self.charge_start_time}, robot_charging = {self.robot_charging}, robot_waiting_to_charge = {self.robot_waiting_to_charge}"

class Task:
    def __init__(self, k, original_index):
        random.seed(a=None, version=2)
        self.original_index = original_index
        self.start_loc = (random.randint(0, total_area), random.randint(0, total_area))
        self.end_loc = (random.randint(0, total_area), random.randint(0, total_area))
        while self.start_loc == self.end_loc:
            self.end_loc = (random.randint(0, total_area), random.randint(0, total_area))
        self.value = random.randint(task_min, task_max)
        self.arrival_time = k
        self.deadline = self.arrival_time + task_deadline
        self.total_distance_to_cover = manhattan_distance(self.end_loc, self.start_loc)
        self.status = "active"
        self.t_slope = round(random.uniform(-3, 3) , 2)

    def __str__(self):
        return f"Task {self.original_index}: start_loc={self.start_loc}, end_loc={self.end_loc}, value={self.value}, arrival_time={self.arrival_time},  deadline={self.deadline}, total_distance_to_cover={self.total_distance_to_cover}, status={self.status}"

#####Functions#######################
#for getting v_soc value for any energy level value and and any selected threshold
def piecewise_linear(input_value, threshold, max_soc):
    # If the input_value is less than the threshold
    if input_value < threshold:
        # Calculate the result using linear interpolation in the range [-1, -0.05]
        result = -1 + (0.95/threshold) * input_value
    # If the input_value is equal to the threshold
    elif input_value == threshold:
        result = 1
    # If the input_value is greater than the threshold
    else:
        # Calculate the result using inverse linear interpolation in the range [1, 0.05]
        result = 1 - (0.95/(max_soc - threshold)) * (input_value - threshold)
    return result

####use this function to get the scaled value in the 0-1 range based on the range of the input value
def scale_value(value, min_value, max_value):
    return (value - min_value) / (max_value - min_value)

####function to calculate manhattan distance between two points
def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

###function to calculate average_task_completion time by all the robots in avail_robots list
def calc_avg_task_completion_time(robot, tasks_list, r_s):
    completion_times = []
    for task in tasks_list:
        completion_time = round((manhattan_distance(robot.cur_loc, task.start_loc) + task.total_distance_to_cover) / r_s)
        if completion_time == 0:
            completion_time = 1
        completion_times.append(completion_time)
    average_time = sum(completion_times) / len(completion_times)
    average_time = round(average_time)
    if average_time == 0:
        average_time = 1
    return average_time

## function to get SoC trace for a particular robot till a particular time slot
def get_charge_trace(robot, k):
    bat = robot.battery_status
    if bat == "new":
        if k < 2:
            trace = state_of_charge_traces[robot.original_index][:k+1]
            value= state_of_charge_traces[robot.original_index][0] ###get the initial soc 
            replication_times = 2-k  ##as the rainflow needs atleast 3 input soc to work
            dummy_trace = []
            dummy_trace.extend([value for i in range(replication_times)])
            new_trace = [*dummy_trace, *trace]
        else:
            new_trace = state_of_charge_traces[robot.original_index][:k+1]
    else:
        old_trace = charge_history_trace[robot.original_index]
        curr_trace = state_of_charge_traces[robot.original_index]
        new_trace = [*old_trace, *curr_trace]  
    return new_trace

#####function to get future soc trace for task
def create_future_SoC_trace_task(prev_trace, required_periods, energy_per_period_task):
    last_index = (len(prev_trace) -1)
    last_energy  = prev_trace[last_index] ##to get the last energy value from the previous trace
    new_list = []
    if required_periods > 0:
        for i in range(0, required_periods):
            new_energy = max(0, last_energy - energy_per_period_task)
            new_list.append(new_energy)
            last_energy = new_energy
        combined_trace =  [*prev_trace, *new_list] #combining the previous trace and 
    else:
        combined_trace = prev_trace        
    return combined_trace

#####function to get future soc trace for charge/idle
def create_future_SoC_trace_charge_idle(prev_trace, energy_per_period, time_2_reach_cs, charging_or_idle_dur, energy_per_period_while_idle, op = "charge"):        
    if op == "charge":
        last_index = (len(prev_trace) -1)
        last_energy  = prev_trace[last_index] ##to get the last energy value from the previous trace
        new_list1 =[]
        if time_2_reach_cs > 0:
            for i in range(0, time_2_reach_cs):
                new_energy = max(0, (last_energy - energy_per_period)) 
                new_list1.append(new_energy)
                last_energy = new_energy
            last_index1 = (len(new_list1) -1)
            last_energy1  = new_list1[last_index1] ##to get the last energy value from the previous trace
        else:
            last_energy1 = prev_trace[last_index]
        new_list2 =[]
        if charging_or_idle_dur > 0:
            for i in range(0, charging_or_idle_dur):
                new_energy2 = min(highest_soc, (last_energy1 + Charging_rate)) 
                new_list2.append(new_energy2)
                last_energy1 = new_energy2
        combined_trace =  [*prev_trace, *new_list1, *new_list2]
    else:
        last_index = (len(prev_trace) -1)
        last_energy  = prev_trace[last_index] ##to get the last energy value from the previous trace
        new_list = []
        if charging_or_idle_dur > 0:
            for i in range(0, charging_or_idle_dur):
                new_energy = last_energy - energy_per_period_while_idle 
                new_list.append(new_energy)
                last_energy = new_energy
        combined_trace =  [*prev_trace, *new_list] #combining the previous trace and the new estimated trace
    return combined_trace

def degradation_calc(combined_soc_trace, initial_deg_value):
    soc_percentage = 100  ## as we have to divide the energy values to get percentage to feed in rainflow
    soc_percentage = np.array(soc_percentage)
    combined_trace = combined_soc_trace/soc_percentage
    df = pd.DataFrame()
    df['SOC'] = combined_trace
    degradation_value = rainflow_code(df)
    degradation_value = float(degradation_value)###this is bcz the deg value returned in the prev line returns a string
    final_deg = initial_deg_value + degradation_value
    return final_deg
    
###munkres function 
def munkres_func(matrix):
    print("-----in munkres function")
    cost_matrix = make_cost_matrix(matrix, lambda cost: (- cost) if
                                      (cost != DISALLOWED) else DISALLOWED)
    m = Munkres()
    indexes = m.compute(cost_matrix)
    selected_robot = [i for i, j in indexes]
    selected_col = [lis[-1] for lis in indexes]
    res = {selected_robot[i]: selected_col[i] for i in range(len(selected_robot))}
    print("----end of munkres results is", res)
    # print_matrix(matrix, msg='Highest profit through this matrix:')
    total = 0
    for row, column in indexes:
        value = matrix[row][column]
        total += value
        # print(f'({row}, {column}) -> {value}')
    print(f'----total profit={total}')
    return res, total

###function to update parameters######
def update_parameters(k):
    for robot in all_robots:
        if robot.status == "idle":
            robot.state_of_charge -= energy_consumption_while_idle
            robot.state_of_charge = max(0, robot.state_of_charge)
            state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
        elif robot.status == "busy":
            if k <= robot.task_end_time:
                energy_consumption = calc_energy_consumption(robot.task_slope, incline = 1)
                robot.state_of_charge -= energy_consumption
                robot.state_of_charge = max(0, robot.state_of_charge)
                state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                if k == robot.task_end_time:
                    # robot.state_of_charge = robot.state_of_charge
                    # state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                    robot.status = "idle"
                    task_end_loc = all_tasks[robot.allocated_task].end_loc
                    robot.cur_loc = task_end_loc
                    robot.allocated_task = None
                    robot.task_end_time = None
        elif robot.status == "going to recharge":
            if k <= robot.charge_start_time:
                robot.state_of_charge -= fixed_energy_consumption
                robot.state_of_charge = max(0, robot.state_of_charge)
                state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                if k == robot.charge_start_time:
                    robot.status = "charging"
                    charge_loc = all_CS[robot.allocated_CS].CS_location
                    robot.cur_loc = charge_loc
                    robot.allocated_task = None
                    #robot.charge_start_time = None
        elif robot.status == "charging":  ###this is for if robot is charging
                if robot.state_of_charge <= highest_soc:
                    robot.state_of_charge += Charging_rate
                    robot.state_of_charge = min(highest_soc, robot.state_of_charge)
                    state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                    if robot.state_of_charge == highest_soc:
                        # robot.state_of_charge = robot.state_of_charge
                        # state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                        robot.status = "idle"
                        robot.charge_start_time = None
                        all_CS[robot.allocated_CS].CS_status = "free"
                        all_CS[robot.allocated_CS].robot_charging = None
        elif robot.status == "needs_charge":
            if k <= robot.CS_reaching_time: 
                robot.state_of_charge -= fixed_energy_consumption
                robot.state_of_charge = max(0, robot.state_of_charge)
                state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                if k == robot.CS_reaching_time:
                    # robot.state_of_charge = robot.state_of_charge
                    # state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                    robot.status = "idle"
                    # robot.CS_reaching_time = None
    return all_robots, all_CS, all_tasks, state_of_charge_traces              

####function to update avail_lists
def update_avail_lists(k):
    avail_tasks = []
    if all_tasks:
        for task in all_tasks:
            if task.deadline < k and task.status != "allocated":
                task.status = "timed_out"
        for task in all_tasks:
            if task.status == "active":
                avail_tasks.append(task)
    ###create avail_robots list
    avail_robots = []
    for robot in all_robots:
        if robot.status == "idle":
            avail_robots.append(robot)        
    ###create avail_cs list
    avail_CS =[]
    for cs in all_CS:
        if cs.CS_status == "free":
            avail_CS.append(cs) 
    return avail_robots, avail_CS, avail_tasks

###code for threading
# Define a function that represents a worker thread
def calculate_matrix_row(matrix, avail_robots, avail_tasks, avail_CS, r_s, old_deg_min, old_deg_max, task_min, task_max, q1, q2, q3, start_row, end_row):
    for i in range(start_row, end_row):
        robot = avail_robots[i]
        for j in range(len(matrix[0])):
            if j < len(avail_tasks):  ###to calc utility for R to T
                ###first check if robot will have enough energy to reach to the nearest CS
                prev_trace = get_charge_trace(robot, k)
                total_dis = (manhattan_distance(robot.cur_loc, avail_tasks[j].start_loc)) + avail_tasks[j].total_distance_to_cover
                slope = avail_tasks[j].t_slope
                total_time_required = round (total_dis/r_s)
                if total_time_required == 0:
                    total_time_required = 1
                energy_consumption = calc_energy_consumption(slope, incline = 1)        
                total_e_to_complete_task = total_time_required * energy_consumption
                if robot.state_of_charge < total_e_to_complete_task:
                    matrix[i][j] = DISALLOWED
                    # print("not enough energy to execute task!!!")    
                else:
                    # min_distance = float('inf')  # Initialize with a large value, its the distance to reach the closet CS
                    max_distance = float('-inf')
                    for cs in all_CS:
                        distance = manhattan_distance(avail_tasks[j].end_loc, cs.CS_location)
                        if distance > max_distance:
                            max_distance = distance
                    time_2_reach_CS = round(max_distance/r_s)
                    if time_2_reach_CS == 0:
                        time_2_reach_CS = 1
                    least_required_e_to_reach_cs = time_2_reach_CS * fixed_energy_consumption   ##calc the amount of e to reach to the fathest CS
                    total_e_required = total_e_to_complete_task + least_required_e_to_reach_cs                    
                    # future_soc_trace = create_future_SoC_trace_task(prev_trace, total_time_required, energy_consumption)
                    # energy_after_completing_task = future_soc_trace[-1]
                    if total_e_required > robot.state_of_charge:
                        matrix[i][j] = DISALLOWED
                        print("energy is not enough to reach nearest cs!!!!")
                    else:
                        future_soc_trace = create_future_SoC_trace_task(prev_trace, total_time_required, energy_consumption)        
                        deg = degradation_calc(future_soc_trace, robot.initial_deg)
                        # print(f"--deg for rob {robot.original_index} to task {avail_tasks[j].original_index}: {deg}")
                        # q1 = min_max_scaling(deg, old_deg_min, old_deg_max, new_deg_min, new_deg_max)
                        scaled_deg = scale_value(deg, old_deg_min, old_deg_max)
                        # print(f"--deg after scaling: {q1*deg}")
                        # print(f"--deg after scaling: {scaled_deg}")
                        value_of_task = avail_tasks[j].value
                        # print(f"--value of task: {value_of_task}")
                        scaled_task_val  = scale_value(value_of_task, task_min, task_max)
                        # print(f"value of task after scaling: {scaled_task_val}")
                        #matrix[i][j] = (avail_tasks[j].value) - (q1 * deg)
                        matrix[i][j] = scaled_task_val - (q1 * scaled_deg)
                        # matrix[i][j] = round(matrix[i][j], 4)
                        print("matrix for task :", matrix[i][j])    
            elif j < len(avail_tasks) + len(avail_CS):  ###to calc utility for R to CS
                if robot.state_of_charge >= soc_threshold:
                    print("!!!robot energy is higher so do not need charge so disallowed")
                    matrix[i][j] = DISALLOWED        
                else:
                    dis_to_CS = manhattan_distance(robot.cur_loc, avail_CS[j - len(avail_tasks)].CS_location)
                    time_to_reach_CS = round(dis_to_CS/r_s)
                    if time_to_reach_CS == 0:
                        time_to_reach_CS = 1
                    curr_e = robot.state_of_charge
                    min_e_to_reach_CS = time_to_reach_CS * fixed_energy_consumption
                    if dis_to_CS == 0 and curr_e < fixed_energy_consumption:
                        min_e_to_reach_CS = curr_e    
                    if curr_e >= min_e_to_reach_CS:
                        if len(avail_tasks) == 0:
                            energy_to_recharge = highest_soc - (curr_e - (min_e_to_reach_CS))
                            recharge_dur = round(energy_to_recharge/Charging_rate)
                            if recharge_dur == 0:
                                recharge_dur = 1
                        else:
                            recharge_dur = calc_avg_task_completion_time(robot, avail_tasks, r_s)
                        prev_trace = get_charge_trace(robot, k)
                        future_soc_trace = create_future_SoC_trace_charge_idle(prev_trace, fixed_energy_consumption, time_to_reach_CS, recharge_dur, energy_consumption_while_idle, op = "charge")
                        deg = degradation_calc(future_soc_trace, robot.initial_deg)
                        # print(f"--deg for rob {robot.original_index} to CS {avail_CS[j - len(avail_tasks)].original_index}: {deg}")
                        # q2 = min_max_scaling(deg, old_deg_min, old_deg_max, new_deg_min, new_deg_max)
                        scaled_deg = scale_value(deg, old_deg_min, old_deg_max)
                        ###new addition
                        scaled_deg = 1 - scaled_deg
                        # if len(avail_tasks) == 0:
                        #     scaled_deg = 1 - scaled_deg
                        # print(f"--deg after scaling: {scaled_deg}")
                        robot_soc = robot.state_of_charge
                        v_soc = piecewise_linear(robot_soc, soc_threshold, Ebat)
                        # print(f"-- v_soc for charge: {v_soc}")
                        ##matrix[i][j] = avail_CS[j - len(avail_tasks)].CS_location
                        # matrix[i][j] = - (q2 *deg * v_soc)
                        matrix[i][j] = - (q2 * scaled_deg * v_soc)
                        # matrix[i][j] = round(matrix[i][j], 4)
                        print(f"--utility for charge: {matrix[i][j]}")    
                    else:
                        print("!!!! not eneough energy to go to this CS, so disallowed")
                        matrix[i][j] = DISALLOWED        
            else:     #######to calc utility for R to idle
                if i == j - len(avail_tasks) - len(avail_CS):
                    if robot.state_of_charge >= soc_threshold:
                        matrix[i][j] = -1000  ###very small negative value -1e-15
                        print("matrix for idle:", matrix[i][j]) 
                    else:
                        matrix[i][j] = -1000      
                else:
                    matrix[i][j] = DISALLOWED

# Function to allocate threads and perform parallel computation for the matrix
def allocate_threads(matrix, avail_robots, avail_tasks, avail_CS, r_s, old_deg_min, old_deg_max, task_min, task_max, q1, q2, q3, num_threads):
    rows_per_thread = len(matrix) // num_threads
    with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
        future_to_row = {
            executor.submit(
                calculate_matrix_row,
                matrix,
                avail_robots,
                avail_tasks,
                avail_CS,
                r_s,
                # energy_consumption,
                old_deg_min,
                old_deg_max,
                task_min,
                task_max,
                q1,
                q2,
                q3,
                i * rows_per_thread,
                (i + 1) * rows_per_thread if i < num_threads - 1 else len(matrix),
            ): i
            for i in range(num_threads)
        }

        for future in concurrent.futures.as_completed(future_to_row):
            try:
                future.result()
            except Exception as e:
                print(f"Exception occurred: {e}")
    return matrix

#####function to do allocation###
def allocation(k, all_robots, all_CS, all_tasks, avail_robots, avail_CS, avail_tasks, state_of_charge_traces):
    # Create matrix and initialize variables
    matrix = [[0 for _ in range(len(avail_tasks) + len(avail_CS) + len(avail_robots))] for _ in range(len(avail_robots))]
    # Call the function to allocate threads using ThreadPoolExecutor
    matrix = allocate_threads(matrix, avail_robots, avail_tasks, avail_CS, r_s, old_deg_min, old_deg_max, task_min, task_max, q1, q2, q3, num_threads)
    ###further tasks after calculating the matrix are written below            
    ##printing the matrix
    print("-------matrix:")
    for row in matrix:
        print(row) 
    decision, total = munkres_func(matrix) ##run the matrix through munkres
    
    for robot_index in decision:
        robot = avail_robots[robot_index]
        col_index = decision[robot_index]
        # If matched with a task
        if col_index < len(avail_tasks):
            task = avail_tasks[col_index]
            total_dis = (manhattan_distance(robot.cur_loc, avail_tasks[col_index].start_loc)) + avail_tasks[col_index].total_distance_to_cover
            total_time_required = round(total_dis/r_s)
            if total_time_required == 0:
                total_time_required = 1
            all_robots[robot.original_index].status = "busy"
            all_robots[robot.original_index].allocated_task = task.original_index
            all_robots[robot.original_index].task_slope = task.t_slope
            all_robots[robot.original_index].task_end_time = k + total_time_required
            all_tasks[task.original_index].status = "allocated"
            if all_robots[robot.original_index].state_of_charge > lowest_soc:
                all_robots[robot.original_index].charge_counter = 0
            # avail_robots.remove(robot)
            print(f"----Robot {robot.original_index} matched with Task {task.original_index}")
        # If matched with a charging station
        elif col_index < len(avail_tasks) + len(avail_CS):
            cs_index = col_index - len(avail_tasks)
            cs = avail_CS[cs_index]
            cs_counter = all_robots[robot.original_index].charge_counter
            all_robots[robot.original_index].charge_counter = cs_counter + 1
            if (all_robots[robot.original_index].charge_counter < max_cs_counter) and (all_robots[robot.original_index].state_of_charge > 5):
                all_robots[robot.original_index].status = "idle"
                all_robots[robot.original_index].idle_time = k+1
                print(f"----Robot {robot.original_index} remains idle")
            else:    
                dis_to_CS = manhattan_distance(robot.cur_loc, cs.CS_location)
                all_robots[robot.original_index].charge_counter = 0
                if dis_to_CS == 0:  ###this if will check if robot already is in the CS location
                    all_robots[robot.original_index].status = "charging"
                    all_robots[robot.original_index].allocated_CS = cs.original_index
                    all_CS[cs.original_index].CS_status = "occupied"
                    all_CS[cs.original_index].robot_charging = robot.original_index
                else:
                    time_to_reach_CS = round(dis_to_CS/r_s)
                    if time_to_reach_CS == 0:
                        time_to_reach_CS = 1
                    all_robots[robot.original_index].status = "going to recharge"
                    all_robots[robot.original_index].allocated_CS = cs.original_index
                    all_CS[cs.original_index].CS_status = "occupied"
                    all_CS[cs.original_index].robot_charging = robot.original_index
                    all_robots[robot.original_index].charge_start_time = k + time_to_reach_CS
                print(f"-----Robot {robot.original_index} matched with Charging Station {cs.original_index}")

        # If matched with another robot
        else:
            all_robots[robot.original_index].status = "idle"
            all_robots[robot.original_index].idle_time = k+1
            print(f"----Robot {robot.original_index} remains idle")
            ###new block of code to implement robots queued on CS
            if all_robots[robot.original_index].status == "idle" and all_robots[robot.original_index].state_of_charge < soc_threshold:
                print("---robot actually needs charge but no CS is available!!")
                all_robots[robot.original_index].status = "needs_charge"
                ###see if any CS is free then allocate to it
                ####code to find which CS will be free soon###
                highest_charge = -1
                preferred_CS_index = None ##this is the cs that will be free soon
                for cs in all_CS:
                    if cs.robot_charging is not None:
                        charging_robot = all_robots[cs.robot_charging]
                        if charging_robot.state_of_charge > highest_charge:
                            highest_charge = charging_robot.state_of_charge
                            print(f"highest_charge in robot charging in CS: {cs}")
                            preferred_CS_index = cs.original_index
                    else:
                        print("no robot is charging")        
                if preferred_CS_index is not None:
                    preferred_CS = all_CS[preferred_CS_index]
                    dis_to_reach = manhattan_distance(robot.cur_loc, preferred_CS.CS_location)
                    # print("preferred CS's distance is", dis_to_reach)
                    time_to_reach = round(dis_to_reach/r_s)
                    if time_to_reach == 0:
                        time_to_reach = 1
                    all_robots[robot.original_index].CS_reaching_time  = k + time_to_reach
                    # print(f"--!!--cs reaching time of that robot {all_robots[robot.original_index]} is {all_robots[robot.original_index].CS_reaching_time}")
                    all_robots[robot.original_index].cur_loc  = preferred_CS.CS_location
                    # print(f"robot's new loc is {all_robots[robot.original_index].cur_loc}")
                    
                    
    return all_robots, all_CS, all_tasks, total


####function to print avail_lists
def print_lists(k, robot_list, task_list, cs_list):
    print(f"---Robots in for k = {k}:")
    # for robot in robot_list:
    #     print(f"robot: {robot}")
    print(f"----Tasks in for k = {k}:")
    count_active_tasks = 0
    for task in task_list:
        if task.status == "active":
            count_active_tasks += 1
    print("----no of active tasks", count_active_tasks)
    # for task in task_list:
    #     print(f"task: {task}")   
    print(f"----CS in for k = {k}:")
    # for cs in cs_list:
    #     print(f"cs {cs}")         
        
# Define a function to calculate bat_deg for a robot
def calculate_bat_deg(robot):
    bat = robot.battery_status
    if bat == "new":
        soc_trace = state_of_charge_traces[robot.original_index]
    else:
        old_trace =  charge_history_trace[robot.original_index] 
        curr_trace = state_of_charge_traces[robot.original_index]
        soc_trace = [*old_trace, *curr_trace]
         
    calculated_deg = degradation_calc(soc_trace, robot.initial_deg)
    bat_deg_traces[robot.original_index].append(calculated_deg)    


##function to calculate energy consumption
def calc_energy_consumption(slope, incline = 1):
    thetaa = math.radians (slope)
    energy_for_inclination = rob_weight * gravity_acceleration * rob_speed
    if incline == 0:
        energy_for_inclination
    energy_consump = (power_consumption + energy_for_inclination  * max(math.sin(thetaa), 0)) * period_length_sec/3600
    return energy_consump 
  
###-----read parameter from json###########
# Helper function to load from JSON
def load_all_from_json(filename):
    with open(filename, 'r') as file:
        return json.load(file)
# Load all data
all_data = load_all_from_json('exp1_data.json')

# Load parameters and extract T
params = all_data['parameters']
# T = len(all_data['tasks_per_k'])  # Assuming T is the length of tasks_per_k
task_min = params.get('task_min', None)  # Default to None if not found
task_max = params.get('task_max', None)  # Default to None if not found
no_of_old_robs = params.get('no_of_old_robs', None)
T = params.get('T', None)
task_deadline = params.get('task_deadline', None)
max_cs_counter = params.get('max_cs_counter', None)
max_no_of_days = params.get('max_no_of_days', None)
periods_per_day = params.get('periods_per_day', None)
tot_order = params.get('tot_order', None)
max_order_per_k = params.get('max_order_per_k', None)
demand_to_meet = params.get('demand_to_meet', None)
R = params.get('R', None)
C = params.get('C', None)
cs_loc = params.get('cs_loc', None)
total_area = params.get('total_area', None)
Ebat = params.get('Ebat', None)
lowest_soc = params.get('lowest_soc', None)
# T = params.get('T', None)
# Populate lists with data
all_robots = [Robot(data['original_index'], data['arrival_period']) for data in all_data['robots']]
all_CS = [ChargingStation(tuple(data['CS_location']), data['original_index']) for data in all_data['charging_stations']]

#######Parameters initialization
num_threads = 16
###define days parameter
no_of_days = -1
day_start = 0   ##start period of any new day
day_end = day_start + periods_per_day - 1  ##end period of any new day
days_per_month = 1 ###it will be 30 if we consider 30 days
###no of robot needed calculation
# demand_to_meet = 1 ###read how much percentage of the total demand we have to meet to calc how many no of robots we need, 1 means 100%
### Define constants and variables

q1 = params.get('q1', None)##multiplicative weight with bat deg when calculating robot to task utility
q2 = params.get('q2', None)
q3 = 1
# cs_loc =5    #total no of CS locations
# total_area = 2000 ###meter square assuming each small cell is 1m sq. in area
# task_min = 5
# task_max = 50  ##range of the task profits
# old_deg_min = 0.0000098
# old_deg_min = 0.00001
# old_deg_max = 0.1   ##range of the battery degradation value
old_deg_min = params.get('old_deg_min', None)
old_deg_max = params.get('old_deg_max', None)
new_deg_min = task_min
new_deg_max =  task_max
period_length_sec = params.get('period_length_sec', None)
# period_length_sec = 60 ## period length in second (600 = 10 min, 300 = 5 min, 60 = 1min)
# soc_threshold = 50 ##this is for v_soc calculation
soc_threshold = params.get('soc_threshold', None) ##this is for v_soc calculation
cpu_frq = 2.26 ## cpu frequency in GHz
r_s = 1.6 * period_length_sec ##in meter/sec or metter/minute according to period puration. the multiplication with period_length is done to convert it according to the decision period
# robot_speed = 2.6 *period_length_sec ##using this speed to have less time to execute tasks
rob_speed = 1.6 ###m/s to use in the energy consumption calc
rob_weight = 20  ##in kg
gravity_acceleration = 9.8 ##in m/s2
power_consumption = 38.213568  ###reference value of power consumption in W for certain cpu freq and robot speed
fixed_energy_consumption = round((power_consumption * period_length_sec/3600),8) ##energy consumption in WH, we will use it when not considering inclinations
print("---energy comsumption for 1 min,", fixed_energy_consumption)
# fixed_energy_consumption = 5  ###just to test with simple values
# energy_consumption_while_idle = round((8.514 * period_length_sec/3600),8)  ###just to have amount of energy drawn while idle, measured in freq of 0.883GHz
energy_consumption_while_idle = params.get('energy_consumption_while_idle', None)
highest_soc = params.get('highest_soc', None)# Battery capacity (Wh) for smaller battery
# lowest_soc = params.get('lowest_soc', None)# Battery capacity (Wh) for smaller battery
Charging_time = 0.75 # hrs to completely recharge the battery
Charging_rate = Ebat/(Charging_time*3600/period_length_sec) #Wh recharged during an entire period in charging mode
# Charging_rate = 3   ## for testing with small simple value

#######old robot soc history extraction
# old_soc_file = 'soc_trace_6.2%_deg.csv'  ###to read previous soc trace of older robots
old_soc_file = params.get('old_soc_file', None)
# no_of_old_robs = 0  ###read it from csv from old robs
charge_history_df = pd.read_csv(old_soc_file)
old_robs = 0
###to flag few robots as old
if no_of_old_robs > 0:
    for robot in all_robots:    
        robot.battery_status = "old"
        no_of_old_robs -= 1
        old_robs += 1
        if no_of_old_robs == 0:
            break

charge_history_trace = {} ##to store the past history of soc of old robots
### to save the charge history of the old robots
if old_robs > 0:
    for robot in all_robots:
        if robot.battery_status == "old":
            column_name = f'soc_Rob{robot.original_index}'
            charge_history_trace[robot.original_index] = charge_history_df[column_name].dropna().tolist()
            old_robs -= 1
            if old_robs <= 0:
                break
# Dictionary to store state_of_charge values for each robot over time
state_of_charge_traces = {robot.original_index: [robot.state_of_charge] for robot in all_robots}
# Initialize bat_deg_traces with the initial_deg of each robot
bat_deg_traces = {robot.original_index: [robot.initial_deg] for robot in all_robots}
all_tasks = []
per_round_allocation_time = []
total_utility = []

'''-----------main operation of the algorithm-------------'''
k=0
####time tracking###
start_time  = time.time()
while k <= T:
    print("---value of k =", k)
    gc.disable()
    if k==0:
        pass
    if k>0:
    ###call the function to update robot energies, status and locations
        all_robots, all_CS, all_tasks, state_of_charge_traces = update_parameters(k) 
        # print(f"#####print all lists after parameter updation at the beginning of k={k}")        
        ####---new addition for per day result storage---#####
        if k == day_end:
            ##updating the start and end day period
            no_of_days +=1
            print(f"--starting new day: {no_of_days}--")
            day_start = k  
            day_end = day_start + periods_per_day
            ###calc the bat deg with concurrent threading
            with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
                # Submit tasks to the ThreadPoolExecutor for concurrent execution
                futures = [executor.submit(calculate_bat_deg, robot) for robot in all_robots]
                # Wait for all threads to complete
                concurrent.futures.wait(futures)
            # print(f"---!!!!bat deg values at period: {k} and after day = {no_of_days} is: {bat_deg_traces}")    
        # print_lists(k, all_robots, all_tasks, all_CS)
        if k==T:   ###this is to update the soc for the last allocation at T-1
            print("----end of working period----")
            break    
    ###update all_tasks by including the new tasks
    task_data_for_k = all_data['tasks_per_k'].get(str(k), [])  # Using str(k) because JSON keys are always strings
    for data in task_data_for_k:
        task = Task(k, data['original_index'])
        task.__dict__.update(data)
        all_tasks.append(task)
    # print(f"#####print all lists after parameter updation at the beginning of k={k}")
    ###create new tasks and update the avail_tasks list                                  
    avail_robots, avail_CS, avail_tasks = update_avail_lists(k) 
    print(f"------print avail lists after making new avail_lists from all lists at the beginning of k={k}")
    # print_lists(k, avail_robots, avail_tasks, avail_CS)       
    ####call allocation####
    if len(avail_robots) != 0:
        alloc_start = time.time() 
        all_robots, all_CS, all_tasks, total = allocation(k, all_robots, all_CS, all_tasks, avail_robots, avail_CS, avail_tasks, state_of_charge_traces)
        if total:
            total_utility.append(total)
        print(f"----end of allocation for k={k}")
        alloc_end = time.time() 
        total_per_round_alloc = alloc_end - alloc_start
        per_round_allocation_time.append(total_per_round_alloc)        
    else:
        print("!!!no avail robots, so no allocatin was called, just updating k!!! ")
    # print_lists(k, all_robots, all_tasks, all_CS)
    del avail_tasks
    del avail_robots
    del avail_CS
    k=k+1
    

end_time  = time.time()
total_time  = end_time - start_time

##code for calculating the gained profit and no_of_tasks allocated
percentage_gained_valuation = []
allocated_task_percentage = []
day_s = 0
for day in range(max_no_of_days):
    total_value = 0
    gained_value = 0
    allocated_tasks = 0
    day_e = day_s + periods_per_day -1 
    daily_tasks = [task for task in all_tasks if day_s <= task.arrival_time <= day_e]
    for t in daily_tasks:
        total_value += t.value
        if t.status == "allocated":
            gained_value += t.value
            allocated_tasks += 1
    if len(daily_tasks) == 0:
        perc_allocated_tasks = 1
        perc_gained_value = 1
    else:    
        perc_allocated_tasks = allocated_tasks/(len(daily_tasks))        
        perc_gained_value = (gained_value/total_value) * 100   
    allocated_task_percentage.append(perc_allocated_tasks)    
    percentage_gained_valuation.append(perc_gained_value)
    day_s = day_e + 1

per_alloc_time  =  (max(per_round_allocation_time))/max_order_per_k       
print(" % gained value for BTC-M", percentage_gained_valuation) 
print(f"execution time per allocation for BTC-M: {per_alloc_time}") 
###make the energy value as SoC percentage
v = (100/Ebat)
soc_trace_percentage = {}
for robot_index, charges in state_of_charge_traces.items():
    soc_trace_percentage[robot_index] = [charge * v for charge in charges]

bat_deg_perc = {}
for robot_index, degrad in bat_deg_traces.items():
    bat_deg_perc[robot_index] = [(100*deg) for deg in degrad]
    
# per_period_execution_time = (total_time / T)
# print(f"----!!! per period execution time for BTC-M: {per_period_execution_time}")     

###to create dataframes
df1 = pd.DataFrame()    
df1['BTC_M_(%)gained_value'] = percentage_gained_valuation
df1['BTC_M_per_alloc_exec_time'] = per_alloc_time
# df1['BTC_M_per_peri_exec_time'] = per_period_execution_time

       
###write reults in a csv
bat_deg_data = {'BTC_M_deg_Rob{}'.format(robot.original_index): bat_deg_perc[robot.original_index] for robot in all_robots}
df2 = pd.DataFrame(bat_deg_data)

soc_data = {'BTC_M_soc_Rob{}'.format(robot.original_index): state_of_charge_traces[robot.original_index] for robot in all_robots} 
df3 = pd.DataFrame(soc_data)
utility_data = total_utility
df4 = pd.DataFrame(utility_data)


# Save DataFrame to a CSV file
experiment_name = "exp1"

File_name1 = f"results_BTC_M_{experiment_name}.csv"
File_name2 = f"bat_deg_BTC_M_{experiment_name}.csv"
File_name3 = f"soc_trace_BTC_M_{experiment_name}.csv"
# File_name4 = f"total_utility_BTC_M_{experiment_name}.csv"


df1.to_csv(File_name1, index=False)
df2.to_csv(File_name2, index=False)
df3.to_csv(File_name3, index=False)
# df4.to_csv(File_name4, index=False)



# Plotting the SoC trace when the no of days = 1
if max_no_of_days == 1:
    fig, ax = plt.subplots()
    plt.rcParams['font.size'] = '16'
    plt.rcParams["legend.fontsize"]= '14'
    plt.rcParams["axes.labelweight"] = "bold"
    plt.rcParams.update({'figure.autolayout': True})

    for robot_index, charges in soc_trace_percentage.items():
        plt.plot(range(0,T+1), charges, label=f'AMR {robot_index}')
    plt.axhline(y=(highest_soc / Ebat) * 100, color='r', linestyle='-', label="Emax")
    plt.xlim([0, T+1])
    plt.ylim([0, Ebat+1]) 
    current_ticks = np.arange(0, T+1, 80)  # Create a range of ticks up to the maximum value in 'time'
    new_ticks = current_ticks * 3
    ax.set_xticks(current_ticks)
    ax.set_xticklabels(new_ticks.astype(int))
    # plt.ylim([0, 119]) 
    # plt.yticks(fontsize = 16)
    for l in range(0, T+1, 2):
        plt.axvline(x=[l], color='grey', alpha=0.1)
    # plt.title('State of Charge over Time Units')
    plt.xlabel('Time (Min)', weight = 'bold')
    plt.ylabel('State of Charge (%)', weight = 'bold')
    plt.legend(ncol = 3, loc = 'upper right', bbox_to_anchor =(1.02, 1.04), frameon = False)
    # plt.legend()
    plt.savefig(f"state_of_charge_algo_{experiment_name}.pdf")
    plt.show()

#### plot bat deg for the robots
if max_no_of_days > 1:
    for robot_index, degradation in bat_deg_perc.items():
        plt.plot(range(0,no_of_days + 2), degradation, label=f'AGR {robot_index}')
    # plt.axhline(y=(highest_soc / Ebat) * 100, color='r', linestyle='-', label="Emax")
    plt.xlim([0, no_of_days + 1])
    # plt.ylim([0, 1.2]) 
    # plt.ylim([0, 119]) 
    plt.xticks(range(0,no_of_days+1, 1), fontsize = 16)#
    plt.yticks(fontsize = 16)
    # for l in range(0, T+1, 2):
    #     plt.axvline(x=[l], color='grey', alpha=0.1)
    # plt.title('Bat degradation (%) over Time Units')
    plt.xlabel('Time Units (Day)', fontsize = 16)
    plt.ylabel('Battery Degradation (%)', fontsize = 16)
    plt.legend()
    # plt.grid(True)
    plt.savefig(f'bat_degradation_BTC_M_{experiment_name}_{max_no_of_days}_days.pdf')
    # right_side = ax.spines["right"]
    # right_side.set_visible(False)
    # top_side = ax.spines["top"]
    # top_side.set_visible(False)
    plt.show()
 




