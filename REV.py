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

##create name of robot, chargin station and task class
class Robot:
    pass
class ChargingStation:
    pass
class Task:
    pass
#####Functions#######################

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
def update_parametets(k):
    checked_robots = []
    for robot in all_robots:
        if robot.status == "idle":
            robot.state_of_charge -= energy_consumption_while_idle
            robot.state_of_charge = max(0, robot.state_of_charge)
            state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
            checked_robots.append(robot.original_index)
            
        elif robot.status == "busy":
            if k <= robot.task_end_time:
                energy_consumption = calc_energy_consumption(robot.task_slope, incline = 1)
                robot.state_of_charge -= energy_consumption
                robot.state_of_charge = max(0, robot.state_of_charge)
                state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                checked_robots.append(robot.original_index)
                if k == robot.task_end_time:
                    # robot.state_of_charge = robot.state_of_charge
                    # state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                    robot.status = "idle"
                    task_end_loc = all_tasks[robot.allocated_task].end_loc
                    robot.cur_loc = task_end_loc
                    robot.allocated_task = None
                    robot.task_end_time = None
    #####check if robots are fully charged
    for robot in all_robots:
        if robot.status == "allocated to CS":  ###this is for if robot is charging
            print(f"checking allocated CS status")
            if robot.state_of_charge <= highest_soc:
                robot.state_of_charge += Charging_rate
                robot.state_of_charge = min(highest_soc, robot.state_of_charge)
                state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                checked_robots.append(robot.original_index)
                if robot.state_of_charge == highest_soc:
                    # robot.state_of_charge = robot.state_of_charge
                    # state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                    robot.status = "idle"
                    print(f"robot is made idle as it fully charged here")
                    robot.charge_start_time = None
                    selected_cs = all_CS[robot.allocated_CS]
                    if selected_cs.robot_waiting_to_charge == -1:
                        print(f"no robot is waiting so CS is freed")
                        selected_cs.CS_status = "free"
                        selected_cs.robot_charging = None
                    else:
                        next_robot_to_charge = selected_cs.robot_waiting_to_charge
                        print(f"there was robot waiting {next_robot_to_charge} so now its in robot charging and waiting queue is none")
                        selected_cs.robot_charging = next_robot_to_charge
                        all_CS[robot.allocated_CS].robot_waiting_to_charge = -1
                        print(f"took out robot waiting to robot charge so now robot waiting is {all_CS[robot.allocated_CS].robot_waiting_to_charge}")
                        all_robots[next_robot_to_charge].status = "allocated to CS" 
                                   
    for robot in all_robots:
        if robot.status == "going to CS":
            print(f"robot is going to cs---- {robot}")
            if k <= robot.charge_start_time:
                robot.state_of_charge -= fixed_energy_consumption
                robot.state_of_charge = max(0, robot.state_of_charge)
                state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                checked_robots.append(robot.original_index)
                if k == robot.charge_start_time:
                    selected_cs = all_CS[robot.allocated_CS]
                    if selected_cs.robot_charging == None:
                        robot.status = "allocated to CS"
                        selected_cs.robot_charging = robot.original_index
                        robot.allocated_task = None
                    else:
                        if selected_cs.robot_waiting_to_charge == -1:
                            selected_cs.robot_waiting_to_charge = robot.original_index  
            else:
                robot.state_of_charge -= energy_consumption_while_idle
                robot.state_of_charge = max(0, robot.state_of_charge)
                state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                checked_robots.append(robot.original_index)
                selected_cs = all_CS[robot.allocated_CS]
                if selected_cs.robot_charging == None:
                    robot.status = "allocated to CS"
                    selected_cs.robot_charging = robot.original_index
                    robot.allocated_task = None
                else:
                     if selected_cs.robot_waiting_to_charge == -1:
                        selected_cs.robot_waiting_to_charge = robot.original_index 
    if len(checked_robots) != len(all_robots):
        print(f"_%%%%%----- bugggg, checked robs = {checked_robots}, it is fixing now")
        diff_in_r = len(all_robots) - len(checked_robots)
        for robot in all_robots:
            if robot.original_index not in checked_robots:
                if robot.status == "allocated to CS":
                    robot.state_of_charge -= energy_consumption_while_idle
                    robot.state_of_charge = max(0, robot.state_of_charge)
                    state_of_charge_traces[robot.original_index].append(robot.state_of_charge)
                    diff_in_r -= 1
                    if diff_in_r == 0:
                        break
    
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


# Define a function that represents a worker thread
def calculate_matrix_row(matrix, avail_robots, avail_tasks, r_s, start_row, end_row):
    for i in range(start_row, end_row):
        robot = avail_robots[i]
        for j in range(len(matrix[0])):
            ###first check if robot will have atleast 25% energy after executing the task
            total_dis = (manhattan_distance(robot.cur_loc, avail_tasks[j].start_loc)) + avail_tasks[j].total_distance_to_cover
            slope = avail_tasks[j].t_slope
            total_time_required = round (total_dis/r_s)
            # print(f"total time required for task {avail_tasks[j].original_index} with total dis {total_dis} is {total_time_required}")
            if total_time_required == 0:
                total_time_required = 1
            energy_consumption = calc_energy_consumption(slope, incline = 1)        
            total_e_to_complete_task = total_time_required * energy_consumption
            # print(f"----to complete task robot need {total_e_to_complete_task},robot will have {robot.state_of_charge - total_e_to_complete_task} left after task")
            if (robot.state_of_charge - total_e_to_complete_task) < lowest_soc:
                matrix[i][j] = DISALLOWED
            else:
                if total_dis == 0:
                    total_dis = 1
                matrix[i][j] = avail_tasks[j].value / total_dis

# Function to allocate threads and perform parallel computation for the matrix
def allocate_threads(matrix, avail_robots, avail_tasks, r_s, num_threads):
    rows_per_thread = len(matrix) // num_threads
    with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
        future_to_row = {
            executor.submit(
                calculate_matrix_row,
                matrix,
                avail_robots,
                avail_tasks,
                r_s,
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

####to remove rows with all Disallowed###
def remove_disallowed_rows(mat, avail_rob, need_charge_index):
    # Identifying the indices of the rows that contain all zeroes
    if len(mat) > 0:
        disallow_row_indices = [i for i, row in enumerate(mat) if all(x == DISALLOWED for x in row)]
        for i in disallow_row_indices:
            index=avail_rob[i].original_index
            need_charge_index.append(index) 
        # Removing the identified rows from the matrix
        new_mat = [row for i, row in enumerate(mat) if i not in disallow_row_indices]
        # Adjusting the avail_robots list accordingly
        new_avail_robots = [robot for i, robot in enumerate(avail_rob) if i not in disallow_row_indices]
    else:
        new_mat = mat
        new_avail_robots = avail_rob
    return new_mat, new_avail_robots, need_charge_index    
####func to delete all disallowed columns
def remove_disallowed_columns(matrix, avail_tasks):
    if len(matrix) > 0:
        # Identifying the indices of the columns that contain all disallowed values
        disallowed_col_indices = {j for j in range(len(matrix[0])) if all(matrix[i][j] == DISALLOWED for i in range(len(matrix)))}
        
        # Removing the identified columns from the matrix
        new_matrix = [[row[j] for j in range(len(row)) if j not in disallowed_col_indices] for row in matrix]
        
        # Adjusting the avail_tasks list accordingly
        new_avail_tasks = [task for i, task in enumerate(avail_tasks) if i not in disallowed_col_indices]
    else:
        new_matrix = matrix
        new_avail_tasks = avail_tasks    
    return new_matrix, new_avail_tasks

#####function to do allocation###
def allocation(k, all_robots, all_tasks, avail_robots, avail_tasks, needs_charge_robot_indexes):
    total = None
    # Create matrix and initialize variables
    matrix = [[0 for _ in range(len(avail_tasks))] for _ in range(len(avail_robots))]
    # Call the function to allocate threads using ThreadPoolExecutor
    matrix = allocate_threads(matrix, avail_robots, avail_tasks, r_s, num_threads)
    print("------before tunning-matrix:")
    matrix, avail_robots, needs_charge_robot_indexes = remove_disallowed_rows(matrix, avail_robots, needs_charge_robot_indexes)
    matrix, avail_tasks = remove_disallowed_columns(matrix, avail_tasks) 
    print("------after tunning-matrix:")
    # for row in matrix:
    #     print(row) 
    if len(matrix) != 0:    
        decision, total = munkres_func(matrix) ##run the matrix through munkres
        for robot_index in decision:
            robot = avail_robots[robot_index]
            col_index = decision[robot_index]
            # If matched with a task
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
            print(f"----Robot {robot.original_index} matched with Task {task.original_index}")
    return all_robots, all_tasks, needs_charge_robot_indexes, total

####to find the closet CS
def find_closest_CS(robot, avail_CS, all_CS):
    # Sort all CS by distance to the robot
    sorted_CS = sorted(all_CS, key=lambda cs: manhattan_distance(robot.cur_loc, cs.CS_location))    
    for cs in sorted_CS:
        if cs in avail_CS:
            return cs, True # Available CS
    return sorted_CS[0], False # No available CS, return the closest one
####to update any attribute in the all list
def update_object_parameters(selected_object, attribute_updates, all_objects):
    # Find and update the object in all_objects list
    for o in all_objects:
        if o.original_index == selected_object.original_index:
            for attribute, new_value in attribute_updates.items():
                setattr(o, attribute, new_value)
            break
###calc CS_reaching time
def calc_cs_reaching_time(robot, cs, r_s, k):
    dis_to_CS = manhattan_distance(robot.cur_loc, cs.CS_location)
    if dis_to_CS == 0:
        CS_reaching_time = k
    else:
        time_to_reach_CS = round(dis_to_CS/r_s)
        if time_to_reach_CS == 0:
            time_to_reach_CS = 1    
        CS_reaching_time = k+time_to_reach_CS
    return CS_reaching_time
     

####function to schedule charging
def charging_decision(need_charge_indexes, all_robots, all_CS, avail_CS):
    print(f"---inside charging function first we have need charge robots {need_charge_indexes}")
    for robot_index in need_charge_indexes:
        robot = next((r for r in all_robots if r.original_index == robot_index), None)
        print(f"robot index of need charge robot in all_robots")
        #Find the closest available CS or the closest CS if none are available
        if robot:
            rob_index = robot.original_index
            closest_CS, is_available = find_closest_CS(robot, avail_CS, all_CS)
            if closest_CS:
                if is_available:
                    print(f"robot {robot.original_index} is matched with avail CS {closest_CS.original_index}")
                    cs_reaching_time = calc_cs_reaching_time(robot, closest_CS, r_s, k)
                    print(f"robot {robot.original_index} 's cs reaching time to avail CS {closest_CS.original_index} is {cs_reaching_time}")
                    robot.allocated_CS = closest_CS.original_index
                    if cs_reaching_time == k:
                        attribute_updates = {"CS_status": "occupied", "charge_start_time": k+1, "robot_charging": robot_index}
                        update_object_parameters(closest_CS, attribute_updates, all_CS)
                        robot.status = "allocated to CS"
                        robot.charge_start_time = k+1
                        robot.cur_loc = closest_CS.CS_location
                        print("robot", robot)
                        print("closest cs", closest_CS)
                    else:
                        attribute_updates = {"CS_status": "occupied", "charge_start_time": cs_reaching_time}
                        update_object_parameters(closest_CS, attribute_updates, all_CS)
                        robot.status = "going to CS"
                        robot.allocated_CS = closest_CS.original_index
                        robot.charge_start_time = cs_reaching_time
                        robot.cur_loc = closest_CS.CS_location
                        print("robot", robot)
                        print("closest cs", closest_CS)
                    print(f"---no of avail_cs before allocating each cs:{len(avail_CS)}")    
                    avail_CS.remove(closest_CS)
                    print(f"---no of avail_cs after allocating each cs:{len(avail_CS)}")
                else:
                    print(f"robot {robot.original_index} is matched with occupied CS {closest_CS.original_index}")
                    cs_reaching_time = calc_cs_reaching_time(robot, closest_CS, r_s, k)
                    print(f"robot {robot.original_index} 's cs reaching time to occupied CS {closest_CS.original_index} is {cs_reaching_time}")
                    robot.allocated_CS = closest_CS.original_index
                    if cs_reaching_time == k:
                        attribute_updates = {"CS_status": "occupied", "charge_start_time": k+1}
                        update_object_parameters(closest_CS, attribute_updates, all_CS)
                        robot.status = "going to CS"
                        robot.charge_start_time = k+1
                        robot.cur_loc = closest_CS.CS_location
                        print("robot", robot)
                        print("closest cs_occupied", closest_CS)
                    else:
                        attribute_updates = {"CS_status": "occupied", "charge_start_time": cs_reaching_time}
                        update_object_parameters(closest_CS, attribute_updates, all_CS)
                        robot.status = "going to CS"
                        robot.charge_start_time = cs_reaching_time
                        robot.cur_loc = closest_CS.CS_location
                        print("robot", robot)
                        print("closest cs_occupied", closest_CS)
                print(f"need charge rbots list after allocating to robot{need_charge_indexes}")        
    need_charge_indexes = []
    print(f"need charge rbots list after allocting to robot{need_charge_indexes}")  
    return all_robots, all_CS, need_charge_indexes                       
   
####function to print avail_lists
def print_lists(k, robot_list, task_list, cs_list):
    print(f"---Robots in for k = {k}:")
    # for robot in robot_list:
    #     print(f"robot: {robot.__dict__}")
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
    #     print(f"cs {cs.__dict__}")         
        
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
    # energy_consump = 4
    return energy_consump 
  
###-----read parameter from json###########
# Helper function to load from JSON
def load_all_from_json(filename):
    with open(filename, 'r') as file:
        return json.load(file)
# Load all data
all_data = load_all_from_json('exp1_data.json')
robots_data = all_data["robots"]
charging_stations_data = all_data["charging_stations"]
tasks_data = all_data["tasks_per_k"]

# Load parameters and extract T
params = all_data['parameters']
task_min = params.get('task_min', None)  # Default to None if not found
task_max = params.get('task_max', None)  # Default to None if not found
no_of_old_robs = params.get('no_of_old_robs', None)
T = params.get('T', None)
task_deadline = params.get('task_deadline', None)
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
# Populate lists with data
all_robots = []
for robot_data in robots_data:
    robot = Robot()
    for key, value in robot_data.items():
        setattr(robot, key, value)
    all_robots.append(robot)
    
all_CS = []
for cs_data in charging_stations_data:
    cs = ChargingStation()
    for key, value in cs_data.items():
        setattr(cs, key, value)
    all_CS.append(cs)

#######Parameters initialization
num_threads = 16
###define days parameter
no_of_days = -1
day_start = 0   ##start period of any new day
day_end = day_start + periods_per_day - 1  ##end period of any new day
days_per_month = 1 ###it will be 30 if we consider 30 days
period_length_sec = params.get('period_length_sec', None)
r_s = 1.7 * period_length_sec ##in meter/sec or metter/minute according to period puration. the multiplication with period_length is done to convert it according to the decision period



### Define constants and variables
old_deg_min = params.get('old_deg_min', None)
old_deg_max = params.get('old_deg_max', None)
new_deg_min = task_min
new_deg_max =  task_max
soc_threshold = 50 ##this is for v_soc calculation
cpu_frq = 2.26 ## cpu frequency in GHz
rob_speed = 1.6 ###m/s to use in the energy consumption calc
rob_weight = 20  ##in kg
gravity_acceleration = 9.8 ##in m/s2
power_consumption = 38.213568  ###reference value of power consumption in W for certain cpu freq and robot speed
fixed_energy_consumption = round((power_consumption * period_length_sec/3600),8) ##energy consumption in WH, we will use it when not considering inclinations 
# fixed_energy_consumption = 5  ###just to test with simple values
energy_consumption_while_idle = params.get('energy_consumption_while_idle', None)

### Define initial energy balance for robots for smaller battery
highest_soc = params.get('highest_soc', None)# Battery capacity (Wh) for smaller battery
lowest_soc = params.get('lowest_soc', None)# Battery capacity (Wh) for smaller battery
lowest_soc = 3

Charging_time = 0.75 # hrs to completely recharge the battery
Charging_rate = Ebat/(Charging_time*3600/period_length_sec) #Wh recharged during an entire period in charging mode

#######old robot soc history extraction
old_soc_file = params.get('old_soc_file', None)  ###to read previous soc trace of older robots
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
needs_charge_robot_indexes = []
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
        all_robots, all_CS, all_tasks, state_of_charge_traces = update_parametets(k) 
        print(f"#####print all lists after parameter updation at the beginning of k={k}")        
        ####---new addition for per day result storage---#####
        if k == day_end:
            ##updating the start and end day period
            no_of_days +=1
            day_start = k  
            day_end = day_start + periods_per_day
            ###calc the bat deg with concurrent threading
            with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
                # Submit tasks to the ThreadPoolExecutor for concurrent execution
                futures = [executor.submit(calculate_bat_deg, robot) for robot in all_robots]
                # Wait for all threads to complete
                concurrent.futures.wait(futures)
        print_lists(k, all_robots, all_tasks, all_CS)
        if k==T:   ###this is to update the soc for the last allocation at T-1
            print("----end of working period----")
            break    
    ###update all_tasks by including the new tasks
    if str(k) in tasks_data:  # Ensure the key exists in the tasks_data dictionary
        tasks = tasks_data[str(k)]  # Retrieve the list of task dictionaries for k = 3
        for task_data in tasks:
            task = Task()
            for key, value in task_data.items():
                setattr(task, key, value)
            all_tasks.append(task)   
    ###create new tasks and update the avail_tasks list                                  
    avail_robots, avail_CS, avail_tasks = update_avail_lists(k) 
    print(f"------print avail lists after making new avail_lists from all lists at the beginning of k={k}")
    print_lists(k, avail_robots, avail_tasks, avail_CS)       
    ####call allocation####
    if len(avail_robots) != 0:
        print(f"robots available {len(avail_robots)} robots")
        alloc_start = time.time()
        if len(avail_tasks) > 0:
            print(f"task available {len(avail_tasks)} tasks")
            all_robots, all_tasks, needs_charge_robot_indexes, total = allocation(k, all_robots, all_tasks, avail_robots, avail_tasks, needs_charge_robot_indexes)
            if total:
                total_utility.append(total)
            print(f"after allocation needs charge robots are {needs_charge_robot_indexes}")
            if len(needs_charge_robot_indexes) > 0:
                print("in charging decision func after allocation")
                all_robots, all_CS, needs_charge_robot_indexes = charging_decision(needs_charge_robot_indexes, all_robots, all_CS, avail_CS)    
            print(f"----end of allocation for k={k}")
        else:
            for robot in avail_robots:
                if robot.state_of_charge <= lowest_soc:
                    needs_charge_robot_indexes.append(robot.original_index)
            print(f"no task is avail so checking if robot needs charge {needs_charge_robot_indexes}")
            if len(needs_charge_robot_indexes) > 0:
                print("no task bt robot needs charge so will call charging decision")
                all_robots, all_CS, needs_charge_robot_indexes = charging_decision(needs_charge_robot_indexes, all_robots, all_CS, avail_CS)
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
print("% gained value for REV:", percentage_gained_valuation) 
print(f"execution time per allocation for REV {per_alloc_time}") 
###make the energy value as SoC percentage
v = (100/Ebat)
soc_trace_percentage = {}
for robot_index, charges in state_of_charge_traces.items():
    soc_trace_percentage[robot_index] = [charge * v for charge in charges]
# print(f"soc trace:", state_of_charge_traces)
bat_deg_perc = {}
for robot_index, degrad in bat_deg_traces.items():
    bat_deg_perc[robot_index] = [(100*deg) for deg in degrad]
    

# per_period_execution_time = (total_time / T)
# print(f"----!!! per period execution time for REV: {per_period_execution_time}")   

###to create dataframes
df1 = pd.DataFrame()    
df1['REV_gained_value'] = percentage_gained_valuation
df1['REV_per_alloc_exec_time'] = per_alloc_time
# df1['REV_per_peri_exec_time'] = per_period_execution_time

       
###write reults in a csv
bat_deg_data = {'REV_deg_Rob{}'.format(robot.original_index): bat_deg_perc[robot.original_index] for robot in all_robots}
df2 = pd.DataFrame(bat_deg_data)

soc_data = {'REV_soc_Rob{}'.format(robot.original_index): state_of_charge_traces[robot.original_index] for robot in all_robots} 
df3 = pd.DataFrame(soc_data)
utility_data = total_utility
df4 = pd.DataFrame(utility_data)



# Save DataFrame to a CSV file
experiment_name = "exp1"

File_name1 = f"results_REV_{experiment_name}.csv"
File_name2 = f"bat_deg_REV_{experiment_name}.csv"
File_name3 = f"soc_trace_REV_{experiment_name}.csv"
# File_name4 = f"total_utility_REV_{experiment_name}.csv"


df1.to_csv(File_name1, index=False)
df2.to_csv(File_name2, index=False)
df3.to_csv(File_name3, index=False)
# df4.to_csv(File_name4, index=False)

# Plotting the SoC trace
if max_no_of_days == 1:
    plt.rcParams['font.size'] = '18'
    plt.rcParams["legend.fontsize"]= '14'
    plt.rcParams["axes.labelweight"] = "bold"
    plt.rcParams.update({'figure.autolayout': True})
    fig, ax = plt.subplots()

    for robot_index, charges in soc_trace_percentage.items():
        plt.plot(range(0,T+1), charges, label=f'AGR {robot_index}')
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
    # plt.grid(True)
    plt.savefig(f"state_of_charge_REV_{experiment_name}.pdf")
    plt.show()

#### plot bat deg for the robots
if max_no_of_days > 1:
    for robot_index, degradation in bat_deg_perc.items():
        plt.plot(range(0,no_of_days + 2), degradation, label=f'AGR {robot_index}')
    # plt.axhline(y=(highest_soc / Ebat) * 100, color='r', linestyle='-', label="Emax")
    plt.xlim([0, no_of_days + 1])
    # plt.ylim([0, 1.2]) 
    # plt.ylim([0, 119]) 
    plt.xticks(range(0,no_of_days+1, 1), fontsize = 18)#
    plt.yticks(fontsize = 18)
    # for l in range(0, T+1, 2):
    #     plt.axvline(x=[l], color='grey', alpha=0.1)
    # plt.title('Bat degradation (%) over Time Units')
    plt.xlabel('Time (Day)', fontsize = 18)
    plt.ylabel('Battery Degradation (%)', fontsize = 18)
    plt.legend()
    # plt.grid(True)
    plt.savefig(f'bat_degradation_REV_{experiment_name}_{max_no_of_days}_days.pdf')
    plt.show()
