import json
import random
import math

# Helper function to save to JSON
def save_all_to_json(data, filename):
    with open(filename, 'w') as file:
        json.dump(data, file, indent=4)

###---functions to generate orders----##
def generate_random_sum(target_sum, small_time_units):
    parts = [0] * small_time_units
    for _ in range(target_sum):
        parts[random.randint(0, small_time_units-1)] += 1
    return parts

###no of orders list creation
def no_of_orders_list_creation(max_no_of_days, small_time_units, hourly_order_perc, periods_per_day, days_per_month, tot_order, min_order_perc, max_order_perc, demand_increase_dur, demand_increase_month_start, demand_increase_month, demand_increase_perc):
    orders_for_days = []
    ###day wise order no creation
    for day in range(max_no_of_days):    
        if day == demand_increase_month:
            tot_order = round(tot_order * (1 + demand_increase_perc))
            orders_for_days.append(tot_order)
            demand_increase_month_start = day
            demand_increase_month = demand_increase_month_start + demand_increase_dur
        else:    
            new_tot_order = round(tot_order * (round(random.uniform(min_order_perc, max_order_perc), 2)))
            orders_for_days.append(new_tot_order)
    ####order per day now will be made orders per hour for all days
    hourly_orders_for_all_days = [] ##this list holds all the orders split in hour wise according to the percentage
    for i in orders_for_days:
        orders = [math.ceil(i * perc) for perc in hourly_order_perc]
        hourly_orders_for_all_days.extend(orders)    
    No_of_orders = []  ##this holds all the orders of all days according to the smallest period set
    for orders in hourly_orders_for_all_days:
        No_of_orders.extend(generate_random_sum(orders, small_time_units))
    return orders_for_days, No_of_orders

### function to calc required no of robots.here we are assuming the avg_task_completion time is given in mins
def calc_required_no_R(avg_task_comp_time, total_orders, demand_to_meet):
    considered_orders = math.ceil(total_orders * demand_to_meet)
    R_tasks_per_hour = 60 / avg_task_comp_time  ###bcz 60 min = 1hr
    R_tasks_per_day  = R_tasks_per_hour * 24
    robot_required = math.ceil(total_orders / R_tasks_per_day)
    return robot_required

####function to calculate manhattan distance between two points
def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

##create robot, charging station, and task objects
class Robot:
    def __init__(self, original_index, k):
        self.original_index = original_index
        self.arrival_period = k
        self.battery_status = "new"
        self.cur_loc = (random.randint(0, total_area), random.randint(0, total_area))
        self.state_of_charge = round(random.uniform(Ebat, Ebat), 4)  ##to have values up to two decimal points
        self.initial_deg = 0
        self.status = "idle"
        self.task_end_time = None
        self.allocated_task = None
        self.charge_start_time = None
        self.allocated_CS = None
        self.CS_reaching_time = None
        self.task_slope = None

    def __str__(self):
        return f"Robot {self.original_index}: cur_loc={self.cur_loc}, task_end_time={self.task_end_time}, charge_start_time={self.charge_start_time}, CS_reaching_time={self.CS_reaching_time}, allocated_CS={self.allocated_CS}, allocated_task={self.allocated_task}, state_of_charge={self.state_of_charge}, status={self.status}, bat_status={self.battery_status}"

class ChargingStation:
    def __init__(self, CS_location, original_index):
        self.original_index = original_index
        self.CS_location = CS_location
        self.CS_status = "free"
        self.charge_start_time = None
        self.robot_charging = None
        self.robot_waiting_to_charge = -1
        self.cs_slope = round(random.uniform(1, 4), 2)

    def __str__(self):
        return f"ChargingStation {self.original_index}: CS_location={self.CS_location}, CS_status={self.CS_status}, charge_start_time={self.charge_start_time}, robot_charging={self.robot_charging}, robot_waiting_to_charge={self.robot_waiting_to_charge}"

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
        self.t_slope = round(random.uniform(-3, 3), 2)

    def __str__(self):
        return f"Task {self.original_index}: start_loc={self.start_loc}, end_loc={self.end_loc}, value={self.value}, arrival_time={self.arrival_time}, deadline={self.deadline}, total_distance_to_cover={self.total_distance_to_cover}, status={self.status}"

###robot and charging station creation
R = 4  ### No of robots
C_to_R_ratio = 0.8  ## charging station(CS) to robot ratio
cs_loc = 2  #total no of CS locations
no_of_CS_per_loc = math.ceil(R * C_to_R_ratio)  ## no of CS (charging stations) per location
C = no_of_CS_per_loc * cs_loc ##total no of charging stations

##--parameters to generate tasks and simulate task arrivals--###
max_no_of_days = 1
hourly_order_perc = [0.01, 0.01, 0.01, 0.02, 0.02, 0.03, 0.04, 0.05, 0.08, 0.09, 0.08, 0.05, 0.1, 0.12, 0.11, 0.09, 0.05, 0.02, 0.01, 0.01, 0]
tot_order = 100 ## total no of orders for each day
min_order_perc = 0.95
max_order_perc = 1.05

small_time_units = 20
periods_per_day = len(hourly_order_perc) * small_time_units
no_of_days = -1
day_start = 0
day_end = day_start + periods_per_day - 1
No_of_orders = []
days_per_month = 1
demand_increase_dur = 2 * days_per_month
demand_increase_month_start = 0
demand_increase_month = demand_increase_month_start + demand_increase_dur - 1
demand_increase_perc = 0
orders_for_days, No_of_orders = no_of_orders_list_creation(max_no_of_days, small_time_units, hourly_order_perc, periods_per_day, days_per_month, tot_order, min_order_perc, max_order_perc, demand_increase_dur, demand_increase_month_start, demand_increase_month, demand_increase_perc)
max_order_per_k = max(No_of_orders)
T = len(No_of_orders)

##### task related parameters
total_area = 800
task_min = 10
task_max = 100

task_deadline = 5
max_cs_counter = 3

###energy parameters
Ebat = 111.0
highest_soc = round(0.8 * Ebat)
lowest_soc = round(0.20 * Ebat)
soc_threshold = round(0.5 * Ebat)
period_length_sec = 60
energy_consumption_while_idle = round((3.514 * period_length_sec / 3600), 8)

q1 = 1
q2 = 1

#######create robots with newer and older batteries####
no_of_old_robs = 1
old_soc_file = 'history_soc_trace_6.2%_deg.csv'
old_deg_min = 0.00001
old_deg_max = 0.1
k = 0
demand_to_meet = 1
robots = [Robot(i, k).__dict__ for i in range(R)]
charging_locations = [(random.randint(0, total_area), random.randint(0, total_area)) for _ in range(cs_loc)]

charging_stations = []
for i in range(C):
    location_index = i % cs_loc
    CS_location = charging_locations[location_index]
    charging_station = ChargingStation(CS_location, i).__dict__
    charging_stations.append(charging_station)

###Generate tasks
tasks_per_k = {}
original_index = 0
for k, no_of_tasks in enumerate(No_of_orders):
    tasks_per_k[k] = [Task(k, original_index + i).__dict__ for i in range(no_of_tasks)]
    original_index += no_of_tasks

# Parameters
params = {
    'task_min': task_min, 'task_max': task_max, 'no_of_old_robs': no_of_old_robs, 'T': T, 'task_deadline': task_deadline,
    'max_cs_counter': max_cs_counter, 'max_no_of_days': max_no_of_days, 'periods_per_day': periods_per_day, 'tot_order': tot_order,
    'max_order_per_k': max_order_per_k, 'demand_to_meet': demand_to_meet, 'R': R, 'C': C, 'cs_loc': cs_loc,
    'total_area': total_area, 'Ebat': Ebat, 'highest_soc': highest_soc, 'lowest_soc': lowest_soc, 'soc_threshold': soc_threshold,
    'old_soc_file': old_soc_file, 'old_deg_min': old_deg_min, 'old_deg_max': old_deg_max, 'q1': q1, 'q2': q2,
    'period_length_sec': period_length_sec, 'energy_consumption_while_idle': energy_consumption_while_idle
}
all_data = {
    "robots": robots,
    "charging_stations": charging_stations,
    "tasks_per_k": tasks_per_k,
    "parameters": params
}

input_file = "exp1_data.json"
# Save all data to one JSON file
save_all_to_json(all_data, input_file)
