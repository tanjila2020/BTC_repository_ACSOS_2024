
**Abstract**
In the context of multi-AGR task allocation, sustainable practices are essential, especially for managing battery degradation, which influences the long-term sustainability of battery-operated systems. Therefore, coordinating the task and charging schedules of AGR fleet is crucial. 
Most existing studies are either based on offline methods, which are unsuitable for online scenarios requiring instant decisions, or concentrated on maximizing task allocation and resource usage without considering battery life. This accelerates battery degradation and reduces 
environmental sustainability of AGR fleets. Considering these challenges, this paper proposes a family of two online joint task allocation and charge scheduling algorithms aiming to maximize the total revenue while minimizing the battery degradation. The first algorithm is based 
on the Kuhn-Munkres approach that allocates tasks at each event optimally. The second algorithm utilizes a greedy approach achieving a sub-optimal solution with reduced computational overhead. Unlike existing algorithms, our approach factors in battery degradation as part of the
scheduling decisions aiming to achieve a balance between maximizing total task revenue and minimizing the battery degradation over the AGR’s operational lifespan. Our results, obtained through extensive simulations based on a real AGR, show that it is possible to achieve up to 
20% longer battery lifespan with minimal revenue losses.

Following are the important packages that are required to execute the BTC_M.py, BTC_G.py, DEG.py and REV.py files:
|package | pip installation command | 
| ------------- | ------------- | 
| [numpy](https://numpy.org/install/)  | `pip install numpy`  |   
| [matplotlib](https://matplotlib.org/stable/users/installing/index.html) | `python -m pip install -U matplotlib`|  
| [time](https://pypi.org/project/times/) | `pip install times`|  
| [pandas](https://pandas.pydata.org/docs/getting_started/install.html) | `pip install pandas`|  
| [math](https://pypi.org/project/python-math/) | `pip install python-math` |
| [munkres](https://pypi.org/project/munkres/) | `pip install munkres` |
| [futures](https://pypi.org/project/futures/) | `pip install futures` |

### Note
-Its better to use the updated version of python. Currently Python 3.9.13 is used.
-Instead of PIP any other python package manager can be used by writing the respective command instad of PIP.

-For instance, if conda environment is used then the similar conda commands will be needed. For example, conda install pandas, conda install matplotlib etc. These commands 
can be easily found with a quick internet search.

-The futures library is needed to be able to execute the codes given in this repository with threading.

-The rainflow model has been integrated by adding the two files named 'BatteryDegradation.py' and 'getDegradation.py'. Make sure these two files are in the same folder with the other files.

-Any other battery degradation model can be applied by replacing the getDegradation.py file with respective model codes and feeding the state of charge traces.

-Most of the functions are the same across the four approaches except for the calculate_matrix_rows() which builds the bipartite graph by calculating
utilities described in the paper and the allocation() which updates the parameters according to the allocation decision. The BTC-M uses the munkres() function to get the optimal allocation where as the BTC-G uses alternate_munkres() function
which is actually the greedy selection to find a suboptimal allocation solution.

This repository contains the proposed BTC-M and BTC-G algorithms and other baselines DEG and REV explained in the paper. It also contains a `create_experiment.py` file which creates initial parameters in random order for different experiment setups
and can create experiments with parameters for single or multiple days to run. To test the Battery capacity degradation metric, experiments for atleast 720 days
need to be run to actually evaluate the difference in capacity dedradation across our algorithms and the baselines. This is because the battery degrades over
a long period.
This can be easily done by changing the value of the variable "max_no_of_days" (in line 111) to 720 days in the `create_experiment.py` file.
### Baselines:
**REV** is a sample baseline similar to the state of the art studies which focus on maximizing the allocated tasks which in our case translates to maximizing
 the total revenue generated from allocating tasks. REV tries to allocate tasks to the AGRs to maximize the total revenue along with taking into consideration 
 the distance needed to travel for executing tasks to further optimize the task allocation.

**DEG** is a sample baseline which prioritizes minimizing battery degradation while allocating tasks to the AGRs without focusing on high revenue generation.
### Our Algorithms:

**BTC-M** It is the proposed algorithm which uses our utility based proposed allocation() algorithm and uses weighted matching algorithm based on the 
Kuhn-Munkres approach to finalize the allocation decision from the genrated utility matrix which defines a bipartite graph. 

**BTC-G** It is a variant of BTC-M which utilizes a greedy approach to provide a sub-optimal allocation decision. 


### Creating test scenarios using  `create_experiment.py` file:
To create different test scenarios, change the range of variables used in `create_experiment.py`. 
- The tunable parameters for robots and charging station objects are: R, the number of AGRs (in line 104), C_to_R_ratio, cs_loc.
- To test for different total number of tasks change the value of "tot_order" in line 116. To test with different hourly distribution of the task arrivals, 
  slightly modify the values of the variable "hourly_order_perc" (line 115),
  make sure the total numbers add up to 1 in that case.
- For testing with multiple number of days set the parameter "max_no_of_days" to be more than 1 (line 111).  To vary the total number of orders in each day 
  change the percentage in the variables "min_order_perc" and
  "max_order_perc". This will vary the total number of orders across the days.
- To change task related parameters, change the value of "total_area" in line 133 from 700-1000. The range of task valuation also can be changed by changing 
  "task_min" and "task_max".
- To test the performance of BTC-M with different β1 values change the value of "q1" in line 148 by any value from 0.1-10. For our experiments we used the 
  same value for β1 and β2. So make sure change the q2 value to as q1.
- To try having some of the AGRs with older batteries, set "no_of_old_robs" from 1 to R. setting R means all the AGRs battery will have some prior degradation. 
  Note that the file "history_soc_trace_6.2%_deg.csv" stored in the variable "old_soc_file" in line 162 must have the same number of columns as the value of 
  "no_of_old_robs" set, because it contains the history of the old robots. Current file contains history of maximum 4 AGRs.
  Testing with any values of "no_of_old_robs" less than 5 should work without changing the current csv file.
 

Running this `create_experiment.py` file successfully will create a JSON file currently named as  `exp1_data.json` having all the parameters needed for all the approaches to execute. 

### Executing scripts of the baselines and our proposed approaches:
1. Just run the scripts  BTC_M.py and  BTC_G.py with the same `exp1_data.json` file to evaluate their performance (already written everything to run without 
   changing anything). 
2. The script will automatically update the experiment results in the designated CSV files (written at the end of the code for each approaches) upon completion.
4. Repeat the same for executing baseline scripts DEG.py, REV.py.
5. To visualize the State of Charge plot, make sure to put the value of the variable "max_no_of_days" to 1 while running the `create_experiment.py` file.
   The plot can be nicely observed with a single day run as it has reasonable data points.
6. Finally one can compare the performances by evaluating the files named for example "results_BTC_M_exp1.csv" and "bat_deg_BTC_M_exp1.csv" with the 
   respective result files from the other scripts.
7. Note that because of the random nature of most of the experimental variables it is preferrable to run each script at least 2-3 times and then get the
   average results for better consistency in the results.

**Citation**

@INPROCEEDINGS {ACSOS2024,
author = {Syeda Tanjila Atik, Daniel Grosu and Marco Brocanelli},
booktitle = {},
title = {Sustainability-aware Online Task and Charge Allocation for Autonomous Ground Robot Fleets},
year = {2024},
pages = {},
doi = {},
url = {},
publisher = {IEEE},
address = {},
month = {}
}
