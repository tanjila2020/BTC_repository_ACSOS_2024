# -*- coding: utf-8 -*-
"""
Created on Fri May 12 08:16:57 2023

@author: syeda atik
"""
import pandas as pd
from BatteryDegradation import *
import numpy as np

def rainflow_code(df):


    # load_soc = pd.read_csv("RA_RM_SoC_1.csv", header=None)
    # load_soc = load_soc.rename(columns={0: "SOC"})
    NonLinear_degradation = []


    # df =  create_trace_for_task(new_trace, 0, 6, 2)
    temp = pd.DataFrame()
    # df['SOC'] = load_soc['SOC']/100

    operating_days = 0
    stop = False
    #to get single degdaration
    temp = pd.concat([temp, df])
    soc = temp['SOC'].values.tolist()
    d = BatteryDegradation(soc)
    degradation = d.non_linear_degradation
    degradation = '{:.10f}'.format(degradation) ##to format after decimal point
    # print("degradation:", degradation)
    return degradation
    #####end
    ####the next section is for getting multiple degradation
    # while not stop:
    #     temp = pd.concat([temp, df])
    #     soc = temp['SOC'].values.tolist()
    #     d = BatteryDegradation(soc)  #("TCM_LP_soc.csv")
    #     NonLinear_degradation.append(d.non_linear_degradation)
    #     # operating_days = operating_days + 1
    #     # if d.non_linear_degradation >= 0.2:
    #     #     stop = True
    #     stop = True


    # n = pd.DataFrame(NonLinear_degradation)
    # print("degradation dataframe:", n)
    #####n.to_csv('RA_RM_degradation_result.csv')
    ###end of multiple degradation


    # print('NonLinear degradation: ',  d.non_linear_degradation)