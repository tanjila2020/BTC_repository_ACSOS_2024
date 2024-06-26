# -*- coding: utf-8 -*-
"""
Created on Thu May 11 19:18:21 2023

@author: akshar chavan
NOTE:
    The program takes in either the filepath and name or the list of soc's
    The total operating time is taken from the length of the given SOC's which results in obtaining desired results
"""



import rainflow  
import pandas as pd
from math import sin, cos, exp
import statistics

class BatteryDegradation():
    
    linear_degradation = 0
    non_linear_degradation = 0
    
    def __init__(self, soc):
        if isinstance(soc, str):
            self.soc_file_name = soc
            self.df = pd.read_csv(self.soc_file_name, header=None)
            if self.df[0].mean() > 1 :
                self.df[0] = self.df[0]/100
            self.soc = self.df[0].values.tolist() # State of Charge of Battery in percent 
        else:
            self.soc = soc
        self.ts = 600 # sample time for the soc data collection
        self.T = 25 # temperature of the battery (Celcius)
        self.t_tot = len(self.soc) * 10  * 60     # Total operating time of battery (seconds)
        self.dischargeDepth = []
        self.meanSoc = []
        self.cycleCount = []
        self.cyclePeriod = []
        self.Crate = []
        self.d_SoC = []
        self.d_DoD = []
        self.getRainflow()
        self.Linear_degradation()
        self.Nonlinear_degradation()
        
    @classmethod
    def get_linear_degradation(cls, linear_degradation):
        cls.linear_degradation = linear_degradation          
        
    @classmethod
    def get_non_linear_degradation(cls, non_linear_degradation):
        cls.non_linear_degradation = non_linear_degradation
            
    def getRainflow(self):
        for rng, mean, count, i_start, i_end in rainflow.extract_cycles(self.soc): 
            self.dischargeDepth.append(rng)
            self.meanSoc.append(mean)
            self.cycleCount.append(count)
            self.cyclePeriod.append(i_end - i_start)
            self.Crate.append(rng * 2 * count / (self.ts/3600))
        
        
    def Linear_degradation(self):

        # SoC stress model
        k_soc = 1.039
        SoC_ref = .6
        
        # DoD stress model
        k_DoD2 = 2.03 # DoD stress model nonlinear coefficient
        k_DoD1 = .2/(3000*.8 ** k_DoD2) # 3000 cycles @ 80% DoD till 80% end of life
        
        #cell temperature effect
        k_t = 0.0693
        T_ref = 25
        d_temp = exp(k_t * (self.T-T_ref) * (273 + T_ref)/(273 + self.T))
        
        # C-rate effect
        d_Crate = 1
    
        for i in range(0, len(self.dischargeDepth)):
            self.d_SoC.append(exp(k_soc * (self.meanSoc[i] - SoC_ref))) # SoC stress model
            self.d_DoD.append(k_DoD1 * self.dischargeDepth[i] ** k_DoD2) # DoD stress model
        
        # calender ageing
        k_cal = 3.31e-9/8
        SoC_avg = statistics.mean(self.meanSoc)
        
        d_cycle = sum(self.d_DoD[i] * self.d_SoC[i] * d_Crate * d_temp * self.cycleCount[i] for i in range(0, len(self.dischargeDepth))) # total cycle ageing
        d_cal = k_cal * self.t_tot * d_temp * exp(k_soc * (SoC_avg - SoC_ref))   
        linear_degradation = d_cycle + d_cal
        self.get_linear_degradation(linear_degradation)
        


    def Nonlinear_degradation(self):
        a = .0575
        b = 121
        
        non_linear_degradation = 1 - a*exp(-b*self.linear_degradation) - (1-a)*exp(-self.linear_degradation)
        self.get_non_linear_degradation(non_linear_degradation)
        
