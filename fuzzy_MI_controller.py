"""
Copyright (c) 2022-2023
All rights reserved.
@author: Dimitris Panagopoulos

Script that implements a fuzzy logic controller namely (Expert-guided Mixed-Initiative Control 
Switcher - EMICS). It can be used in the experiment conducted for ERL described here
(https://github.com/uob-erl/fuzzy_mi_controller) as an alternative to the FuzzyLite library that 
has been utilized to create the fuzzy inference system so far. 

Specifically, the engine could be removed in "mi_fuzzy_controller_node.cpp" and this class could be 
called instead in lines 227-230. The only thing needed is to publish the error and speed value
to be taken into account in the class by subscibing to ROS topics 'vel_error_average_' and 
'cmd_vel_robot_.linear.x', respectively. 
"""

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class MixedInitiativeContoller:
    def __init__(self):
        self.input_1_space = np.arange(0, 0.105, 0.005)   # the range of all possible values for input1
        self.input_2_space = np.arange(-0.4, 0.405, 0.05) # the range of all possible values for input2
        self.output_space = np.arange(-1.0, 1.25, 0.25)   # the range of all possible values for output
        self.defuzzify_method = 'lom'  # selected defuzzication method
        
        # Create the input and output fuzzy sets
        self.input_1 = ctrl.Antecedent(self.input_1_space, 'error')
        self.input_2 = ctrl.Antecedent(self.input_2_space, 'speed')
        self.output = ctrl.Consequent(self.output_space, 'LOA decision', self.defuzzify_method)
        
        # Create the membership functions for the inputs and output 
        self.input_1['small'] = fuzz.trapmf(self.input_1.universe, [0, 0, 0.035, 0.060])
        self.input_1['medium'] = fuzz.trapmf(self.input_1.universe, [0.045, 0.055, 0.065, 0.080])
        self.input_1['large'] = fuzz.trapmf(self.input_1.universe, [0.065, 0.085, 0.1, 0.1])
        
        self.input_2['reverse'] = fuzz.trapmf(self.input_2.universe, [-0.4, -0.4, -0.03, -0.02])
        self.input_2['zero'] = fuzz.trimf(self.input_2.universe, [-0.03, 0, 0.03])
        self.input_2['forward'] = fuzz.trapmf(self.input_2.universe, [0.02, 0.03, 0.4, 0.4])
        
        self.output['no_change'] = fuzz.trimf(self.output.universe, [-1, -1, 0])
        self.output['change'] = fuzz.trimf(self.output.universe, [0, 1, 1])
        
        
    # Create the fuzzy rule base
    def fuzzy_rules(self):
        # IF error is small OR error is medium THEN LOA is no change
        rule1 = ctrl.Rule(self.input_1['small'] | self.input_1['medium'], self.output['no_change'])
        # IF error is large AND speed is not reverse THEN LOA is change
        rule2 = ctrl.Rule(self.input_1['large'] & ~self.input_2['reverse'], self.output['change'])
        # IF speed is reverse AND error is large THEN LOA is no change
        rule3 = ctrl.Rule(self.input_2['reverse'] & self.input_1['large'], self.output['no_change'])
        
        return [rule1, rule2, rule3]
        
    
    def controllerANDinference(self, error_value, speed_value, view):
        controller = ctrl.ControlSystem(self.fuzzy_rules()) # Define the fuzzy control system
        self.fuzzy_inference = ctrl.ControlSystemSimulation(controller) # simulate the fuzzy control system
        
        self.fuzzy_inference.input['error'] = error_value  # specify the input 1
        self.fuzzy_inference.input['speed'] = speed_value  # specify the input 2
        self.fuzzy_inference.compute()   # call the compute method
        print(round(self.fuzzy_inference.output['LOA decision'], 3))  # print the output result
        
        self.view_plots(view)  # view the plots
        
        return self.fuzzy_inference.output['LOA decision']
        
    
    def view_plots(self, view_plots):
        if view_plots:
            self.input_1.view()
            self.input_2.view()
            self.output.view()
            self.output.view(sim=self.fuzzy_inference)
        else:
            print('no plots')


# # comment out should you wish to run a dummy example to check the functionality of the system
# # in general we don't need .. make sure the below is commented
# import random
# error = 0.04  # this value is obtained by subscribing to ROS topic 'vel_error_average_'
# speed = 0.2   # this value is obtained by subscribing to ROS topic 'cmd_vel_robot_.linear.x'
# view_plots = False  # default False
# ITERATOR = 1
# EPISODES = 3
# while ITERATOR <= EPISODES:
#     print("error:", error, "speed:", speed)       
#     MI = MixedInitiativeContoller()
     
#     inference = MI.controllerANDinference(error, speed, view_plots)       
#     ITERATOR += 1
#     error = round(random.uniform(0, 0.1), 2)    # this value is obtained by subscribing to ROS topic 'vel_error_average_'
#     speed = round(random.uniform(-0.4, 0.4), 2) # this value is obtained by subscribing to ROS topic 'cmd_vel_robot_.linear.x'
