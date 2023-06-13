import itclab
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import skfuzzy as fuzz
from skfuzzy import control as ctrl

######################################################
# Use this script for evaluating model predictions   #
# and PID controller performance for the TCLab       #
# Adjust only PID and model sections                 #
######################################################

######################################################
# Fuzzy PID Controller                               #
######################################################
def fuzzy_pid(sp, pv, pv_last, dt):
    # Fuzzy membership functions
    sp_range = np.arange(0, 101, 1)
    pv_range = np.arange(0, 101, 1)
    op_range = np.arange(0, 101, 1)

    # Define fuzzy variables
    sp_fuzzy = ctrl.Antecedent(sp_range, 'setpoint')
    pv_fuzzy = ctrl.Antecedent(pv_range, 'process_variable')
    op_fuzzy = ctrl.Consequent(op_range, 'output')

    # Membership functions
    sp_fuzzy['low'] = fuzz.trimf(sp_range, [0, 0, 50])
    sp_fuzzy['medium'] = fuzz.trimf(sp_range, [0, 50, 100])
    sp_fuzzy['high'] = fuzz.trimf(sp_range, [50, 100, 100])

    pv_fuzzy['low'] = fuzz.trimf(pv_range, [0, 0, 50])
    pv_fuzzy['medium'] = fuzz.trimf(pv_range, [0, 50, 100])
    pv_fuzzy['high'] = fuzz.trimf(pv_range, [50, 100, 100])

    op_fuzzy['low'] = fuzz.trimf(op_range, [0, 0, 50])
    op_fuzzy['medium'] = fuzz.trimf(op_range, [0, 50, 100])
    op_fuzzy['high'] = fuzz.trimf(op_range, [50, 100, 100])

    # Fuzzy rules
    rule1 = ctrl.Rule(sp_fuzzy['low'] & pv_fuzzy['low'], op_fuzzy['low'])
    rule2 = ctrl.Rule(sp_fuzzy['low'] & pv_fuzzy['medium'], op_fuzzy['medium'])
    rule3 = ctrl.Rule(sp_fuzzy['low'] & pv_fuzzy['high'], op_fuzzy['high'])
    rule4 = ctrl.Rule(sp_fuzzy['medium'] & pv_fuzzy['low'], op_fuzzy['low'])
    rule5 = ctrl.Rule(sp_fuzzy['medium'] & pv_fuzzy['medium'], op_fuzzy['medium'])
    rule6 = ctrl.Rule(sp_fuzzy['medium'] & pv_fuzzy['high'], op_fuzzy['high'])
    rule7 = ctrl.Rule(sp_fuzzy['high'] & pv_fuzzy['low'], op_fuzzy['low'])
    rule8 = ctrl.Rule(sp_fuzzy['high'] & pv_fuzzy['medium'], op_fuzzy['medium'])
    rule9 = ctrl.Rule(sp_fuzzy['high'] & pv_fuzzy['high'], op_fuzzy['high'])

    # Control system
    fuzzy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
    pid_ctrl = ctrl.ControlSystemSimulation(fuzzy_ctrl)

    # Set inputs
    pid_ctrl.input['setpoint'] = sp
    pid_ctrl.input['process_variable'] = pv

    # Compute PID output
    pid_ctrl.compute()

    # Get output