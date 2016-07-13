#!/usr/bin/env python
import pygame, sys
from pygame.locals import *
import os
import random
from time import sleep
from math import ceil
import pandas as pd
import rospy
from std_msgs.msg import Bool
from random import random
from bisect import bisect
from denoise.denoise import denoise
import pickle
import timeit
from gridsim.gridsim import Simulation
from action_selection.action_selection import init_probs, pick_action, det_k, update_P, det_all_k, init_k, update_all_P
import operator

alpha_signal = []
result = 0

def weighted_choice(choices):
    values, weights = zip(*choices)
    total = 0
    cum_weights = []
    for w in weights:
        total += w
        cum_weights.append(total)
    x = random() * total
    i = bisect(cum_weights, x)
    return values[i]


def callback(data):
    global alpha_signal, subscription, subscribed, closed, responded, result
    alpha_signal.append(data.data)

    if len(alpha_signal) == 17:
        subscription.unregister()
        subscribed = False
        closed = make_decision()
        if closed:
            result = 1
        if not closed:
            result = 0
        alpha_signal = []
        responded = True
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", closed)


def make_decision():
    global alpha_signal
    if True in alpha_signal:
        return True
    else:
        return False


def get_action(actions_list):
   # action = random.choice(actions_set)
    action = weighted_choice(actions_list)
    return action


def get_response():
    global subscribed, subscription, responded, result, steps, counter
    subscription = rospy.Subscriber("/openbci/eyes_closed", Bool, callback)
    subscribed = True
    print 'Step {} of {}'.format(counter,steps)
    start = timeit.default_timer()
    while (not responded):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        pass
    stop = timeit.default_timer()
    print stop - start 
    responded = False
    return result


def start_simulation(x_size, y_size, steps):
    "this function works"

    global closed, counter
    rospy.init_node('gridworld', anonymous=True)
    #start_2D_grid(initial_x_state, initial_y_state, num_x_states, num_y_states)
    sim = Simulation('/home/sahabi/mo/lib/python/gridsim/config.txt','/home/sahabi/mo/lib/python/gridsim/matrix.txt')
    x_state_log = []
    y_state_log = []
    action_log = []
    response_log = []
    prev_x_state_log = []
    prev_y_state_log = []
    error_log = []
    x_denoise_log = []
    y_denoise_log = []
    x_entry_log = []
    y_entry_log = []
    current_state_list = []
    time_step_list = []
    evaluation_list = []
    done = sim.update()
    p = init_probs(x_size, y_size)
    s = init_k(x_size, y_size)
    current_state = (0, 0)
    actions = ['east','west','north','south']

    for i in range(0, steps):
        counter = i+1
        x_entry = [0,0,0,0]
        y_entry = [0,0,0,0]
        sleep(3.5)
        current_state_list.append(current_state)
        time_step_list.append(counter-1)

        action =  pick_action(current_state, actions, p, x_size, y_size)
        #action = get_action([('north',15),('east',35),('south',35),('west',15)])
        action_log.append(action)

        if action == 'north':
            y_entry[1] = 0
            x_entry[1] = 0
        elif action == 'east':
            y_entry[1] = 0
            x_entry[1] = 1
        elif action == 'south':
            y_entry[1] = 1
            x_entry[1] = 0
        elif action == 'west':
            y_entry[1] = 0
            x_entry[1] = 0

        blocks = sim.get_state()
        current_state = (blocks["agents"][0][0], blocks["agents"][0][1])

        prev_x_state = blocks["agents"][0][0]
        prev_y_state = blocks["agents"][0][1]
        x_entry[0] = blocks["agents"][0][0]
        y_entry[0] = blocks["agents"][0][1]

        prev_x_state_log.append(prev_x_state)
        prev_y_state_log.append(prev_y_state)

        sim.move_agent(0, action)

        blocks = sim.get_state()

        x_state = blocks["agents"][0][0]
        y_state = blocks["agents"][0][1]

        x_state_log.append(x_state)
        y_state_log.append(y_state)

        done = sim.update()

        if done:
            pygame.quit()
            sys.exit()

        response = get_response()
        #response = 0
        x_entry[2] = response
        y_entry[2] = response
        x_entry[3] = counter - 1
        y_entry[3] = counter - 1
        response_log.append(response)
        evaluation_list = response_log[:]

        x_entry_log.append(x_entry)
        y_entry_log.append(y_entry)

        x_denoise_log.append(denoise(x_entry_log,.3,.5,10))
        y_denoise_log.append(denoise(y_entry_log,.3,.5,10))

        #print 'original {}'.format(y_denoise_log[counter-1][3]) 
        #print 'origints {}'.format(y_denoise_log[counter-1][4])
        #print 'denoised {}'.format(y_denoise_log[counter-1][2])
        #print 'eval list pre: {}'.format(evaluation_list)
        
        for i,column in enumerate(y_denoise_log[counter-1][4]):
            for x,element in enumerate(column):
                if type(element) == list:
                    if element[0] != -1:
                        if element[0] != y_denoise_log[counter-1][2][i][x] and (action_log[element[1]] == 'north' or action_log[element[1]] == 'south'):
                            if evaluation_list[element[1]] == 1:
                                evaluation_list[element[1]] = 0
                            if evaluation_list[element[1]] == 0:
                                evaluation_list[element[1]] = 1


        for i,column in enumerate(x_denoise_log[counter-1][4]):
            for x,element in enumerate(column):
                if type(element) == list:
                    if element[0] != -1:                                
                        if element[0] != x_denoise_log[counter-1][2][i][x] and (action_log[element[1]] == 'east' or action_log[element[1]] == 'west'):
                            if evaluation_list[element[1]] == 1:
                                evaluation_list[element[1]] = 0
                            if evaluation_list[element[1]] == 0:
                                evaluation_list[element[1]] = 1
        
        #print 'eval list post: {}'.format(evaluation_list)
        
        #K = det_k(P,current_state,action,response)
        s = init_k(x_size, y_size)
        all_K = det_all_k(s,current_state_list,action_log,evaluation_list)
        
        p = init_probs(x_size, y_size)
        p = update_all_P(p,all_K)
        #print p
        #P = update_P(P,K)

        if action == 'east' and response == 1:
            error_log.append(False)
        elif action == 'south' and response == 1:            
            error_log.append(False)
        elif action == 'north' and response == 0:
            error_log.append(False)
        elif action == 'west' and response  == 0:
            error_log.append(False)
        else:
            error_log.append(True)

    x_state_logging = {'Previous_x_State': prev_x_state_log, 'Action': action_log, 
                'New_x_State': x_state_log, 'User_Response': response_log, 'Error': error_log}

    y_state_logging = {'Previous_y_State': prev_y_state_log, 'Action': action_log, 
                'New_y_State': y_state_log, 'User_Response': response_log, 'Error': error_log}

    x_denoise_logging = {'Maxflow': [i[0] for i in x_denoise_log],'Denoised_x_Image': [i[1] for i in x_denoise_log],
    'Final_Denoised_x_Image': [i[2] for i in x_denoise_log],'x_Image': [i[3] for i in x_denoise_log]}
    y_denoise_logging = {'Maxflow': [i[0] for i in y_denoise_log],'Denoised_y_Image': [i[1] for i in y_denoise_log],
    'Final_Denoised_y_Image': [i[2] for i in y_denoise_log],'y_Image': [i[3] for i in y_denoise_log]}

    x_merged_log = x_state_logging.copy()
    x_merged_log.update(x_denoise_logging)
    y_merged_log = y_state_logging.copy()
    y_merged_log.update(y_denoise_logging)

    return (x_merged_log, y_merged_log, response_log, evaluation_list)


if __name__=="__main__":

    counter = 0
    responded = False
    subscribed = False
    closed = False
    subscription = 0
    steps = int(sys.argv[1])

    log = start_simulation(10, 10, steps)

    log_x_df = pd.DataFrame()
    log_x_df = log_x_df.from_dict(log[0], orient='columns', dtype=None)
    log_y_df = pd.DataFrame()
    log_y_df = log_y_df.from_dict(log[1], orient='columns', dtype=None)
    log_x_df.to_pickle('log_x.p')
    log_x_df.to_pickle('log_y.p')
    print log[2]
    print log[3]