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
from action_selection.action_selection import init_probs, pick_action, det_k, update_P, det_all_k, init_k, update_all_P, get_max, get_target, manhattan_dist
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

    if len(alpha_signal) == 16:
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

def start_simulation(x_size, y_size, steps,param = 0.25,beta = 0.5):
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
    current_state_list.append(current_state)
    
    actions = ['east','west','north','south']

    for i in range(0, steps):
        counter = i+1
        x_entry = [0,0,0,0]
        y_entry = [0,0,0,0]
        sleep(4.5)
        blocks = sim.get_state()
         
        time_step_list.append(counter-1)
        action  =  pick_action(current_state, actions, p, x_size, y_size)
        if action == 'stay':
            break
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

        S_max = get_max(p)
        S_target = get_target(current_state,S_max)

        for j in range(y_size):
            print '\n'
            print '\t',
            for i in range(x_size):
                print "%.3f" % p[(i,j)],
                print '\t',
            for i in range(x_size):
                if (i,j) == current_state:
                    print " * ",
                elif (i,j) == S_target:
                    print " X ",
                elif (i,j) in S_max:
                    print " O ",
                else:
                    print " _ ",
           
        print '\n'

        sim.move_agent(0, action)
        prev_x_state = blocks["agents"][0][0]
        prev_y_state = blocks["agents"][0][1]
        x_entry[0] = blocks["agents"][0][0]
        y_entry[0] = blocks["agents"][0][1]

        prev_x_state_log.append(prev_x_state)
        prev_y_state_log.append(prev_y_state)

        done = sim.update()
        blocks = sim.get_state()
        current_state = blocks["agents"][0]
        current_state_list.append(current_state)
        x_state = blocks["agents"][0][0]
        y_state = blocks["agents"][0][1]

        x_state_log.append(x_state)
        y_state_log.append(y_state)  

        if done:
            pygame.quit()
            sys.exit()

        print 'Step {} of {}'.format(counter,steps)
        response = get_response()
        print response
        x_entry[2] = response
        y_entry[2] = response
        x_entry[3] = counter - 1
        y_entry[3] = counter - 1
        response_log.append(response)
        evaluation_list = response_log[:]

        x_entry_log.append(x_entry)
        y_entry_log.append(y_entry)

        x_denoise_log.append(denoise(x_entry_log,param,beta,5))
        y_denoise_log.append(denoise(y_entry_log,param,beta,5))

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
        all_K = det_all_k(s,current_state_list[:-1],action_log,evaluation_list)
        
        p = init_probs(x_size, y_size)
        p = update_all_P(p,all_K)

        #print p
        #P = update_P(P,K)
        #print manhattan_dist(current_state_list[-1],(5,5))

        if manhattan_dist(current_state_list[-2],(5,5)) < manhattan_dist(current_state_list[-1],(5,5)) and response == 1:
            error_log.append(True)
        elif manhattan_dist(current_state_list[-2],(5,5)) >= manhattan_dist(current_state_list[-1],(5,5)) and response == 0:            
            error_log.append(True)
        else:
            error_log.append(False)

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

def start_offline_simulation(current_state_list_offline,action_offline_list,prev_x_state_list_offline,prev_y_state_list_offline, response_list_offline, current_x_state_list_offline, current_y_state_list_offline, param, beta, x_size=10, y_size=10):

    "this function doesn't work"

    global closed, counter
    #rospy.init_node('gridworld', anonymous=True)
    #start_2D_grid(initial_x_state, initial_y_state, num_x_states, num_y_states)
    #sim = Simulation('/home/sahabi/mo/lib/python/gridsim/config.txt','/home/sahabi/mo/lib/python/gridsim/matrix.txt')
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
    #done = sim.update()
    p = init_probs(x_size, y_size)
    s = init_k(x_size, y_size)
    current_state = (0, 0)
    current_state_list.append(current_state)
    actions = ['east','west','north','south']

    for offline in range(0, len(action_offline_list)):
        counter = offline+1
        x_entry = [0,0,0,0]
        y_entry = [0,0,0,0]
    #    sleep(4.5)
    #    blocks = sim.get_state()
         
        time_step_list.append(counter-1)
        action  =  action_offline_list[offline]
        if action == 'stay':
            break
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

        S_max = get_max(p)
        S_target = get_target(current_state,S_max)
        print current_state
        #sleep(0.5)

        for j in range(y_size):
            print '\n'
            print '\t',
            for i in range(x_size):
                print "%.3f" % p[(i,j)],
                print '\t',
            for i in range(x_size):
                if (i,j) == current_state:
                    print " * ",
                elif (i,j) == S_target:
                    print " X ",
                elif (i,j) in S_max:
                    print " O ",
                else:
                    print " _ ",
           
        print '\n'

    #    sim.move_agent(0, action)
        prev_x_state = prev_x_state_list_offline[offline]
        prev_y_state = prev_y_state_list_offline[offline]
        x_entry[0] = prev_x_state_list_offline[offline]
        y_entry[0] = prev_y_state_list_offline[offline]

        prev_x_state_log.append(prev_x_state)
        prev_y_state_log.append(prev_y_state)

        current_state = current_state_list_offline[offline]
        current_state_list.append(current_state)
        x_state = current_x_state_list_offline[offline]
        y_state = current_y_state_list_offline[offline]

        x_state_log.append(x_state)
        y_state_log.append(y_state)

        print 'Step {} of {}'.format(counter,len(action_offline_list))
    #    response = get_response()
        response = response_list_offline[offline]
        x_entry[2] = response
        y_entry[2] = response
        x_entry[3] = counter - 1
        y_entry[3] = counter - 1
        response_log.append(response)
        evaluation_list = response_log[:]

        x_entry_log.append(x_entry)
        y_entry_log.append(y_entry)

        x_denoise_log.append(denoise(x_entry_log,param,beta,10))
        y_denoise_log.append(denoise(y_entry_log,param,beta,10))

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
        all_K = det_all_k(s,current_state_list[:-1],action_log,evaluation_list)
        
        p = init_probs(x_size, y_size)
        p = update_all_P(p,all_K)

        #print p
        #P = update_P(P,K)
        #print manhattan_dist(current_state_list[-1],(5,5))

        if manhattan_dist(current_state_list[-2],(5,5)) < manhattan_dist(current_state_list[-1],(5,5)) and response == 1:
            error_log.append(True)
        elif manhattan_dist(current_state_list[-2],(5,5)) >= manhattan_dist(current_state_list[-1],(5,5)) and response == 0:            
            error_log.append(True)
        else:
            error_log.append(False)
    print prev_x_state_log

    sleep(0.5)

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

    # param_list = [.1,.15,.20,.25,.30]
    # beta_list = [.5,.6,.7,.8,.9,1]

    # x_log9 = pd.read_pickle('/home/sahabi/log_x9.p')
    # y_log9 = pd.read_pickle('/home/sahabi/log_y9.p')
    # action_offline_list = []
    # prev_x_state_list_offline = []
    # prev_y_state_list_offline = []
    # current_state_list_offline = []
    # response_list_offline = []
    # current_x_state_list_offline = []
    # current_y_state_list_offline = []
    # #param = param
    # #beta = beta

    # for row in range(0,len(y_log9)):
    #     j = y_log9.loc[row, 'Previous_y_State']
    #     i = x_log9.loc[row, 'Previous_x_State']
    #     j_c = y_log9.loc[row, 'New_y_State']
    #     i_c = x_log9.loc[row, 'New_x_State']
    #     response = y_log9.loc[row, 'User_Response']
    #     action = x_log9.loc[row, 'Action']
    #     prev_x_state_list_offline.append(i)
    #     prev_y_state_list_offline.append(j)
    #     current_x_state_list_offline.append(i_c)
    #     current_y_state_list_offline.append(j_c)
    #     action_offline_list.append(action) 
    #     response_list_offline.append(response)
    #     current_state_list_offline.append((i,j))


    # for param in param_list:
    #     for beta in beta_list:

    #         log = start_offline_simulation(current_state_list_offline,action_offline_list,prev_x_state_list_offline,prev_y_state_list_offline, response_list_offline, current_x_state_list_offline, current_y_state_list_offline, param, beta)
    #         log_x_df = pd.DataFrame()
    #         log_x_df = log_x_df.from_dict(log[0], orient='columns', dtype=None)

    #         log_y_df = pd.DataFrame()
    #         log_y_df = log_y_df.from_dict(log[1], orient='columns', dtype=None)

    #         log_x_df.to_pickle('log_offline_x_err_{}_beta_{}.p'.format(param,beta))

    #         log_y_df.to_pickle('log_offline_y_err_{}_beta_{}.p'.format(param,beta))
    # log_x_df.to_pickle('log_offline_x_err_{}_beta_{}.p'.format(param,beta))

    # log_y_df.to_pickle('log_offline_y_err_{}_beta_{}.p'.format(param,beta))
    param = .25
    beta = .5

    steps = int(sys.argv[1])
    print 'log_x10.p'
    log = start_simulation(5, 5, steps, param, beta)

    log_x_df = pd.DataFrame()
    log_x_df = log_x_df.from_dict(log[0], orient='columns', dtype=None)

    log_y_df = pd.DataFrame()
    log_y_df = log_y_df.from_dict(log[1], orient='columns', dtype=None)

    log_x_df.to_pickle('log_x2_5x5_err_{}_beta_{}.p'.format(param,beta))

    log_y_df.to_pickle('log_y2_5x5_err_{}_beta_{}.p'.format(param,beta))

    print log[2]
    print log[3]