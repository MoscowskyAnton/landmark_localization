#!/usr/bin/env python
# coding: utf-8

'''
simple test, where robot moves through landmarks field with constant linear and angular speed
'''

import numpy as np
import matplotlib.pyplot as plt
from landmark_localization.ll_hf2d import HF2D
from landmark_localization.ll_sdl2d import SDL2D
from landmark_localization.ll_amcl2d import AMCL2D
from landmark_localization.landmark_localization_core import substract_angles, plot_cov
import copy
import datetime
from landmark_localization.test_utils import *
from pathlib import Path
import argparse

if __name__ == '__main__':        
    
    parser = argparse.ArgumentParser()
    
    parser.add_argument('-HF', dest="hf", default=False, const=True, action='store_const')
    parser.add_argument('-AMCL', dest="amcl", default=False, const=True, action='store_const')
    parser.add_argument('-SDL_HF', dest="sdl_hf", default=False, const=True, action='store_const')
    parser.add_argument('-SDL_AMCL', dest="sdl_amcl", default=True, const=True, action='store_const')
    
    parser.add_argument('-PLOT_FIELD', dest="plot_field", default=False, const=True, action='store_const')
    
    parser.add_argument('-PLOT_STUFF', dest="plot_stuff", default=False, const=True, action='store_const')
    
    parser.add_argument('-can_measure_r', dest="can_measure_r", default=False, const=True, action='store_const')
    parser.add_argument('-can_measure_a', dest="can_measure_a", default=False, const=True, action='store_const')
    
    parser.add_argument('--steps', dest="steps", default=300, type=int)
    
    parser.add_argument('--sub_dir', dest="sub_dir", default="", type=str)
    
    parser.add_argument('--np_min_amcl', dest="np_min_amcl", default=1000, type=int)
    parser.add_argument('--np_max_amcl', dest="np_max_amcl", default=2000, type=int)
    
    parser.add_argument('--np_min_amcl_sdl', dest="np_min_amcl_sdl", default=100, type=int)
    parser.add_argument('--np_max_amcl_sdl', dest="np_max_amcl_sdl", default=200, type=int)
    
    parser.add_argument('--n_landmarks', dest="n_landmarks", default=10, type=int)
    
    parser.add_argument('--dt', dest="dt", default=3, type=float)
    
    args = parser.parse_args()
    
    test_params = {}
    
    HIST = args.hf
    AMCL = args.amcl
    SDL_HF = args.sdl_hf
    SDL_AMCL = args.sdl_amcl
    
    PLOT_FIELD = args.plot_field
    PLOT_STUFF = args.plot_stuff
    
    test_params['sim'] = {}
    test_params['sim']['dt'] = 3
    test_params['sim']['measure_freq'] = 0
    test_params['sim']['steps'] = args.steps
    test_params['sim']['step'] = 0
    
    test_params['field'] = {}
    test_params['field']['N_landmarks'] = args.n_landmarks
    test_params['field']['x_max'] = 10
    test_params['field']['y_max'] = 12
    
    test_params['robot'] = {}
    test_params['robot']['x'] = 0
    test_params['robot']['y'] = -9
    test_params['robot']['Y'] = 0
    test_params['robot']['v'] = 0.1
    test_params['robot']['w'] = 0.012
    test_params['robot']['sv'] = 0.01
    test_params['robot']['sw'] = 0.01
    
    test_params['sensor'] = {}
    test_params['sensor']['max_r'] = 10
    test_params['sensor']['max_a'] = np.pi
    test_params['sensor']['sr'] = 0.1
    test_params['sensor']['sa'] = 0.01
    
    test_params['sensor']['can_measure_r'] = args.can_measure_r
    test_params['sensor']['can_measure_a'] = args.can_measure_a
    
    landmarks = generate_landmarks(test_params)        
    #TODO: read/write params from file    
    
    real_history = []
    landmarks_num_history = []
    
    times = {'motion': [],
             'landmarks': [],
             'get_pose': []}
    timer = tick_tocker()
    '''
    localization methods
    '''    
    hf_params = {}
    hf_params['dims'] = {}
    hf_params['dims']['x'] = {}
    hf_params['dims']['x']['min'] = -test_params['field']['x_max']
    hf_params['dims']['x']['max'] = test_params['field']['x_max']
    hf_params['dims']['x']['d_res'] = 0.2
    hf_params['dims']['y'] = {}
    hf_params['dims']['y']['min'] = -test_params['field']['y_max']
    hf_params['dims']['y']['max'] = test_params['field']['y_max']
    hf_params['dims']['y']['d_res'] = 0.2
    hf_params['dims']['Y'] = {}
    hf_params['dims']['Y']['min'] = -np.pi
    hf_params['dims']['Y']['max'] = np.pi
    hf_params['dims']['Y']['d_res'] = 0.15
    hf_params['calc_type'] = "ADDITION"
    #hf_params['calc_type'] = "MULTIPLICATION" # NOTE: not works well
    hf_params['yaw_discount'] = 0.1
    hf_params['prev_step_weight'] = 0.5
    hf_params['motion_update_type'] = 'PREV_COV'
    hf_params['pose_calc_type'] = 'MAX'
    hf = HF2D(hf_params)
    hf_history = []
    hf_times = copy.deepcopy(times)
    
    amcl_params = {}
    amcl_params['NP'] = args.np_min_amcl
    amcl_params['NPmax'] = args.np_max_amcl
    amcl_params['alpha_slow'] = 0.05
    amcl_params['alpha_fast'] = 0.1
    amcl_params['dims'] = {}
    amcl_params['dims']['x'] = {}
    amcl_params['dims']['x']['min'] = -test_params['field']['x_max']
    amcl_params['dims']['x']['max'] = test_params['field']['x_max']
    amcl_params['dims']['y'] = {}
    amcl_params['dims']['y']['min'] = -test_params['field']['y_max']
    amcl_params['dims']['y']['max'] = test_params['field']['y_max']
    amcl_params['dims']['Y'] = {}
    amcl_params['dims']['Y']['min'] = -np.pi
    amcl_params['dims']['Y']['max'] = np.pi
    amcl_params['calc_type'] = "MULTIPLICATION"
    amcl = AMCL2D(amcl_params)
    amcl_history = []
    amcl_times = copy.deepcopy(times)
    
    sdl_hf_params = {}
    sdl_hf_params['verbose'] = False
    sdl_hf_params['dims'] = {}
    sdl_hf_params['dims']['x'] = {}
    sdl_hf_params['dims']['x']['min'] = -test_params['field']['x_max']
    sdl_hf_params['dims']['x']['max'] = test_params['field']['x_max']
    sdl_hf_params['dims']['y'] = {}
    sdl_hf_params['dims']['y']['min'] = -test_params['field']['y_max']
    sdl_hf_params['dims']['y']['max'] = test_params['field']['y_max']
    sdl_hf_params['dims']['Y'] = {}
    sdl_hf_params['dims']['Y']['min'] = -np.pi
    sdl_hf_params['dims']['Y']['max'] = np.pi
    sdl_hf_params['inner_method'] = 'hf'
    sdl_hf_params['inner_method_params'] = copy.deepcopy(hf_params)    
    #sdl_hf_params['inner_method_params']['pose_calc_type'] = 'SUM'
    sdl_hf = SDL2D(sdl_hf_params)
    sdl_hf_history = []
    sdl_hf_times = copy.deepcopy(times)
    
    sdl_amcl_params = {}
    sdl_amcl_params['verbose'] = False
    sdl_amcl_params['dims'] = {}
    sdl_amcl_params['dims']['x'] = {}
    sdl_amcl_params['dims']['x']['min'] = -test_params['field']['x_max']
    sdl_amcl_params['dims']['x']['max'] = test_params['field']['x_max']
    sdl_amcl_params['dims']['y'] = {}
    sdl_amcl_params['dims']['y']['min'] = -test_params['field']['y_max']
    sdl_amcl_params['dims']['y']['max'] = test_params['field']['y_max']
    sdl_amcl_params['dims']['Y'] = {}
    sdl_amcl_params['dims']['Y']['min'] = -np.pi
    sdl_amcl_params['dims']['Y']['max'] = np.pi    
    sdl_amcl_params['inner_method'] = 'amcl'
    sdl_amcl_params['inner_method_params'] = copy.deepcopy(amcl_params)
    # amcl params overwrite
    sdl_amcl_params['inner_method_params']['NP'] = args.np_min_amcl_sdl
    sdl_amcl_params['inner_method_params']['NPmax'] = args.np_max_amcl_sdl
    
    sdl_amcl = SDL2D(sdl_amcl_params)
    sdl_amcl_history = []
    sdl_amcl_times = copy.deepcopy(times)
    
    measure_freq_cnt = 0    
    if PLOT_FIELD:
        field_figure = plt.figure('field')           
    
    while test_params['sim']['step'] < test_params['sim']['steps']:        
        if PLOT_FIELD:
            plt.figure('field')
        test_params['sim']['step'] += 1
        
        # MOTION
        motion_params = do_motion(test_params)
        real_history.append([test_params['robot']['x'], test_params['robot']['y'], test_params['robot']['Y']])
        if HIST:
            timer.start()
            hf.motion_update(motion_params)
            hf_times['motion'].append(timer.end())
        if AMCL:
            timer.start()
            amcl.motion_update(motion_params)
            amcl_times['motion'].append(timer.end())
        if SDL_HF:
            timer.start()
            sdl_hf.motion_update(motion_params)
            sdl_hf_times['motion'].append(timer.end())
        if SDL_AMCL:
            timer.start()
            sdl_amcl.motion_update(motion_params)
            sdl_amcl_times['motion'].append(timer.end())
                
        # MEASURE
        landmarks_params = []
        if measure_freq_cnt == test_params['sim']['measure_freq'] or test_params['sim']['step'] == 1:            
            landmarks_params = do_measure(test_params, landmarks, calc_r = test_params['sensor']['can_measure_r'], calc_a = test_params['sensor']['can_measure_a'])
            if HIST:
                timer.start()
                hf.landmarks_update(landmarks_params)
                hf_times['landmarks'].append(timer.end())
            if AMCL:
                timer.start()
                amcl.landmarks_update(landmarks_params)
                amcl_times['landmarks'].append(timer.end())
            if SDL_HF:
                timer.start()
                sdl_hf.landmarks_update(landmarks_params)
                sdl_hf_times['landmarks'].append(timer.end())
            if SDL_AMCL:
                timer.start()
                sdl_amcl.landmarks_update(landmarks_params)
                sdl_amcl_times['landmarks'].append(timer.end())
            measure_freq_cnt = 0
        else:
            measure_freq_cnt += 1
        landmarks_num_history.append(len(landmarks_params))
                                
        # GET POSES
        if HIST:
            timer.start()
            hf_pose = hf.get_pose()
            hf_history.append(hf_pose)
            hf_times['get_pose'].append(timer.end())
        if AMCL:
            timer.start()
            amcl_pose = amcl.get_pose()        
            amcl_history.append(amcl_pose)
            amcl_times['get_pose'].append(timer.end())
        if SDL_HF:
            timer.start()
            sdl_hf_pose = sdl_hf.get_pose()
            sdl_hf_history.append(sdl_hf_pose)
            sdl_hf_times['get_pose'].append(timer.end())
        if SDL_AMCL:
            timer.start()
            sdl_amcl_pose = sdl_amcl.get_pose()
            sdl_amcl_history.append(sdl_amcl_pose)
            sdl_amcl_times['get_pose'].append(timer.end())
                
        # PLOT STUFF
        if PLOT_FIELD:
            plot_exp_base(field_figure, test_params, landmarks_params, landmarks)        
            plt.plot(np.array(real_history)[:,0], np.array(real_history)[:,1], '-r')
            
            if HIST:
                hf.plot()        
                plot_robot_pose(hf_pose[0], hf_pose[1], hf_pose[2], "green", "hf")                
                plt.plot(np.array(hf_history)[:,0], np.array(hf_history)[:,1], '-g')        
                plot_cov(plt.gca(), hf_pose, hf.get_cov(), color = 'g')
            
            if AMCL:
                amcl.plot()
                plot_robot_pose(amcl_pose[0], amcl_pose[1], amcl_pose[2], "blue", "amcl")                
                plt.plot(np.array(amcl_history)[:,0], np.array(amcl_history)[:,1], '-b')
                plot_cov(plt.gca(), amcl_pose, amcl.get_cov(), color = 'b')
                
            if SDL_HF:
                sdl_hf.plot()
                plot_robot_pose(sdl_hf_pose[0], sdl_hf_pose[1], sdl_hf_pose[2], "magenta", "sdl+hf")                
                plt.plot(np.array(sdl_hf_history)[:,0], np.array(sdl_hf_history)[:,1], '-m')        
                plot_cov(plt.gca(), sdl_hf_pose, sdl_hf.get_cov(), color = 'm')
                
            if SDL_AMCL:
                sdl_amcl.plot()
                plot_robot_pose(sdl_amcl_pose[0], sdl_amcl_pose[1], sdl_amcl_pose[2], "cyan", "sdl+amcl")
                plt.plot(np.array(sdl_amcl_history)[:,0], np.array(sdl_amcl_history)[:,1], '-c')
                #plot_cov(plt.gca(), sdl_amcl_pose, sdl_amcl.get_cov(), color = 'c')
            
            plt.legend()
            plt.grid()
            plt.pause(0.01)
        
        if PLOT_STUFF or (test_params['sim']['step'] == test_params['sim']['steps']):
            plt.figure('boxplot')
            plt.cla()        
            data = []
            labels = []
            colors = []
            if HIST:
                data.append(get_pose_errors(hf_history, real_history))
                labels.append('hf')
                colors.append('green')
            if AMCL:
                data.append(get_pose_errors(amcl_history, real_history))
                labels.append('amcl')
                colors.append('blue')
            if SDL_HF:
                data.append(get_pose_errors(sdl_hf_history, real_history))
                labels.append('sdl+hf')
                colors.append('magenta')
            if SDL_AMCL:
                data.append(get_pose_errors(sdl_amcl_history, real_history))
                labels.append('sdl+amcl')        
                colors.append('cyan')

            moving_boxplot(data, labels, colors, showfliers = False, current = (test_params['sim']['step'] < test_params['sim']['steps']))        
            plt.pause(0.01)
            
            plt.figure('time plot')
            plt.cla()        
            time_data = []
            time_labels = []
            colors = []
            if HIST:
                time_labels.append('hf')
                time_data.append(hf_times)
                colors.append('green')
            if AMCL:
                time_labels.append('amcl')
                time_data.append(amcl_times)
                colors.append('blue')
            if SDL_HF:
                time_labels.append('sdl_hf')
                time_data.append(sdl_hf_times)
                colors.append('magenta')
            if SDL_AMCL:
                time_labels.append('sdl_amcl')
                time_data.append(sdl_amcl_times)
                colors.append('cyan')
            
            time_plot(time_data, time_labels, colors)        
            
            plt.pause(0.01)
        
    # GET AND SAVE DATA
    folder_path = 'data/simple_circle_motion/'+args.sub_dir+'/'+datetime.datetime.now().strftime("%m-%d-%H-%M")
    Path(folder_path).mkdir(parents=True, exist_ok=True)
    
    raw_poses = []
    labels = []
    raw_poses.append(np.array(real_history))
    labels.append('gt')
    if HIST:                
        raw_poses.append(np.array(hf_history))
        labels.append('hf')
        save_dict_params(hf_params, folder_path + '/hf_params.yaml')
    if AMCL:
        raw_poses.append(np.array(amcl_history))
        labels.append('amcl')
        save_dict_params(amcl_params, folder_path + '/amcl_params.yaml')
    if SDL_HF:
        raw_poses.append(np.array(sdl_hf_history))
        labels.append('sdl_hf')
        save_dict_params(sdl_hf_params, folder_path + '/sdl_hf_params.yaml')
    if SDL_AMCL:
        raw_poses.append(np.array(sdl_amcl_history))
        labels.append('sdl_amcl')
        save_dict_params(sdl_amcl_params, folder_path + '/sdl_amcl_params.yaml')
        
    save_poses_raw(raw_poses, labels, folder_path+'/raw_poses.csv', landmarks_num_history)    
    save_time_raw(time_data, time_labels, folder_path+'/raw_time.csv')
    
    plt.figure('time plot')
    plt.savefig(folder_path + '/time_plot.png')
    plt.figure('boxplot')
    plt.savefig(folder_path + '/pose_error.png')
    
    save_dict_params(test_params, folder_path + '/test_params.yaml')
    save_dict_params(landmarks, folder_path + '/landmarks.yaml')
        
    
    
