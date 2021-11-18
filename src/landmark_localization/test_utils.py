#!/usr/bin/env python
# coding: utf-8
import numpy as np
import matplotlib.pyplot as plt
from landmark_localization.landmark_localization_core import substract_angles
import time
import pandas as pd
import yaml
# =====================
#   P L O T   D A T A
# =====================
def boxplot( data, ax, labels, colors, current = True, rotation  = 0, showfliers = True):
    for i, datum in enumerate(data):
        bp = ax.boxplot(datum, sym='.'+colors[i][0], positions = range(i,i+1), zorder = 1, showfliers = showfliers)
        plt.setp(bp['medians'], color=colors[i], linewidth=3)    
        if current and len(datum) > 0:
            plt.plot(i, datum[-1], "o", color = 'black', zorder = 2)
            plt.plot(i, datum[-1], ".", color = colors[i], zorder = 3)
    plt.xticks(range(0,len(data)), labels, rotation = rotation)    
    
def moving_boxplot(data, labels, colors, showfliers = True, current = True):
    data_labels = list(zip(data, labels, colors))            
    data_labels = sorted(data_labels, key=lambda item: np.median(item[0]) )                
    data_labels = list(zip(*data_labels))
    
    boxplot(data_labels[0], plt.gca(), data_labels[1], data_labels[2], showfliers = showfliers, current = current)
    plt.grid()  
    plt.title("Pose error distribution")
    plt.ylabel('Pose error, m')
    
def time_plot(time_data, labels, colors):
    for i, method_data in enumerate(time_data):
        total = 0
        for step, data in method_data.items():
            total += np.mean(data)
            #plt.plot(i, total, 'o', color = colors[i])
            plt.errorbar(i, total, yerr=np.std(data),  color = colors[i], fmt='.')#,uplims=True, lolims=True)
            
    plt.xticks(range(len(time_data)), labels)
    plt.grid()
    plt.title('Time distribution')
    plt.ylabel('Ellapsed time, s')
    

# =====================
#   P L O T   S I M
# =====================
def plot_robot_pose(x, y, Y, color, label = None, radius = None):
    plt.plot(x, y, "o", color = 'black', zorder = 6)
    plt.plot(x, y, ".", color = color, zorder = 7)
    plt.plot([x, x + np.cos(Y)], [y, y + np.sin(Y)], "-", color = 'black', zorder = 6, lw = 3)
    plt.plot([x, x + np.cos(Y)], [y, y + np.sin(Y)], "-", color = color, label = label, zorder = 7)
    if not radius is None:
        circle = plt.Circle((x, y), radius, fill = False, color = color, ls = ":")
        ax = plt.gca()
        ax.add_patch(circle)
    plt.legend()    

def plot_exp_base(figure, test_params, landmarks_params, landmarks, padding = 1):
    figure.clf()
    plt.xlim(-test_params['field']['x_max'] - padding, test_params['field']['x_max'] + padding)
    plt.ylim(-test_params['field']['y_max'] - padding, test_params['field']['y_max'] + padding)
    ax = plt.gca()    
    ax.set_aspect('equal', 'box')
    plt.title("Sim step {} of {}, dt = {}".format(test_params['sim']['step'], test_params['sim']['steps'], test_params['sim']['dt']))    
    
    for landmark_param in landmarks_params:
        plt.plot(landmark_param['x'], landmark_param['y'], 'ro')
    
    for landmark in landmarks:
        plt.plot(landmark['x'], landmark['y'], 'k.')
        
    plot_robot_pose(test_params['robot']['x'], test_params['robot']['y'], test_params['robot']['Y'], 'red', 'real_pose', radius = test_params['sensor']['max_r']) 
    
# ===========
#   S I M
# ===========

'''
empty_quad - quadrants where are no landmars are placed   
 2 | 1
---+---
 3 | 0
'''
def generate_landmarks(test_params, empty_quad = []):
    landmarks = []
    for i in range(test_params['field']['N_landmarks']):
        landmark = {}
        while True:
            landmark['x'] = np.random.uniform(-test_params['field']['x_max'], test_params['field']['x_max'])
            landmark['y'] = np.random.uniform(-test_params['field']['y_max'], test_params['field']['y_max'])
            
            if landmark['x'] > 0:
                if landmark['y'] < 0:
                    landmark_quad = 0
                else:
                    landmark_quad = 1
            else:
                if landmark['y'] < 0:
                    landmark_quad = 3
                else:
                    landmark_quad = 2
                                        
            if not landmark_quad in empty_quad:
                break            
        landmarks.append(landmark)
    return landmarks  

def do_motion(test_params):
    # real
    test_params['robot']['Y'] += test_params['sim']['dt'] * test_params['robot']['w']
    test_params['robot']['Y'] = (test_params['robot']['Y'] + np.pi)%(2*np.pi) - np.pi
    test_params['robot']['x'] += (test_params['sim']['dt'] * test_params['robot']['v']) * np.cos(test_params['robot']['Y'])
    test_params['robot']['y'] += (test_params['sim']['dt'] * test_params['robot']['v']) * np.sin(test_params['robot']['Y'])
    
    # noisy
    motion_params = {}
    motion_params['dt'] = test_params['sim']['dt']    
    motion_params['wY'] = np.random.normal(test_params['robot']['w'], test_params['robot']['sw'])
    motion_params['swY'] = test_params['robot']['sw']    
    motion_params['vx'] = np.random.normal(test_params['robot']['v'], test_params['robot']['sv'])
    motion_params['svx'] = test_params['robot']['sv']
    
    return motion_params 

def do_measure(test_params, landmarks, calc_r = True, calc_a = True):
    landmarks_params = []
    for landmark in landmarks:
        dx = test_params['robot']['x'] - landmark['x']
        dy = test_params['robot']['y'] - landmark['y']
        r = np.hypot(dx, dy)
        a_glob = np.arctan2(dy, dx)
        a = substract_angles(a_glob, test_params['robot']['Y'])
        
        a_glob_ = np.arctan2(-dy, -dx)
        a_ = substract_angles(a_glob_, test_params['robot']['Y'])
                
        if r <= test_params['sensor']['max_r'] and np.abs(a_) <= test_params['sensor']['max_a']:
            landmark_param = {}
            landmark_param['x'] = landmark['x']
            landmark_param['y'] = landmark['y']
            if calc_r:                
                landmark_param['r'] = np.random.normal(r, test_params['sensor']['sr'])
                landmark_param['sr'] = test_params['sensor']['sr']
            if calc_a:
                landmark_param['a'] = np.random.normal(a_, test_params['sensor']['sa'])
                landmark_param['sa'] = test_params['sensor']['sa']                    
            landmarks_params.append(landmark_param)
    return landmarks_params 

    
# ==============
# V A R I O U S
# ==============
def get_pose_errors(data, real):
    d = np.array(data)
    r = np.array(real)
    dr = np.hypot(d[:,0] - r[:,0], d[:,1] - r[:,1])
    #print(dr)
    dr = dr[~np.isnan(dr)]
    #print(dr)
    return dr   

class tick_tocker(object):
    def __init__(self):
        self.start_t = None
        
    def start(self):
        self.start_t = time.time()
        
    def end(self):
        return time.time() - self.start_t
    
# ==================
# D A T A   S A V E
# ==================
def save_poses_raw(data, labels, full_name, landmarks = None):
    data_unravel = []
    labels_unravel = []
    for i, d in enumerate(data):
        data_unravel += [d[:,0], d[:,1], d[:,2]]
        labels_unravel += [f'{labels[i]}_x', f'{labels[i]}_y', f'{labels[i]}_a']    
    if not landmarks is None:
        data_unravel.append(landmarks)
        labels_unravel.append('l_num')
    df = pd.DataFrame(np.array(data_unravel).T, columns = labels_unravel)
    df.to_csv(full_name)

def save_time_raw(data, labels, full_name):
    data_unravel = []
    labels_unravel = []
    for i, d in enumerate(data):
        for name, rd in d.items():
            data_unravel.append(rd)
            labels_unravel.append(f'{labels[i]}_{name}')    
    df = pd.DataFrame(np.array(data_unravel).T, columns = labels_unravel)
    df.to_csv(full_name)    
    
def save_dict_params(params, full_name):
    f = open(full_name, 'w')
    yaml.dump(params, f)
    
