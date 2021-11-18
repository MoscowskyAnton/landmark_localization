#!/usr/bin/env python
# coding: utf-8

'''
simple test with one robot step
'''

import numpy as np
import matplotlib.pyplot as plt
import copy
from landmark_localization import test_utils
from landmark_localization.landmark_localization_core import substract_angles

def draw_landmarks_as_robot(robot_pose, landmarks_params, marker = 'b*'):
    for l in landmarks_params:
        plt.plot(robot_pose['x'] + l['r'] * np.cos(substract_angles(l['a'],-robot_pose['Y'])),
                 robot_pose['y'] + l['r'] * np.sin(substract_angles(l['a'],-robot_pose['Y'])), marker)
    

if __name__ == '__main__':
    
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    ax = plt.gca()    
    ax.set_aspect('equal', 'box')
    
    test_params = {}
    test_params['sensor'] = {}
    test_params['sensor']['max_r'] = 100
    test_params['sensor']['max_a'] = np.pi
    test_params['sensor']['sr'] = 0.1
    test_params['sensor']['sa'] = 0.01
    
    test_params['robot'] = {}
    test_params['robot']['x'] = 0
    test_params['robot']['y'] = 0
    test_params['robot']['Y'] = 0
    test_params['robot']['v'] = 1
    test_params['robot']['w'] = 1
    test_params['robot']['sv'] = 0.1
    test_params['robot']['sw'] = 0.1
    
    test_params['sim'] = {'dt':1}
               
    landmarks = [{'x':5,'y':5},{'x':5,'y':-5}]
            
    first_pose = copy.deepcopy(test_params['robot'])
    
    lp1 = test_utils.do_measure(test_params, landmarks)
    #print(lp1)
    draw_landmarks_as_robot(first_pose, lp1, '*m')
    
    mp = test_utils.do_motion(test_params)
    
    second_pose_gt = copy.deepcopy(test_params['robot'])
    
    second_pose_noisy = {}
    second_pose_noisy['Y'] = first_pose['Y'] + mp['wY'] * test_params['sim']['dt']
    second_pose_noisy['x'] = first_pose['x'] + np.cos(second_pose_noisy['Y']) * mp['vx'] * test_params['sim']['dt']
    second_pose_noisy['y'] = first_pose['x'] + np.sin(second_pose_noisy['Y']) * mp['vx'] * test_params['sim']['dt']
    
    lp2 = test_utils.do_measure(test_params, landmarks)
    draw_landmarks_as_robot(second_pose_noisy, lp2)
    
    for l in lp2:
        plt.plot()
        
    
    
    test_utils.plot_robot_pose(first_pose['x'], first_pose['y'], first_pose['Y'], color = "red")
    test_utils.plot_robot_pose(second_pose_gt['x'], second_pose_gt['y'], second_pose_gt['Y'], color = "red")
    
    test_utils.plot_robot_pose(second_pose_noisy['x'], second_pose_noisy['y'], second_pose_noisy['Y'], color = "blue")
    
    for l in landmarks:
        plt.plot(l['x'], l['y'],"*r")
        
    
    
    plt.show()
    
    
