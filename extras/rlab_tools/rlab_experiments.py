#!/usr/bin/env python

import pxssh
from subprocess import check_call
import time
import argparse
import numpy as np
import time
import logging
import glob
import os

import path_to_ha

import rospy
from hybrid_automaton_msgs import srv

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Connect via ssh to start simulation and start rosbag record locally.', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    
    parser.add_argument('input', type=str, default='solutionpaths_particles/', help='the directory that contains the csv files that contain the policies')
    parser.add_argument('--server', type=str, default='bottom-2.robotics.tu-berlin.de', help='the ssh server')
    parser.add_argument('--usr', type=str, default='Robotics', help='the username for the ssh connection')
    parser.add_argument('--pwd', type=str, default='', help='the password for the ssh connection')
    parser.add_argument('--trials', type=int, default=10, help='number of experimental trials')
    parser.add_argument('--duration', type=int, default=60, help='maximum duration of one experiment in seconds')
    args = parser.parse_args()
    
    logging.basicConfig(filename='rlabexperiment.log', level=logging.DEBUG, filemode='w')
    
    print "Don't forget! roslaunch rbo_wam_description rbo_wam_state_publisher.launch tf_prefix:=truth joint_states_topic:=/joint_states_truth"
    
    policies = glob.glob(args.input + '*.csv')
    
    s = pxssh.pxssh()
    if not s.login(args.server, args.usr, args.pwd):
        print "SSH session failed on login."
        print str(s)
    else:
        print "SSH session login successful"
        s.sendline ('cd C:/rswin/bin')
        s.prompt()
        
        for j, policy in enumerate(policies):
            logging.info('Policy %i/%i  (%s)' % (j, len(policies), policy))
            
            # load solution path
            path = np.genfromtxt(policy, skip_header=1)
            # each timestep includes robot DOFs [floats], expectContact [int/bool], normal [3 floats], EE transform [7 floats]
            assert(path.shape[1] == 7+1+3+7)
            
            # turn into ha
            errorparams = (0.02, 0.02)
            init = ' '.join([str(x) for x in path[0,:7]])
            myha = path_to_ha.create_hybrid_automaton(path)
            
            for i in range(args.trials): # for all experiments
                errors_const  = ' '.join([str(x) for x in np.random.normal(0, errorparams[0], 7)]) 
                errors_linear = ' '.join([str(x) for x in np.random.normal(0, errorparams[1], 7)])
                
                cmd = './ControllerManagerApp.exe --simulation --nogui --q0 {} --joint_error_params {} {}'.format(init, errors_const, errors_linear)
                print "Trial No. %i/%i -- %s" % (i, args.trials, cmd)
                
                logging.info("PARAMS %i %s %s" % (i, errors_const, errors_linear))
                logging.info(cmd)
                
                s.sendline(cmd)
                
                time.sleep(2)
                
                rospy.wait_for_service('update_hybrid_automaton')
                call_ha = rospy.ServiceProxy('update_hybrid_automaton', srv.UpdateHybridAutomaton)
                call_ha(myha.xml())
                
                check_call(["rosbag", "record", "--duration=" + str(args.duration), "-orun_%02i_%04i_%s" % (j, i, os.path.basename(policy)[:-4]),
                            "/joint_states", "/joint_states_truth", "/ee_velocity", "/ft_notool", "/ft_raw", "/ft_sensor", "/ham_state", "/tf"])
                
                time.sleep(args.duration + 1)
                
                s.sendcontrol('c')
                s.prompt()
                logging.info(s.before)
        
        s.logout()

