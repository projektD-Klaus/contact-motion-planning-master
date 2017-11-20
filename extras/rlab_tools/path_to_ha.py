#!/usr/bin/env python

import numpy as np
import math
import argparse
import tf.transformations as tra

import hatools.components as ha

def extrapolate_configuration(a, b, alpha = 1.5):
    return a + (b-a) * alpha

def transform_from_vector(a):
    return np.dot(tra.translation_matrix(a[:3]), tra.quaternion_matrix(a[3:]))

def extrapolate_transform(a, b, alpha = 1.5):
    T_a = transform_from_vector(a)
    T_b = transform_from_vector(b)
    
    T_delta = np.dot(tra.inverse_matrix(T_a), T_b)
    T_delta[:3, 3] = T_delta[:3, 3] * alpha
    return T_delta
    #return np.dot(T_a, T_delta)

def create_hybrid_automaton(roblib_trajectory):
    hybrid_automaton = ha.HybridAutomaton(current_control_mode='goto_0')
    # go through trajectory points and add control mode + switch
    points = roblib_trajectory.shape[0]
    in_contact = False
    for t in range(points):
        # add motion depending on contact change
        next_in_contact = (roblib_trajectory[t, 7] == 1)
        normal = roblib_trajectory[t, 8:11]
        normal_undefined = np.linalg.norm(normal) == 0
        
        ctrl_mode_name = 'goto_{}'.format(t)
        next_ctrl_mode_name = 'goto_{}'.format(t+1)
        ctrller_name = 'cntrl_{}'.format(t)
        
        mykp = np.array([300, 200, 150, 200, 10, 10, 5])
        mykv = np.array([2, 4, 2, 0.8, 0.2, 0.2, 0.15])
        #mykp = np.array([900, 2500, 600, 500, 50, 50, 8])
        #mykv = np.array([10, 20, 5, 2, 0.5, 0.5, 0.05])
        myv_max = np.array([0.1, 0.04])#np.array([0.25, 0.08])
        
        if absolute_motion:
            if t==0:
                print "(absolute goal)"
                rlab_traj = roblib_trajectory[:,:7].transpose()
                print rlab_traj
                ctrl_mode = ha.JointControlMode(rlab_traj, name = ctrl_mode_name, controller_name = ctrller_name)
                # that's important to avoid jerky behavior on the spot! --> it's actually the minimum_completion_time
                #ctrl_mode.controlset.controllers[0].properties['completion_times'] = '[1,1]1.0'
                ctrl_mode.controlset.controllers[0].properties['kp'] = mykp
                ctrl_mode.controlset.controllers[0].properties['kv'] = mykv
                ctrl_switch = ha.JointConfigurationSwitch(ctrl_mode_name, next_ctrl_mode_name, controller = ctrller_name, epsilon = str(math.radians(5.0)))
        
        elif (not in_contact or normal_undefined) and not next_in_contact:
        #if True:
            # free space motion
            print "(free space -> free space)"
            if t == 0:
                ctrl_mode = ha.JointControlMode(roblib_trajectory[t, :7], name = ctrl_mode_name, controller_name = ctrller_name)
                # that's important to avoid jerky behavior on the spot! --> it's actually the minimum_completion_time
                ctrl_mode.controlset.controllers[0].properties['completion_times'] = '[1,1]1.0'
                #ctrl_mode.controlset.controllers[0].properties['kp'] = mykp
                #ctrl_mode.controlset.controllers[0].properties['kv'] = mykv
                ctrl_switch = ha.JointConfigurationSwitch(ctrl_mode_name, next_ctrl_mode_name, controller = ctrller_name, epsilon = str(math.radians(5.0)))
            elif t == (points - 1): # last one
                goal = roblib_trajectory[t, :7] - roblib_trajectory[t-1, :7]
                ctrl_mode = ha.JointControlMode(goal, goal_is_relative='1', name = ctrl_mode_name, controller_name = ctrller_name)
                ctrl_switch = ha.JointConfigurationSwitch(ctrl_mode_name, next_ctrl_mode_name, controller = ctrller_name, epsilon = str(math.radians(5.0)), goal_is_relative = '1')
                #tmp = ha.ForceTorqueSwitch(ctrl_mode_name, next_ctrl_mode_name, goal = np.array([0., 0., 0., 0, 0, 0]), norm_weights = np.array([1, 1, 1, 0, 0, 0]), negate = '1', epsilon='1', jump_criterion = "NORM_L2", frame_id = '', goal_is_relative = '1')
                #ctrl_switch.add(tmp.con)
            else:
                goal = (roblib_trajectory[t, :7] - roblib_trajectory[t-1, :7]) * 2.
                ctrl_mode = ha.JointControlMode(goal, goal_is_relative='1', name = ctrl_mode_name, controller_name = ctrller_name)
                ctrl_switch = ha.JointConfigurationSwitch(ctrl_mode_name, next_ctrl_mode_name, name='TheSpecialOne', controller = ctrller_name, epsilon = str(math.radians(5.0)), goal_is_relative = '1')
                
            
        elif (not in_contact or normal_undefined) and next_in_contact:
            # move until contact
            print "(free space -> in contact)"
            if False:
                goal = extrapolate_transform(roblib_trajectory[t-1, -7:], roblib_trajectory[t, -7:], 1.2)
                ctrl_mode = ha.HTransformControlMode(goal, name = ctrl_mode_name, controller_name = ctrller_name, goal_is_relative='1')
                ctrl_mode.controlset.controllers[0].properties['v_max'] = myv_max
            
            goal = (roblib_trajectory[t, :7] - roblib_trajectory[t-1, :7]) * 2.
            #goal = extrapolate_configuration(roblib_trajectory[t-1, :7], roblib_trajectory[t, :7], 1.3)
            ctrl_mode = ha.JointControlMode(goal, goal_is_relative='1', name = ctrl_mode_name, controller_name = ctrller_name)
            #ctrl_mode.controlset.controllers[0].properties['kp'] = mykp
            #ctrl_mode.controlset.controllers[0].properties['kv'] = mykv
            ctrl_mode.controlset.controllers[0].properties['v_max'] = np.array([0.1]*7)
            
            ctrl_switch = ha.ForceTorqueSwitch(ctrl_mode_name, next_ctrl_mode_name, epsilon='4', goal = np.array([0., 0., 0., 0, 0, 0]), norm_weights = np.array([1, 1, 1, 0, 0, 0]), negate = '1', jump_criterion = "NORM_L2", frame_id = '', goal_is_relative = '1')
            time_switch = ha.TimeSwitch(ctrl_mode_name, next_ctrl_mode_name, duration = 1.0)
            ctrl_switch.add(time_switch.conditions[0])
            
        elif in_contact and next_in_contact:
            # move until contact changes
            print "(in contact -> in contact)"
            goal = extrapolate_configuration(roblib_trajectory[t-1, :7], roblib_trajectory[t, :7], 1.5)
            ctrl_mode = ha.JointControlMode(goal, name = ctrl_mode_name, controller_name = ctrller_name)
            ctrl_switch = ha.ForceTorqueSwitch(ctrl_mode_name, next_ctrl_mode_name, goal = np.array([0., 0., 0., 0, 0, 0]), norm_weights = np.array([1, 1, 1, 0, 0, 0]), negate = '1', epsilon='1', jump_criterion = "NORM_L2", frame_id = '', goal_is_relative = '1')
            
        elif in_contact and not next_in_contact:
            # move until no more contact
            print "(in contact -> free space) normal_defined:", ~normal_undefined
            ft_in_ee = tra.translation_matrix([0, 0, 0])
            
            goal = extrapolate_transform([0, 0, 0, 0, 0, 0, 1], roblib_trajectory[t, -7:], 1.0)
            delta = extrapolate_transform(roblib_trajectory[t-1, -7:], roblib_trajectory[t, -7:], 1.0)
            
            normal_in_ee = np.dot(tra.inverse_matrix(transform_from_vector(roblib_trajectory[t-1, -7:]))[:3, :3], normal)
            normal_in_ft = np.dot(ft_in_ee[:3, :3], normal_in_ee)
            
            print "Normal in world: ", normal # in world
            print "Normal in ee: ", normal_in_ee
            
            # project onto normal plane
            print delta[:3, 3]
            #print desired_delta
            dist = np.linalg.norm(delta[:3, 3])
            delta[:3, 3] = delta[:3, 3] * (0.03 / dist)
            print dist
            print normal_in_ft
            print delta
            #ctrl_mode = ha.HTransformControlMode(delta, name = ctrl_mode_name, controller_name = ctrller_name, goal_is_relative='1')
            #ctrl_mode = ha.InterpolatedHTransformImpedanceControlMode(goal, name = ctrl_mode_name, controller_name = ctrller_name, goal_is_relative='1')
            # three things needed:
            # 1) desired_displacement: delta vector in ee frame 
            # 2) force_gradient: normal vector in ee frame
            ctrl_mode = ha.ControlMode(name = ctrl_mode_name).set(ha.NakamuraControlSet().add(ha.ForceHTransformController(name = ctrller_name,
                    desired_distance = dist*1.2,
                    desired_displacement=tra.translation_matrix(delta[:3, 3]),
                    #force_gradient=tra.translation_matrix([0, 0, 0.005]),
                    force_gradient=tra.translation_matrix(-normal_in_ee * 0.0025),
                    desired_force_dimension=np.array(-normal_in_ft + [0, 0, 0]))))
            ctrl_mode.controlset.controllers[0].properties['desired_min_force'] = '-5'
            ctrl_mode.controlset.controllers[0].properties['desired_max_force'] = '-3'
            #ctrl_mode = ha.ControlMode(name = ctrl_mode_name).set(ha.NakamuraControlSet().add(ha.ForceHTransformController(name = ctrller_name, desired_distance = dist*1.2, desired_displacement=tra.translation_matrix(delta[:3, 3]), force_gradient=tra.translation_matrix(normal * 0.005), desired_force_dimension=np.array([0, 0, 1, 0, 0, 0]))))
            
            ctrl_mode.controlset.controllers[0].properties['v_max'] = np.array([0.01, 0.01])#myv_max
            
            ctrl_switch = ha.ForceTorqueSwitch(ctrl_mode_name, next_ctrl_mode_name, epsilon = '1.2', goal_is_relative = '0', negate = '0', goal = np.array([0., 0., 0., 0, 0, 0]), norm_weights = np.array([1, 1, 1, 0, 0, 0]), jump_criterion = "NORM_L1", frame_id = '')
            pose_condition = ha.JumpCondition('FrameDisplacementSensor', goal = np.zeros(3), controller = '', jump_criterion = 'NORM_L2', 
                epsilon = '{}'.format(dist*0.5), negate = '1', goal_is_relative = '1', frame_id = 'EE', norm_weights = np.ones(3))
            ctrl_switch.add(pose_condition)
            #del ctrl_switch.conditions[0]
        
        if t == (points - 1):
            hybrid_automaton.add([ctrl_mode])
        elif not absolute_motion:
            hybrid_automaton.add([ctrl_mode, ctrl_switch])
        
        in_contact = next_in_contact
    
    # add final gravity mode
    #hybrid_automaton.add(ha.GravityCompensationMode(name = 'goto_{}'.format(points)))
    #hybrid_automaton.add(ha.JointControlMode(np.zeros(7), name = 'goto_{}'.format(points), goal_is_relative='1'))
    
    return hybrid_automaton


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Turn csv path into hybrid automaton.', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    
    parser.add_argument('input', type=str, default='solutionpath_easy.csv', help='the csv file to parse')
    parser.add_argument('--absolute', action="store_true",
                        help='dont do relative motions')
    #parser.add_argument('--angle', type=float, default = 10.,
                        #help='angle of attack during sliding (in deg)')
    #parser.add_argument('--speed', type=float, default = 0.01,
                        #help='speed during sliding')
    #parser.add_argument('--inflation', type=float, default = 0.05,
                        #help='inflation of fingers during sliding')
    #parser.add_argument('--force', type=float, default = 4.,
                        #help='downward force during sliding')
    #parser.add_argument('--wallforce', type=float, default = -6.,
                        #help='wall force to finish sliding')
    #parser.add_argument('--anglesliding', type=float, default = -10.,
                        #help='angle during sliding to edge (in deg)')
    #parser.add_argument('--edgedistance', type=float, default = 1.,
                        #help='multiplicative factor for the jump condition for the distance to the edge')
    #parser.add_argument('--positionx', type=float, default = 0.,
                        #help='positional offset parallel to the constraint (e.g. wall)')
    #grasps = ["all", "edge_grasp", "wall_grasp", "surface_grasp"]
    #parser.add_argument('--grasp', choices=grasps, default=grasps[0],
                        #help='which grasp type to use, default: all')

    args = parser.parse_args()
    
    path = np.genfromtxt(args.input, skip_header=1)
    if args.absolute:
        absolute_motion=True
    else:
        absolute_motion=False
    
    # each timestep includes robot DOFs [floats], expectContact [int/bool], normal [3 floats], EE transform [7 floats]
    assert(path.shape[1] == 7+1+3+7)
    
    hybrid_automaton = create_hybrid_automaton(path)
    
    with open('output.txt', 'w') as f:
        f.write(hybrid_automaton.xml())
