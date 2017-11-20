import bag2panda

def extract_values_from_filename(name, prefix='error', suffix='_xxxx-xx-xx-xx-xx-xx.bag', func=float):
    return [func(x) for x in name[len(prefix):-len(suffix)].split('_')]

import openravepy
env = openravepy.Environment()
env.Load('robots/barrettwam.robot.xml')
robot = env.GetRobots()[0]
manip = robot.GetManipulators()[0]
manip.GetEndEffectorTransform()

def get_ee_path(dof_values, robot, manip):
    ee_path = []
    for j in dof_values:
        robot.SetDOFValues(j, manip.GetArmIndices())
        ee_path.append(manip.GetEndEffectorTransform())
    return ee_path

def get_ee_path_position(ee_path):
    ee_path_position = []
    for ee in ee_path:
        ee_path_position.append(tra.translation_from_matrix(ee))
    return np.array(ee_path_position)

def get_list_of_files_per_particles(all_files):
    res = {}
    all_files = sorted(all_files)
    for f in all_files:
        tmp = extract_values_from_filename(f, 'run_', func=str)
        num_particles = int(tmp[2][len('solutionpath'):])
        if num_particles not in res:
            res[num_particles] = [f]
        else:
            res[num_particles].append(f)
    return res

def get_shit(files, robot, manip):
    ee_paths = []
    for f in files:
        j = bag2panda.bag2panda(f, ['/joint_states_truth']).values
        ee = get_ee_path_position(get_ee_path(j, robot, manip))
        ee_paths.append(ee)
    return ee_paths

def get_last_poses(ee_paths):
    res = []
    for p in ee_paths:
        res.append(p[-1])
    return np.array(res)

def mean_std_to_goal(files, goal, robot, manip):
    mean_std = {}
    for k, v in files.iteritems():
        tmp = get_shit(v, robot, manip)
        tmp2 = get_last_poses(tmp)
        mean_std[k] = [np.mean(np.linalg.norm(tmp2-goal, axis=1)),
                        np.std(np.linalg.norm(tmp2-goal, axis=1))]
    return mean_std

from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure(); ax = fig.add_subplot(111, projection='3d')
