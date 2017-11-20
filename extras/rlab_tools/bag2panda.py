import pandas as pd
import numpy as np
import os
import rosbag
from scipy.signal import argrelextrema

def bag2panda(bag_filename, topics = ['/joint_states', '/ft_sensor', '/pressures', '/tf'], tf_child_frame_id = 'ee'):
    dataframes = {}
    with rosbag.Bag(bag_filename, 'r') as bag:
        types_and_topics = bag.get_type_and_topic_info()
        print types_and_topics
        
        for topic in topics:
            data = []
            
            for _, msg, t in rosbag.Bag(bag_filename).read_messages(topic):
                if types_and_topics[1][topic][0] == 'sensor_msgs/JointState':
                    data.append([t.to_nsec()] + list(msg.position)) # + msg.velocity + msg.effort)
                elif types_and_topics[1][topic][0] == 'std_msgs/Float64MultiArray':
                    data.append([t.to_nsec()] + list(msg.data))
                elif types_and_topics[1][topic][0] == 'tf2_msgs/TFMessage':
                    for tf in msg.transforms:
                        if tf.child_frame_id == tf_child_frame_id:
                            print tf.transform.translation
                            data.append([t.to_nsec()] + [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
            
            if (len(data) > 0):
                data = np.array(data)
                dataframes[topic] = pd.DataFrame(data=data[:,1:], index=pd.to_datetime(data[:,0]))
                dataframes[topic].columns = pd.MultiIndex.from_product([[topic], range(data.shape[1] - 1)])
                dataframes[topic].drop_duplicates(inplace=True)
            else:
                print "Could not find any data for topic ", topic
        
    return pd.concat(dataframes.values(), axis=1)

def extract_timestamps(data, template):
    res = []
    
    column = '/joint_states'
    
    for i in range(data.shape[0] - template.shape[0]):
        res.append(np.linalg.norm((data[column] - template[column].tshift(i)).fillna(0)))
    
    idx = argrelextrema(np.array(res), np.less)[0]
    
    return (res, idx, data.index[idx])

def convert(bag_filenames, topics = ['/joint_states', '/ft_sensor', '/pressures', '/tf']):
    for f in bag_filenames:
        data = bag2panda(f, topics)
        data.to_pickle(''.join(os.path.splitext(f)[:-1]) + '.pkl')

def store_pandas(prefix = 'run', indices = range(10), indexstring = "%02i"):
    for i in indices:
        name = indexstring % i
        data = bag2panda(prefix + name + '.bag')
        data.to_pickle(prefix + name + '.pkl')

def load_pandas(prefix = 'run', indices = range(10), indexstring = "%02i"):
    pandas = []
    
    for i in indices:
        name = indexstring % i
        pandas.append(pd.read_pickle(prefix + name + '.pkl'))
    
    return pandas
