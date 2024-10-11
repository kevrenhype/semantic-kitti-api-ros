import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud
import struct
import yaml

# 初始化节点
rospy.init_node('point_cloud_publisher')

label_array = [[] for _ in range(20)]
semantic_array = [[]for _ in range(20)]

with open("config/semantic-kitti-all.yaml", 'r', encoding='utf-8') as file:
    yaml_data = yaml.safe_load(file)

points = np.array([])
sem_label = np.array([])

mapping = {}
for key, value in yaml_data.items():
    mapping[key] = value
# 整合标签 
def tidyLabel(sem_label):
    for i, l in enumerate(sem_label):
        if l == 252:
            sem_label[i] = 10
        elif l == 253:
            sem_label[i] = 31
        elif l == 254:
            sem_label[i]= 30
        elif l == 255:
            sem_label[i]= 32
        elif l == 256:
            sem_label[i]=16
        elif l == 257:
            sem_label[i]=13
        elif l == 258:
            sem_label[i]= 18
        elif l == 259:
            sem_label[i]= 20
    return sem_label
        

def publish_point_cloud(points,sem_label):
    # start_time = rospy.Time.now()
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
          PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]
    coherent_header = rospy.Header()
    coherent_header.stamp = rospy.Time.now()
    coherent_header.frame_id = 'base_link'
    
    intensityList = []
    for i in range(len(semantic_array)):
        semantic_array[i] = []
        intensityList.append(mapping["learning_color_map"][i][::-1][0] *0.299 + mapping["learning_color_map"][i][::-1][1] *0.587 + mapping["learning_color_map"][i][::-1][2] *0.114)

    # print("运行时间0毫秒:", rospy.Time.now().to_sec()*1000 - start_time.to_sec()*1000)
    for l,p in zip(sem_label,points):
        mapLabel = mapping["learning_map"][l]
        #相应数组增加点p
        semantic_array[mapLabel].append(p)
    

    # print("运行时间1毫秒:", rospy.Time.now().to_sec()*1000 - start_time.to_sec()*1000)
    for i in range(len(semantic_array)):
        # 创建PointFields
        semantic_array_inten = []
        if len(semantic_array[i]):
            semantic_array[i] = np.array(semantic_array[i])
            new_column = np.full((semantic_array[i].shape[0],1), intensityList[i])  # 创建形状为 (1, N) 的全 10 数组
            semantic_array_inten = np.hstack((semantic_array[i], new_column))
            semantic_array_inten.tolist()
        topic = mapping["learning_label_topic_map"][i]

        # print("运行时间2毫秒:", rospy.Time.now().to_sec()*1000 - start_time.to_sec()*1000)

        # print(mapping["learning_label_topic_map"][i]," semantic_array_inten num : ", len(semantic_array_inten), )
        laser_cloud_msg = create_cloud(coherent_header, fields, semantic_array_inten)

        # print("运行时间3毫秒:", rospy.Time.now().to_sec()*1000 - start_time.to_sec()*1000)

        # 发布点云
        pub = rospy.Publisher(topic, PointCloud2, queue_size=1)
        pub.publish(laser_cloud_msg)

def separatePoints(points, sem_label):
    # 循环遍历标签和点
    for l,p in zip(sem_label,points):
        mapLabel = mapping["learning_map"][l]
        #相应数组增加点p
        semantic_array[mapLabel].append(p)
    

    # for l,p in sem_label,points:
    #     print(mapping["learning_map"][l])
        
        



def run(points_, sem_label_):
    # 计算运行时间
    start_time = rospy.Time.now()
    # 复制一份 不混淆原本的数据
    points = points_
    sem_label = sem_label_
    publish_point_cloud(points,sem_label)

    print("运行时间3 毫秒：", rospy.Time.now().to_sec()*1000 - start_time.to_sec()*1000)
    
    # print(sem_label)
    # publish_point_cloud(points, sem_label, sem_label_color)
    