#! /usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetMapResponse, GetMapRequest
import numpy as np


def map_callback(msg):

    pass

if __name__ == '__main__':
    rospy.init_node('map_test')
    map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)
    get_map = rospy.ServiceProxy('/dynamic_map', GetMap)
    get_map.wait_for_service()
    req = GetMapRequest()
    grids = get_map(req)
    print grids.map.header
    print grids.map.info
    map_data = np.asarray(grids.map.data,np.int8)
    np.savetxt('map.txt', map_data, delimiter=',')