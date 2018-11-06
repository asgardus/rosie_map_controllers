import rospy
from std_msgs.msg import String, Float32,
from tf.msg import transform_broadcaster
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import OccupancyGrid, Odometry
from visulalization_msgs.msg import MarkerArray, Marker

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

EXTEND_AREA = 10.0  # [m] grid map extention length

show_animation = True

OccupancyGrid localGrid

def generate_gaussian_grid_map(xyreso, std):

    minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(xyreso)

    gmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        for iy in range(yw):

            x = ix * xyreso + minx
            y = iy * xyreso + miny

            # Search minimum distance
            mindis = float("inf")
            for (iox, ioy) in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if mindis >= d:
                    mindis = d

            pdf = (1.0 - norm.cdf(mindis, 0.0, std))
            gmap[ix][iy] = pdf

    return gmap, minx, maxx, miny, maxy


def calc_grid_map_config(xyreso):
    minx = 0
    miny = 0
    maxx = localGrid.info.width
    maxy = localGrid.info.height
	
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))

    return minx, miny, maxx, maxy, xw, yw


def draw_heatmap(data, minx, maxx, miny, maxy, xyreso):
    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]


def odomCallback(data):
	data.pose.pose

def mapCallback(data):
	localGrid = data

def main():
    print(__file__ + " start!!")
	rospy.init_node('gaussmap', anonymous=True)
	
	
	grid_pub = rospy.Publisher('rosie_occupancy_grid',OccupancyGrid, queue_size=10)
	wall_pub = rospy.Publisher('walls', Int16, queue_size=10)
	marker_sub = rospy.Subscriber('maze_map', MarkerArray, mapCallback)
	odom_sub = rospy.Subscriber('odom', Odometry, odomCallback)
	rate = rospy.Rate(10)

    xyreso = localGrid.info.resolution  # xy grid resolution
    STD = 0.25 # standard diviation for gaussian distribution

    gmap, minx, maxx, miny, maxy = generate_gaussian_grid_map(
        xyreso, STD)

	
	while not rospy.is_shutdown():
		pub.publish("thisisastring")
		rate.sleep()
		rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
