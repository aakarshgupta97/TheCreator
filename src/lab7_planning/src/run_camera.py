import sys
import rospy
import intera_interface

def main():
	while not rospy.core.is_shutdown():
		rospy.init_node('camera_display', anonymous=True)
		cameras = intera_interface.Cameras()
		cameras.start_streaming('right_hand_camera')

if __name__ == '__main__':
	main()