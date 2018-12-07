from ar_track_alvar_msgs.msg import AlvarMarkers
import math
import numpy as np
import rospy 

received_marker = False

marker_rotations = []
marker_translations = []

def ar_marker_callback(ar_pose):
    print("Entered")
    global received_marker
    if received_marker is False:
        print("Hit the ar_marker callback!")
        markers = ar_pose.markers
        assert len(markers) == 1
        marker = markers[0]
        pose = marker.pose.pose
        trans = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
        R = quaternion_matrix(q)
        marker_rotations.append(R)
        marker_translations.append(trans)
        received_marker = True
    else:
        pass

def transform(pc, R, t):
    return np.dot(R.T, ((pc - t).T)).T

def quaternion_matrix(quaternion):
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def get_top_left_AR_tag():

    global received_marker
    ar_topic = '/ar_pose_marker'

    while not rospy.core.is_shutdown() and not received_marker:
        rospy.Subscriber(ar_topic, AlvarMarkers, ar_marker_callback)

    rospy.rostime.wallsleep(0.05)
    print("Got Marker!")

def main():
    get_top_left_AR_tag()
    return marker_translations

if __name__ == '__main__':
    main()