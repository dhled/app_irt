#!/usr/bin/env python
import smach
import rospy
import math
import tf
from geometry_msgs.msg import Pose
from ar_track_alvar_msgs.msg import AlvarMarkers



def compute_distance_from_center(pose):
    x_ = pose.position.x
    y_ = pose.position.y
    z_ = pose.position.z
    return math.sqrt(x_**2 + y_**2 + z_**2)


#convert marker from camera frame to robot's base frame
def transform_pose(pose, target_frame="/world"):
    tf_listener = tf.TransformListener()
    if tf_listener.canTransform(target_frame, pose.header.frame_id, rospy.Time(0)):
        transform = tf_listener.transformPose(target_frame, pose)
        return transform.pose
    else:
        return None



class EmptyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue"], input_keys=[], output_keys=[], io_keys=[])

    def execute(self, ud):
        rospy.sleep(2)
        return "continue"



class CheckPoseChange(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["changed", "stopped"])

    def execute(self, ud):
        while not rospy.is_shutdown() or self.preempt_requested():
            try:
                msg = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers, 1)
            except rospy.ROSException:
                continue
            if len(msg.markers) > 0:
                if False:
                    return "changed"
            else:
                continue
                
        return "stopped"

class GetPoseTracker(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["updated", "stopped"], io_keys=["target"])
    
    def execute(self, ud):
        while not rospy.is_shutdown() or self.preempt_requested():
            try:
                msg = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers, 1)
            except rospy.ROSException:
                continue
            if len(msg.markers) > 0:
                marker = msg.markers[0]
                marker.pose.header.frame_id = marker.header.frame_id
                transformed_pose = transform_pose(marker.pose)
                if transformed_pose:
                    transformed_pose.position.z += 0.3
                    ud.target = transformed_pose
                    return "updated"
                else:
                    continue
                
        return "stopped"

        
