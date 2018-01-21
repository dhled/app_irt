#!/usr/bin/env python

import smach
import rospy
import actionlib
from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest, GetMotionPlanResponse,\
                            GetCartesianPath, GetCartesianPathRequest, GetCartesianPathResponse
from moveit_msgs.msg import JointConstraint, Constraints, ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped



class ServiceState(smach.State):
    def __init__(self, service_name, service_type, outcomes, io_keys):
        smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys)
        self._serv_name = service_name
        self._serv_type = service_type
        self._to = 10
        if "preempt" not in self.get_registered_outcomes():
            self.register_outcomes(["preempt"])
        if "timeout" not in self.get_registered_outcomes():
            self.register_outcomes(["timeout"])

    def setup_request(self, ud):
        raise NotImplementedError
    
    def analyse_answer(self, ud, service_answer):
        raise NotImplementedError
    
    def wait_for_service(self, service_request):
        to_ = rospy.Time.now() + rospy.Duration(self._to)
        service_answer = None
        while service_answer == None:
            rospy.sleep(0.01)
            if self.preempt_requested() or rospy.is_shutdown():
                self.service_preempt()
                return "preempt"
            if rospy.Time.now() > to_:
                rospy.logwarn("'%s timed out : it will return the following outcome '%s'"
                             %(self.__class__.__name__,self._to_outcome))
                return "timeout"
            else:
                try:
                    rospy.wait_for_service(self._serv_name,1)
                except Exception:
                    continue
                try:
                    service_caller = rospy.ServiceProxy(self._serv_name, self._serv_type)
                    service_answer = service_caller(service_request)
                    return service_answer
                except Exception:
                    continue

    
    def execute(self, ud):
        service_request = self.setup_request(ud)
        service_answer = self.wait_for_service(service_request)
        if service_answer == "preempt":
            return "preempt"
        elif service_answer == "timeout":
            return self._to_outcome
        else:
            return self.analyse_answer(ud, service_answer)


class ActionClientState(smach.State):
    def __init__(self, action_server_name, action_type, outcomes=[], io_keys=[]):
        smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys)
        self._tosv = 10
        self._toac = 120
        self._as_name = action_server_name
        self._action_type = action_type
        self._local_userdata = None
        self._done = False
        
        self.ac_client = None
        if "preempt" not in self.get_registered_outcomes():
            self.register_outcomes(["preempt"])
        if "timeout" not in self.get_registered_outcomes():
            self.register_outcomes(["timeout"])
    
    def setup_goal(self, ud):
        raise NotImplementedError
    
    def analyse_feedback(self, ud, feedback_msg):
        pass
    
    def analyse_result(self, ud, result):
        raise NotImplementedError
    
    def done_cb(self, status, result):
        self._done = True
    
    def feedback_wrapper(self, msg):
        self.analyse_feedback(self._local_userdata, msg)
    
    def wait_for_server(self):
        to_ = rospy.Time.now() + rospy.Duration(self._tosv)
        server_connected = False
        while server_connected == False:
            rospy.sleep(0.01)
            if self.preempt_requested() or rospy.is_shutdown():
                self.service_preempt()
                return "preempt"
            if rospy.Time.now() > to_:
                rospy.logwarn("'%s connection to server timed out : it will return the following outcome '%s'"
                             %(self.__class__.__name__,self._tosv_outcome))
                return "timeout"
            else:
                server_connected = self.ac_client.wait_for_server(rospy.Duration(0.1))
                
    def wait_for_result(self):
        to_ = rospy.Time.now() + rospy.Duration(self._toac)
        while self._done == False:
            if self.preempt_requested() or rospy.is_shutdown():
                self.service_preempt()
                self.ac_client.cancel_goal()
                return "preempt"
            if rospy.Time.now() > to_:
                rospy.logwarn("'%s action timed out : it will return the following outcome '%s'"
                             %(self.__class__.__name__,self._toac_outcome))
                self.ac_client.cancel_goal()
                return "timeout"
            else:
                rospy.sleep(0.01)

        
    def execute(self, ud):
        self._local_userdata = ud
        goal = self.setup_goal(ud)
        self.ac_client = actionlib.SimpleActionClient(self._as_name, self._action_type)
        connection = self.wait_for_server()
        if connection == "preempt":
            return "preempt"
        elif connection == "timeout":
            return self._tosv_outcome
        else:
            pass
        self._done = False
        self.ac_client.send_goal(goal, done_cb = self.done_cb, active_cb=None, feedback_cb = self.feedback_wrapper)
        result = self.wait_for_result()
        if result == "preempt":
            rospy.loginfo("goal preempted")
            return "preempt"
        elif result == "timeout":
            return self._toac_outcome
        else:
            pass
        ud = self._local_userdata
        return self.analyse_result(ud, self.ac_client.get_result())

class GenerateArticularPlan(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, "/plan_kinematic_path", GetMotionPlan, outcomes=["success"], io_keys=["target", "group_name", "plan", "namespace"])
        
    def convert_to_joint_constraints(self, current_js, target):
        jcs = []
        if isinstance(target, list):
            for i_jt in range(len(target)):
                jc = JointConstraint()
                jc.joint_name = current_js.name[i_jt]
                jc.position = target[i_jt]
                jc.tolerance_above = jc.tolerance_below = 0.001
                jc.weight = 1.0
                jcs.append(jc)
        else: 
            rospy.logerr("[GenerateArticularPlan] Not compatible data 'target' with current value : %s"%str(target))
            raise Exception()
        return jcs
            
    def get_last_joint_state(self, namespace):
        if(namespace != '""'):
            state = rospy.wait_for_message("/"+namespace+"/joint_states", JointState, timeout=5)
        else:
            state = rospy.wait_for_message("/joint_states", JointState, timeout=5)
        return state
                
    def setup_request(self, ud):
        request = GetMotionPlanRequest()
        request.motion_plan_request.group_name = ud.group_name
        current_state = self.get_last_joint_state(ud.namespace)
        jcs = self.convert_to_joint_constraints(current_state, ud.target, ud.type)
        if(len(jcs) == 0):
            raise Exception()
        request.motion_plan_request.start_state.joint_state = current_state
        request.motion_plan_request.start_state.is_diff = True
        _constraint = Constraints()
        _constraint.joint_constraints = jcs
        request.motion_plan_request.goal_constraints.append(_constraint)
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 1.0
        request.motion_plan_request.max_acceleration_scaling_factor = 1.0
        return request
    
    def analyse_answer(self, ud, service_answer):
        if(service_answer.motion_plan_response.error_code.val != 1):
            rospy.logerr("[GenerateCartesianPlan] Planning failed")
            return "preempt"
        plan = service_answer.motion_plan_response.trajectory
        ud.plan = plan
        return "success"

    
class GenerateCartesianPlan(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, "/compute_cartesian_path", GetCartesianPath, outcomes=["success"], io_keys=["target", "group_name", "plan", "namespace"])
        
    def convert_to_waypoints(self, target):
        waypoints = []
        if(isinstance(target, Pose) or isinstance(target, PoseStamped)): # 1 points but in the Pose or Posestamped format
            waypoints.append(target)
        else:
            rospy.logerr("[GenerateCartesianPlan] Userdata 'target' with current value %s is not compatible." %str(target))
        return waypoints
    
    def convertToPose(self, pt):
        if(isinstance(pt,Pose) or isinstance(pt,PoseStamped)):
            return pt
        else:
            pose = Pose()
            if(len(pt) != 7):
                rospy.logerr("[GenerateCartesianPlan] Not well formated point in the userdata 'target' : %s "%str(pt))
                raise Exception()
            else:
                pose.position.x = pt[0]
                pose.position.y = pt[1]
                pose.position.z = pt[2]
                pose.orientation.x = pt[3]
                pose.orientation.y = pt[4]
                pose.orientation.z = pt[5]
                pose.orientation.w = pt[6]
            return pose
        
    def get_last_joint_state(self, namespace):
        if(namespace != '""'):
            state = rospy.wait_for_message("/"+namespace+"/joint_states", JointState, timeout=5)
        else:
            state = rospy.wait_for_message("/joint_states", JointState, timeout=5)
        return state
                
    def setup_request(self, ud):
        request = GetCartesianPathRequest()
        request.group_name = ud.group_name
        waypoints = self.convert_to_waypoints(ud.target)
        if(len(waypoints) == 0):
            raise Exception()
        request.waypoints = waypoints
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.start_state.joint_state = self.get_last_joint_state(ud.namespace)
        return request
    
    def analyse_answer(self, ud, service_answer):
        if(service_answer.error_code.val != 1):
            rospy.logerr("[GenerateCartesianPlan] Planning failed")
            return "preempt"
        plan = service_answer.solution
        fraction = service_answer.fraction
        if(fraction ==0):
            rospy.loginfo(plan)
            rospy.loginfo(fraction)
            rospy.logerr("[GenerateCartesianPlan] Planning fraction is different than 1")
            return "preempt"
        ud.plan = plan
        return "success"


class Move(ActionClientState):
    def __init__(self):
        ActionClientState.__init__(self, "/execute_trajectory", ExecuteTrajectoryAction , outcomes=["success"], io_keys=["plan"])
    
    def setup_goal(self, ud):
        goal = ExecuteTrajectoryGoal()
        goal.trajectory = ud.plan
        return goal
    
    def analyse_result(self, ud, result):
        if(result.error_code.val != 1):
            rospy.logerr("[Move] Moving failed")
            return "preempt"
        return "success"