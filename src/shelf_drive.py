#!/usr/bin/env python2.7
import rospy
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import UInt32MultiArray
from geometry_msgs.msg import Twist
import rospy
import tf

# Dictionary visible markers
markers = {}
# Current_velocities_global
current_vel_z = 0
current_vel_x = 0
current_vel_roll = 0
# Current_target_point_index
crnt_trgt_pnt_idx = 0
# velocity message
cmd_vel_msg = Twist()

# modes of corrections are :parallel, secuencial, single, semi-secuencial
trajectory_plan = {"0":{},"1":{},"2":{},"3":{},"4":{},"5":{}}  # trajectory 
# definition of the first point
trajectory_plan["0"]["reference_marker_id"] = 501
trajectory_plan["0"]["reached"] = False
trajectory_plan["0"]["corrections"] = {
    "type": "semi-secuencial", "finished": False, "0": {}, "1": {}}
trajectory_plan["0"]["corrections"]["1"]["type"] = "single"
trajectory_plan["0"]["corrections"]["1"]["axis"] = "z"
trajectory_plan["0"]["corrections"]["1"]["error_tolerance"] = 0.05
trajectory_plan["0"]["corrections"]["1"]["controller_parameters"] = {
    "k": 300, "bias": 400}
trajectory_plan["0"]["corrections"]["1"]["target"] = 0.25
trajectory_plan["0"]["corrections"]["1"]["finished"] = False
trajectory_plan["0"]["corrections"]["0"]["type"] = "single"
trajectory_plan["0"]["corrections"]["0"]["axis"] = "roll"
trajectory_plan["0"]["corrections"]["0"]["error_tolerance"] = 0.5
trajectory_plan["0"]["corrections"]["0"]["controller_parameters"] = {
    "k": -300, "bias": -400}
trajectory_plan["0"]["corrections"]["0"]["target"] = -3.14/2
trajectory_plan["0"]["corrections"]["0"]["finished"] = False


# definition of the second point
trajectory_plan["1"]["reference_marker_id"] = 501
trajectory_plan["1"]["reached"] = False
trajectory_plan["1"]["corrections"] = {
    "type": "semi-secuencial", "finished": False, "0": {},"1":{}}
trajectory_plan["1"]["corrections"]["0"]["type"] = "single"
trajectory_plan["1"]["corrections"]["0"]["axis"] = "z"
trajectory_plan["1"]["corrections"]["0"]["error_tolerance"] = 0.05
trajectory_plan["1"]["corrections"]["0"]["controller_parameters"] = {
    "k": 200, "bias": 400}
trajectory_plan["1"]["corrections"]["0"]["target"] = 0.40
trajectory_plan["1"]["corrections"]["0"]["finished"] = False
trajectory_plan["1"]["corrections"]["1"]["type"] = "single"
trajectory_plan["1"]["corrections"]["1"]["axis"] = "x"
trajectory_plan["1"]["corrections"]["1"]["error_tolerance"] = 0.005
trajectory_plan["1"]["corrections"]["1"]["controller_parameters"] = {
    "k": 250, "bias": 400}
trajectory_plan["1"]["corrections"]["1"]["target"] = 1.3
trajectory_plan["1"]["corrections"]["1"]["finished"] = False

# definition of the third point
trajectory_plan["2"]["reference_marker_id"] = 501
trajectory_plan["2"]["reached"] = False
trajectory_plan["2"]["corrections"] = {
    "type": "semi-secuencial", "finished": False, "0": {}}
trajectory_plan["2"]["corrections"]["0"]["type"] = "single"
trajectory_plan["2"]["corrections"]["0"]["axis"] = "roll"
trajectory_plan["2"]["corrections"]["0"]["error_tolerance"] = 0.1
trajectory_plan["2"]["corrections"]["0"]["controller_parameters"] = {
    "k": -700, "bias": -300}
trajectory_plan["2"]["corrections"]["0"]["target"] = -3.14/1.8
trajectory_plan["2"]["corrections"]["0"]["finished"] = False

#definition of forth point
trajectory_plan["3"]["reference_marker_id"] = 401
trajectory_plan["3"]["reached"] = False
trajectory_plan["3"]["corrections"] = {
    "type": "semi-secuencial", "finished": False, "0": {}, "1": {}}
trajectory_plan["3"]["corrections"]["1"]["type"] = "single"
trajectory_plan["3"]["corrections"]["1"]["axis"] = "z"
trajectory_plan["3"]["corrections"]["1"]["error_tolerance"] = 0.005
trajectory_plan["3"]["corrections"]["1"]["controller_parameters"] = {
    "k": 500, "bias": 400}
trajectory_plan["3"]["corrections"]["1"]["target"] = 0
trajectory_plan["3"]["corrections"]["1"]["finished"] = False
trajectory_plan["3"]["corrections"]["0"]["type"] = "single"
trajectory_plan["3"]["corrections"]["0"]["axis"] = "roll"
trajectory_plan["3"]["corrections"]["0"]["error_tolerance"] = 0.1
trajectory_plan["3"]["corrections"]["0"]["controller_parameters"] = {
    "k": -700, "bias": -300}
trajectory_plan["3"]["corrections"]["0"]["target"] = -3.14/2
trajectory_plan["3"]["corrections"]["0"]["finished"] = False

#definition of the fifth point
trajectory_plan["4"]["reference_marker_id"] = 401
trajectory_plan["4"]["reached"] = False
trajectory_plan["4"]["corrections"] = {
    "type": "semi-secuencial", "finished": False, "0": {},"1":{},"2":{}}
trajectory_plan["4"]["corrections"]["1"]["type"] = "single"
trajectory_plan["4"]["corrections"]["1"]["axis"] = "z"
trajectory_plan["4"]["corrections"]["1"]["error_tolerance"] = 0.05
trajectory_plan["4"]["corrections"]["1"]["controller_parameters"] = {
    "k": 200, "bias": 400}
trajectory_plan["4"]["corrections"]["1"]["target"] = -0.08
trajectory_plan["4"]["corrections"]["1"]["finished"] = False
trajectory_plan["4"]["corrections"]["2"]["type"] = "single"
trajectory_plan["4"]["corrections"]["2"]["axis"] = "x"
trajectory_plan["4"]["corrections"]["2"]["error_tolerance"] = 0.05
trajectory_plan["4"]["corrections"]["2"]["controller_parameters"] = {
    "k": 200, "bias": 400}
trajectory_plan["4"]["corrections"]["2"]["target"] = 0.9
trajectory_plan["4"]["corrections"]["2"]["finished"] = False
trajectory_plan["4"]["corrections"]["0"]["type"] = "single"
trajectory_plan["4"]["corrections"]["0"]["axis"] = "roll"
trajectory_plan["4"]["corrections"]["0"]["error_tolerance"] = 0.005
trajectory_plan["4"]["corrections"]["0"]["controller_parameters"] = {
    "k": -300, "bias": -300}
trajectory_plan["4"]["corrections"]["0"]["target"] = -3.14/2
trajectory_plan["4"]["corrections"]["0"]["finished"] = False

#definition of sixth point
trajectory_plan["5"]["reference_marker_id"] = 401
trajectory_plan["5"]["reached"] = False
trajectory_plan["5"]["corrections"] = {
    "type": "semi-secuencial", "finished": False, "0": {},"1":{},"2":{}}
trajectory_plan["5"]["corrections"]["1"]["type"] = "single"
trajectory_plan["5"]["corrections"]["1"]["axis"] = "z"
trajectory_plan["5"]["corrections"]["1"]["error_tolerance"] = 0.005
trajectory_plan["5"]["corrections"]["1"]["controller_parameters"] = {
    "k": 500, "bias": 200}
trajectory_plan["5"]["corrections"]["1"]["target"] = -0.08
trajectory_plan["5"]["corrections"]["1"]["finished"] = False
trajectory_plan["5"]["corrections"]["2"]["type"] = "single"
trajectory_plan["5"]["corrections"]["2"]["axis"] = "x"
trajectory_plan["5"]["corrections"]["2"]["error_tolerance"] = 0.005
trajectory_plan["5"]["corrections"]["2"]["controller_parameters"] = {
    "k": 500, "bias": 200}
trajectory_plan["5"]["corrections"]["2"]["target"] = 0.597
trajectory_plan["5"]["corrections"]["2"]["finished"] = False
trajectory_plan["5"]["corrections"]["0"]["type"] = "single"
trajectory_plan["5"]["corrections"]["0"]["axis"] = "roll"
trajectory_plan["5"]["corrections"]["0"]["error_tolerance"] = 0.001
trajectory_plan["5"]["corrections"]["0"]["controller_parameters"] = {
    "k": -300, "bias": -300}
trajectory_plan["5"]["corrections"]["0"]["target"] = -3.14/2
trajectory_plan["5"]["corrections"]["0"]["finished"] = False



def ramp(beginning_value, ending_value):
    return_value = beginning_value
    slope_factor = 100
    if abs(beginning_value - ending_value) <= slope_factor:
        return ending_value
    elif beginning_value > ending_value:
        return_value = beginning_value - slope_factor
    elif beginning_value < ending_value:
        return_value = beginning_value + slope_factor

    return return_value


def stop():
    global current_vel_z, current_vel_x, current_vel_roll
    current_vel_x = ramp(current_vel_x, 0)
    current_vel_z = ramp(current_vel_z, 0)
    current_vel_roll = ramp(current_vel_roll, 0)


def process_correction(marker, correction):
    global current_vel_z, current_vel_x, current_vel_roll
    if correction["type"] == "single":
        error = marker[correction["axis"]]-correction["target"]
        k = correction["controller_parameters"]["k"]
        bias = correction["controller_parameters"]["bias"]
        error_tolerance = correction["error_tolerance"]
        target_vel = (error) * k + (error)/abs(error) * bias
        rospy.loginfo(error)
        if correction["axis"] == "z":
            # correct z
            if(abs(error) > error_tolerance):
                current_vel_z = ramp(current_vel_z, target_vel)
                # current_vel_x = ramp(current_vel_x, 0)
                # current_vel_roll = ramp(current_vel_roll, 0)
                correction["finished"] = False
            else:
                current_vel_z = ramp(current_vel_z, 0)
                correction["finished"] = True
        elif correction["axis"] == "x":
            # correct x
            if(abs(error) > error_tolerance):
                # current_vel_z = ramp(current_vel_z, 0)
                current_vel_x = ramp(current_vel_x, target_vel)
                # current_vel_roll = ramp(current_vel_roll, 0)
                correction["finished"] = False
            else:
                current_vel_x = ramp(current_vel_x, 0)
                correction["finished"] = True
        elif correction["axis"] == "roll":
            # correct roll
            if(abs(error) > error_tolerance):
                # current_vel_z = ramp(current_vel_z, 0)
                # current_vel_x = ramp(current_vel_x, 0)
                current_vel_roll = ramp(current_vel_roll, target_vel)
                correction["finished"] = False
            else:
                current_vel_roll = ramp(current_vel_roll, 0)
                correction["finished"] = True
    elif correction["type"] == "secuencial":
        for i in range(0, len(correction)-2):
            if correction[str(i)]["finished"] == False:
                process_correction(marker, correction[str(i)])
                break
            else:
                continue
        if correction[str(len(correction)-3)]["finished"] == True:
            correction["finished"] = True
    elif correction["type"] == "parallel":
        correction["finished"] = True
        for i in range(0, len(correction)-2):
            process_correction(marker, correction[str(i)])
            if correction[str(i)]["finished"] == False:
                correction["finished"] = False
    elif correction["type"] == "semi-secuencial":
        for i in range(0, len(correction)-2):
            if correction[str(i)]["finished"] == False:
                process_correction(marker, correction[str(i)])
                break
            else:
                correction[str(i)]["finished"] = False
                continue
        if correction[str(len(correction)-3)]["finished"] == True:
            correction["finished"] = True
    return correction["finished"]


def Markers_list_Callback(msg):
    global current_vel_z, current_vel_x, current_vel_roll
    global crnt_trgt_pnt_idx
    global cmd_vel_msg
    global markers

    if crnt_trgt_pnt_idx < len(trajectory_plan):
        current_point = trajectory_plan[str(crnt_trgt_pnt_idx)]
        number_of_detected_markers = len(msg.data)
        if number_of_detected_markers == 0:
            # no markers found
            stop()
        else:  # found some markers
            if current_point["reached"] == True:  # point has already been reached
                crnt_trgt_pnt_idx = crnt_trgt_pnt_idx + 1  # move to next point
                rospy.loginfo("reached point")
            else:  # point has not been processed yet
                if current_point["reference_marker_id"] in msg.data:
                    rospy.loginfo("processing point and found its marker index: " + str(crnt_trgt_pnt_idx))
                    current_point["reached"] = process_correction(
                        markers[current_point["reference_marker_id"]], current_point["corrections"])

                else:  # reference marker not found, stop
                    rospy.loginfo("processing point and marker is not found")
                    stop()
    else:  # reached the end of the trajectory
        current_vel_z = ramp(current_vel_z, 0)
        current_vel_x = ramp(current_vel_x, 0)
        current_vel_roll = ramp(current_vel_roll, 0)

    cmd_vel_msg.linear.z = current_vel_z
    cmd_vel_msg.linear.x = current_vel_x
    cmd_vel_msg.angular.z = current_vel_roll
    pub.publish(cmd_vel_msg)


def MarkersCallback(msg):
    global markers

    for marker in msg.markers:
        orientation_q = marker.pose.pose.orientation
        orientation_list = [
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            orientation_list)
        # rospy.loginfo ("yaw:"+str(yaw)+" pitch:"+str(pitch)+" roll:"+str(roll))
        z = marker.pose.pose.position.x
        y = marker.pose.pose.position.y
        x = marker.pose.pose.position.z

        markers[marker.id] = {"z": z, "x": x, "roll": roll}

    # global current_vel_z, current_vel_x, current_vel_roll
    # global Final_stage
    # # global roll, pitch, yaw
    # cmd_vel_msg = Twist()
    # if len(msg.markers) != 0:
    #     for marker in msg.markers:
    #         if marker.id == 501:
    #             orientation_q = marker.pose.pose.orientation
    #             orientation_list = [
    #                 orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #             (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
    #                 orientation_list)
    #             # rospy.loginfo ("yaw:"+str(yaw)+" pitch:"+str(pitch)+" roll:"+str(roll))
    #             z = marker.pose.pose.position.x
    #             y = marker.pose.pose.position.y
    #             x = marker.pose.pose.position.z

    #             Target_z = 0.5
    #             Target_x = 1.7
    #             Target_roll = -3.14/2
    #             Error_tolerance_distance = 0.001  # meters
    #             Error_tolerance_angles = 0.01  # radians
    #             k = 2000
    #             bias = 200

    #             Reached_target_x = False
    #             Reached_target_z = False
    #             Reached_target_roll = False

    #             Error_z = z-Target_z
    #             Error_x = x-Target_x
    #             Error_roll = roll-Target_roll

    #             Target_vel_z = (Error_z) * k + (Error_z)/abs(Error_z) * bias
    #             Target_vel_x = (Error_x) * k + (Error_x)/abs(Error_x) * bias
    #             Target_vel_roll = (-1*Error_roll) * k - \
    #                 (Error_roll)/abs(Error_roll) * bias

    #             cmd_vel_msg.linear.x = 0
    #             cmd_vel_msg.linear.y = 0
    #             cmd_vel_msg.linear.z = 0
    #             cmd_vel_msg.angular.x = 0
    #             cmd_vel_msg.angular.y = 0
    #             cmd_vel_msg.angular.z = 0

    #             if Final_stage == False:
    #                 if(abs(Error_z) > Error_tolerance_distance*100):
    #                     current_vel_z = ramp(current_vel_z, Target_vel_z)
    #                     current_vel_x = ramp(current_vel_x, 0)
    #                     current_vel_roll = ramp(current_vel_roll, 0)

    #                     Reached_target_z = False
    #                 else:
    #                     current_vel_z = ramp(current_vel_z, 0)
    #                     Reached_target_z = True

    #                 if(Reached_target_z == True):
    #                     if(abs(Error_x) > Error_tolerance_distance*10):
    #                         current_vel_z = ramp(current_vel_z, 0)
    #                         current_vel_x = ramp(current_vel_x, Target_vel_x)
    #                         current_vel_roll = ramp(current_vel_roll, 0)

    #                         Reached_target_x = False
    #                     else:
    #                         current_vel_x = ramp(current_vel_x, 0)
    #                         Reached_target_x = True
    #                         Final_stage = True

    #             else:
    #                 if(abs(Error_z) > Error_tolerance_distance*2):
    #                     current_vel_z = ramp(current_vel_z, Target_vel_z/2)
    #                 else:
    #                     current_vel_z = ramp(current_vel_z, 0)
    #                 if(abs(Error_x) > Error_tolerance_distance):
    #                     current_vel_x = ramp(current_vel_x, Target_vel_x/2)
    #                 else:
    #                     current_vel_x = ramp(current_vel_x, 0)
    #                 if(abs(Error_roll) > Error_tolerance_angles):
    #                     current_vel_roll = ramp(
    #                         current_vel_roll, Target_vel_roll/2)
    #                 else:
    #                     current_vel_roll = ramp(current_vel_roll, 0)

    #             cmd_vel_msg.linear.z = current_vel_z
    #             cmd_vel_msg.linear.x = current_vel_x
    #             cmd_vel_msg.angular.z = current_vel_roll

    # else:
    #     cmd_vel_msg.linear.x = ramp(current_vel_x, 0)
    #     cmd_vel_msg.linear.y = 0
    #     cmd_vel_msg.linear.z = ramp(current_vel_z, 0)
    #     cmd_vel_msg.angular.x = 0
    #     cmd_vel_msg.angular.y = 0
    #     cmd_vel_msg.angular.z = ramp(current_vel_roll, 0)

    # pub.publish(cmd_vel_msg)


rospy.init_node('shelf_drive_node', log_level=rospy.DEBUG)
pub = rospy.Publisher('/cmd_vel', Twist)
sub = rospy.Subscriber('/aruco_marker_publisher/markers',
                       MarkerArray, MarkersCallback)
sub_markers_list = rospy.Subscriber(
    '/aruco_marker_publisher/markers_list', UInt32MultiArray, Markers_list_Callback)

r = rospy.Rate(100)
while not rospy.is_shutdown():
    r.sleep()
