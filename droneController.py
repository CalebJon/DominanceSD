import rospy
import tensorflow as tf
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from vision_msgs.msg import Detection2DArray

#defined modes
stabilizeMode = "0"
guidedMode = "4"
loiterMode = "5"
landMode = "9"
posHoldMode = "16"
guidedNoGPSMode = "20"

class DroneController:

	def __init__(self):

		#initialize the node
		rospy.init_node("droneControllerNode")

		#subscribe to input info
		rospy.Subscriber("/mavros/state", State, self.ardupilotStateChange)
		rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.poseUpdate)
		rospy.Subscriber("/detectnet/detections", Detection2DArray, self.detectionUpdate)

		#publishes position commands to the flight controller
		self.positionPub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1);

		#services that can be carried out by the flight controller
		rospy.wait_for_service('/mavros/cmd/arming')
		self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool);
		rospy.wait_for_service('/mavros/cmd/takeoff')
		self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL);
		rospy.wait_for_service('/mavros/setMode')
		self.changeModeService = rospy.ServiceProxy('/mavros/setMode', SetMode);

		#class variables
		self.pose = Pose()
		self.timestamp = rospy.Time()
		self.theta = 0
		self.ardupilotState = stabilizeMode
		self.detectionArray = Detection2DArray()

		rospy.loginfo("droneControllerNode has initialized")


	def arm(self):
		return self.armService(True)


	def ardupilotStateChange(self, data):
		self.ardupilotState = data


	def autoNav(self):
		#while not rospy.isShutdown():


	def detectionUpdate(self, data):
		self.detectionArray = data


	def disarm(self):
		return self.armService(False)


	def gotoPos(x, y, z, yaw):
		pose = pose()
		pose.position.x = x
		pose.position.y = y
		pose.position.z = z

		self.theta = self.theta + yaw

		quat = tf.transformation.quaternion_from_euler(0, 0, self.theta)

		pose.orientation.x = quat[0]
		pose.orientation.y = quat[1]
		pose.orientation.z = quat[2]
		pose.orientation.w = quat[3]

		poseStamped = PoseStamped()
		poseStamped.header.stamp = self.timestamp
		poseStamped.pose = pose

		#command flight controller to fly to these coordinates
		self.positionPub(poseStamped)


	def land(self):
		self.changeModeService(custom_mode=landMode)
		self.disarm()


	def update(self, data):
		self.pose = data.pose
		self.timestamp = data.header.stamp


	def takeoff(self, height=0.5): #takeoff height is in meters
		self.changeModeService(custom_mode=guidedMode)
		self.arm()
		return self.takeoffService(altitude=height)


	def takeoffAndLand(self):
		self.takeoff(height=0.5)
		rospy.sleep(3);
		self.land()


	def main():
		dc = DroneController()

		rospy.loginfo("Drone controller has started")

		rospy.loginfo("Taking off")
		dc.takeoff(height=0.5)	#height is in meters

		rospy.loginfo("Drone has taken off")
		rospy.sleep(3)

		rospy.loginfo("Landing")
		dc.land()

		rospy.loginfo("Cycle complete")

		dc.disarm()


	if __name__ == '__main__':
		main()