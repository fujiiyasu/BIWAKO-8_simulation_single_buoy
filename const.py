import math

class parameter:

	def __init__(self):

		self.V = 12.0

		# ROBOT CONTROL PARAMETER
		self.distance_Kp = 2.0
		self.distance_Kd = 3.0
		self.bearing_Kp = 0.5
		self.bearing_Kd = 2.3
		self.STRAGHT_MAX_THRUST = 12.0
		self.KEEP_MAX_THRUST = 10.0*(math.sqrt(2.0)/2)
		self.TIME_STEP = 100

		# WAY POINT
		self.way_point_file = './waypoint/test.csv'

		# MODE
		self.data_log_mode = False
		self.debug_mode = False
		self.state_display_mode = False
		self.gps_error_mode = False

		# DISTURBANCE PATTERN
		self.disturbance_mode = 0
		# 0: NATURAL DISTURBANCE, 1: RANDOM DISTURBANCE, 2: STEP DISTURBANCE

		# CONTROL MODE
		self.mode = 0
		# 0:直進モード, 1:カタマランモード，2: キープモード

		# CONTROL STRATEGY
		self.policy = 0
		# 0:SIMPLE POLICY, 1:FLEXIBLE POLICY

		# OTHER
		self.r = 1.0
		self.main_distance_tolerance = 3.0
		self.wait_distance_tolerance = 10.0
		self.head_tolerance = 5.0
		self.monitoring_duration = 5 # [min]
		self.disturbance_duration = 15 # [min]
		self.traveling_duration = 60 * 0.5# [min]

		self.max_traveling_num = ((24 * 60)/self.traveling_duration)+1

		self.monitoring_step = self.min_to_step(self.monitoring_duration)
		self.disturbance_step = self.min_to_step(self.disturbance_duration)
		self.traveling_step = self.min_to_step(self.traveling_duration)
		self.workspace = 'C:/Users/is0232xf/OneDrive - 学校法人立命館/Webots/BIWAKO-X_waypoint/controllers/Python_BIWAKO-X/result/'

	def min_to_step(self, min):
		step = (min * 60 * 1000)/self.TIME_STEP
		return step
