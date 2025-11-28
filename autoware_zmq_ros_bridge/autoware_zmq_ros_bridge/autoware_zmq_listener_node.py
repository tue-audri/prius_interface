import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import GearCommand, HazardLightsCommand, TurnIndicatorsCommand
from tier4_vehicle_msgs.msg import ActuationCommandStamped, VehicleEmergencyStamped
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearReport, SteeringReport, VelocityReport, TurnIndicatorsReport, HazardLightsReport, ControlModeReport
import zmq
import threading
import capnp
import os
import math
from std_srvs.srv import SetBool
from std_msgs.msg import Int32

class ZMQCapnpBridgeNode(Node):
    def __init__(self):
        super().__init__('zmq_capnp_bridge_node')

        # Declare and get parameters
        self.declare_parameter('capnp_dir', '')
        self.declare_parameter('publish_rate_hz', 10.0)
        capnp_dir = self.get_parameter('capnp_dir').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # New parameter
        self.declare_parameter('openpilot_auto_reenable_timeout', 5.0)  # Timeout in seconds
        self.openpilot_auto_reenable_timeout = self.get_parameter('openpilot_auto_reenable_timeout').get_parameter_value().double_value

        self.zmq_publish_rate=100

        self.openpilot_state = True
        self.manual_override = False

        self.openpilot_disabled_time = None

        # Load Cap'n Proto schema
        if capnp_dir:
            capnp.remove_import_hook()
            self.log_capnp = capnp.load(os.path.join(capnp_dir, 'log.capnp'))
        else:
            self.get_logger().error('Capnp directory not provided. Exiting.')
            return

        # ZMQ setup
       

        self.get_logger().info("Initializing ZMQ context and sockets...")

        try:
            self.zmq_context = zmq.Context()
            self.get_logger().info("ZMQ context created successfully")

            # Subscriber socket for carState
            self.get_logger().info("Creating ZMQ subscriber socket...")
            self.sub_socket = self.zmq_context.socket(zmq.SUB)
            self.sub_socket.connect("tcp://127.0.0.1:9041")
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, '')

            # Publisher socket for carControl
            self.get_logger().info("Creating ZMQ publisher socket...")
            self.pub_socket = self.zmq_context.socket(zmq.PUB)
            self.pub_socket.bind("tcp://127.0.0.1:63225")
            self.get_logger().info("ZMQ publisher socket created successfully")
        except Exception as e:
            self.get_logger().error(f"Error initializing ZMQ: {e}")
            return

        # ROS-to-ZMQ control signals dictionary
        self.control_signals = {
            'steering_angle_deg': 0.0,
            'accel': 0.0,
            'brake': 0.0,
            'hazard_lights': False,
            'turn_signal': 0,
            'gear': 0,
            'emergency': False,
            'enable': True,
            'latActive': True,
            'longActive': True,
            'cruise_cancel': False,
            'openpilot_state': 0,
            'longControlState':0,
        }

        # ZMQ-to-ROS state signals dictionary
        self.state_signals = {
            'steering_angle_deg': 0.0,
            'vEgo': 0.0,
            'gear': 'park',
            'turn_signal': 0,
            'hazard_lights': 1,
            'yaw_rate': 0.0,
            'control_mode': 1,
            'steeringPressed': False,
            'gasPressed': False,
            'brakePressed': False,
            'cruiseEnabled' : False,
            'accFaulted' : True,
        }

        self.turn_signal_mapping = { #DBC -> Autoware
            1: 2,  # Left
            2: 3,  # Right
            3: 1   # Disabled
        }
        self.gear_mapping = { #DBC -> Autoware
            'park': 22,  # P → Autoware: 22
            'reverse': 20,  # R → 20
            'neutral': 1,   # N → 1
            'drive': 3,   # D → 3
            # B (treated as Drive) → 3
        }

        #Calculated from the wheel lock of 37deg and the maximum steering angle of 470deg
        self.steer_ratio = 12.702 #15.74 from openpilot

        # ROS publishers
        self.control_mode_pub = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', 10)
        self.gear_pub = self.create_publisher(GearReport, '/vehicle/status/gear_status', 10)
        self.hazard_pub = self.create_publisher(HazardLightsReport, '/vehicle/status/hazard_lights_status', 10)
        self.steer_pub = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        self.turn_pub = self.create_publisher(TurnIndicatorsReport, '/vehicle/status/turn_indicators_status', 10)
        self.vel_pub = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)
        self.openpilot_state_pub = self.create_publisher(Int32, '/vehicle/openpilot/state', 10)

        # ROS subscribers
        #self.create_subscription(ActuationCommandStamped, '/control/command/actuation_cmd', self.actuation_cb, 10)
        self.create_subscription(Control, '/control/command/control_cmd', self.control_cb, 10)
        self.create_subscription(VehicleEmergencyStamped, '/control/command/emergency_cmd', self.emergency_cb, 10)
        self.create_subscription(GearCommand, '/control/command/gear_cmd', self.gear_cmd_cb, 10)
        self.create_subscription(HazardLightsCommand, '/control/command/hazard_lights_cmd', self.hazard_cmd_cb, 10)
        self.create_subscription(TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', self.turn_cmd_cb, 10)
        self.enable_srv = self.create_service(SetBool, '/openpilot/enable', self.enable_cb)
                # Start periodic publishing timer
        self.create_timer(1.0 / self.publish_rate, self.publish_ros_messages)

        self.create_timer(1.0 / self.zmq_publish_rate, self.send_car_control)

        # Add connection health check timer
        self.create_timer(5.0, self.check_zmq_connection)  # Check every 5 seconds

        self.listener_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listener_thread.start()

        #For Testing
        self.steering_test_pub = self.create_publisher(SteeringReport, '/vehicle/status/steering_test', 10)
        self.create_timer(0.01, self.publish_steering_test)
    def publish_steering_test(self):
        msg = SteeringReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.steering_tire_angle = math.radians(self.state_signals['steering_angle_deg']/self.steer_ratio)
        self.steering_test_pub.publish(msg)
        

    # def actuation_cb(self, msg):
    #     self.control_signals['steering_angle_deg'] = msg.actuation.steer_cmd*180/math.pi
    #     self.control_signals['accel'] = msg.actuation.accel_cmd
    #     self.control_signals['brake'] = msg.actuation.brake_cmd

    def manage_openpilot_state(self):
        self.manual_override = self.state_signals['brakePressed'] or self.state_signals['gasPressed'] #or self.state_signals['steeringPressed']

        if (self.manual_override and self.state_signals['cruiseEnabled'] and self.openpilot_state):
            self.get_logger().info("Manual override active. Disabling OpenPilot.")
            self.openpilot_state = False
            self.openpilot_disabled_time = self.get_clock().now().nanoseconds / 1e9
        elif (self.openpilot_disabled_time is not None and self.get_clock().now().nanoseconds / 1e9 - self.openpilot_disabled_time > self.openpilot_auto_reenable_timeout):
            self.get_logger().info("OpenPilot auto-reenable timeout reached. Enabling OpenPilot.")
            self.openpilot_state = True
            self.openpilot_disabled_time = None

        self.control_signals['enable'] = self.openpilot_state
        self.control_signals['latActive'] = self.openpilot_state and self.state_signals['cruiseEnabled']
        self.control_signals['longActive'] = self.openpilot_state 
        self.control_signals['cruise_cancel'] = not self.openpilot_state

        if (self.openpilot_state and self.state_signals['cruiseEnabled'] and not self.manual_override):
            self.control_signals['longControlState'] = 1  # PID - OpenPilot Longitudinal Control
        else:
            self.control_signals['longControlState'] = 0  # OFF - OpenPilot Longitudinal Control

        if self.state_signals['cruiseEnabled']:
            self.state_signals['control_mode'] = 1
        else:
            self.state_signals['control_mode'] = 4
        
        if self.openpilot_state:
            if self.state_signals['accFaulted']:
                self.control_signals['openpilot_state'] = 2 # ACC Faulted or Lock-out Condition
            else:
                self.control_signals['openpilot_state'] = 3 # Openpilot enabled or Ready to Engage
                if self.state_signals['cruiseEnabled']:
                    self.control_signals['openpilot_state'] = 4 # Openpilot Engaged
        else:
            self.control_signals['openpilot_state'] = 1 # Openpilot Disengaged


    # Service callback to enable/disable openpilot
    def enable_cb(self, request, response):
        if request.data:
            if not self.manual_override:  # Allow re-enabling only if no manual input
                self.openpilot_state = True
                self.get_logger().info("OpenPilot enabled via service")
                response.success = True
                response.message = "OpenPilot enabled."
            else:
                self.get_logger().warn("Manual override still active. Cannot enable.")
                response.success = False
                response.message = "Cannot enable: manual override active."
        else:
            self.openpilot_state = False
            self.get_logger().info("OpenPilot disabled via service")
            response.success = True
            response.message = "OpenPilot disabled."

        return response

    def control_cb(self, msg):
        self.control_signals['accel'] = msg.longitudinal.acceleration
        self.control_signals['steering_angle_deg'] = math.degrees(msg.lateral.steering_tire_angle)*self.steer_ratio
        # self.control_signals['longActive'] = True#msg.longitudinal_active

    def emergency_cb(self, msg):
        self.control_signals['emergency'] = msg.emergency

    def gear_cmd_cb(self, msg):
        self.control_signals['gear'] = msg.command

    def hazard_cmd_cb(self, msg):
        self.control_signals['hazard_lights'] = msg.command

    def turn_cmd_cb(self, msg):
        self.control_signals['turn_signal'] = msg.command

    def listen_loop(self):
        self.get_logger().info("Starting ZMQ listener loop...")
        connection_established = False

        while rclpy.ok():
            try:
                if not connection_established:
                    self.get_logger().info("Waiting for ZMQ connection to recieve carState messages")
                msg = self.sub_socket.recv()
                # Log first successful connection
                if not connection_established:
                    self.get_logger().info("ZMQ connection established - receiving carState messages")
                    connection_established = True
                # event = self.log_capnp.Event.from_bytes(msg)
                with self.log_capnp.Event.from_bytes(msg) as evt:
                    if evt.which() == 'carState':
                        car_state = getattr(evt, 'carState')
                        self.process_car_state(car_state)
            except zmq.error.ZMQError as e:
                if connection_established:
                    self.get_logger().warn(f"ZMQ connection lost: {e}")
                    connection_established = False
                else:
                    self.get_logger().error(f"ZMQ connection error: {e}")
            except Exception as e:
                self.get_logger().error(f"ZMQ receive error: {e}")

    def process_car_state(self, car_state):
        self.state_signals['vEgo'] = car_state.vEgo
        self.state_signals['steering_angle_deg'] = car_state.steeringAngleDeg
        self.state_signals['steeringPressed'] = car_state.steeringPressed
        self.state_signals['gasPressed'] = car_state.gasPressed
        self.state_signals['brakePressed'] = car_state.brakePressed
        self.state_signals['yaw_rate'] = car_state.yawRate
        self.state_signals['gear'] = car_state.gearShifter
        if car_state.leftBlinker:
            self.state_signals['turn_signal'] = 2
            if car_state.rightBlinker:
                self.state_signals['hazard_lights'] = 2
            else:
                self.state_signals['hazard_lights'] = 1
        elif car_state.rightBlinker:
            self.state_signals['turn_signal'] = 3
            self.state_signals['hazard_lights'] = 1
        else:
            self.state_signals['turn_signal'] = 1
            self.state_signals['hazard_lights'] = 1
        # self.state_signals['turn_signal'] = car_state.leftBlinker or car_state.rightBlinker
        # self.state_signals['hazard_lights'] = car_state.leftBlinker and car_state.rightBlinker
        self.state_signals['steering_angle_deg'] = car_state.steeringAngleDeg
        self.state_signals['steeringPressed'] = car_state.steeringPressed
        self.state_signals['gasPressed'] = car_state.gasPressed
        self.state_signals['brakePressed'] = car_state.brakePressed
        self.state_signals['cruiseEnabled'] = car_state.cruiseState.enabled
        self.state_signals['accFaulted'] = car_state.accFaulted

    def publish_ros_messages(self):
        self.publish_velocity()
        self.publish_steering()
        self.publish_gear()
        self.publish_turn_indicators()
        self.publish_hazard_lights()
        self.publish_control_mode()
        self.publish_openpilot_state()

    def publish_openpilot_state(self):
        msg = Int32()
        msg.data = self.control_signals['openpilot_state']
        self.openpilot_state_pub.publish(msg)

    def publish_velocity(self):
        msg = VelocityReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        vel= float(self.state_signals['vEgo'])
        if self.state_signals['gear']=='reverse':
            vel=-vel
        msg.longitudinal_velocity = vel
        msg.lateral_velocity = 0.0 #Need to change according to panda_can_rcv
        msg.heading_rate = math.radians(self.state_signals['yaw_rate']) #
        msg.header.frame_id = "base_link"
        self.vel_pub.publish(msg)

    def publish_steering(self):
        msg = SteeringReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.steering_tire_angle = math.radians(self.state_signals['steering_angle_deg']/self.steer_ratio)
        self.steer_pub.publish(msg)

    def publish_gear(self):
        msg = GearReport()
        msg.stamp = self.get_clock().now().to_msg()
        # print(self.state_signals['gear'])
        # self.get_logger().info('The error check : "%s"' % self.state_signals['gear'])
        msg.report = int(self.gear_mapping[self.state_signals['gear']])
        self.gear_pub.publish(msg)

    def publish_turn_indicators(self):
        msg = TurnIndicatorsReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.report = int(self.state_signals['turn_signal'])
        self.turn_pub.publish(msg)

    def publish_hazard_lights(self):
        msg = HazardLightsReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.report = self.state_signals['hazard_lights']
        self.hazard_pub.publish(msg)

    def publish_control_mode(self):
        msg = ControlModeReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = int(self.state_signals['control_mode'])
        self.control_mode_pub.publish(msg)

    def check_zmq_connection(self):
        """Periodically check ZMQ connection health"""
        try:
            # Check subscriber socket
            if self.sub_socket.closed:
                self.get_logger().warn("ZMQ subscriber socket is closed")
            else:
                self.get_logger().debug("ZMQ subscriber socket is healthy")
                
            # Check publisher socket  
            if self.pub_socket.closed:
                self.get_logger().warn("ZMQ publisher socket is closed")
            else:
                self.get_logger().debug("ZMQ publisher socket is healthy")
                
        except Exception as e:
            self.get_logger().error(f"ZMQ connection check failed: {e}")

    def send_car_control(self):
        try:
            self.manage_openpilot_state()
            
            event = self.log_capnp.Event.new_message()
            event.init('carControl')
            #event.carControl.actuators.accel = self.control_signals['accel']
            event.carControl.actuators.accel = self.control_signals['accel']
            event.carControl.actuators.steeringAngleDeg = self.control_signals['steering_angle_deg']
            event.carControl.enabled = self.control_signals['enable']
            event.carControl.latActive = self.control_signals['latActive']
            event.carControl.longActive = self.control_signals['longActive']
            event.carControl.actuators.longControlState = 0#self.control_signals['longControlState']

            event.carControl.hudControl.leadDistanceBars = 2

            event.carControl.cruiseControl.cancel = self.control_signals['cruise_cancel']

            self.pub_socket.send(event.to_bytes())

            if not hasattr(self, '_publishing_established'):
                self._publishing_established = True
                self.get_logger().info("ZMQ publishing established - sending carControl messages")

        except Exception as e:
            # Reset publishing status if error occurs
            if hasattr(self, '_publishing_established'):
                self._publishing_established = False
                self.get_logger().warn("ZMQ publishing lost - carControl messages not being sent")
            self.get_logger().error(f"ZMQ publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ZMQCapnpBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
