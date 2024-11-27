import rclpy

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from enum import Enum
from threading import Thread

from geopy.point import Point
from geopy.distance import distance
from geopy.distance import geodesic

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue

from px4_msgs.msg import VehicleCommand, OffboardControlMode, \
                            VehicleAttitudeSetpoint, VehicleRatesSetpoint, TrajectorySetpoint, \
                            VehicleGlobalPosition, VtolVehicleStatus, VehicleStatus, AirspeedValidated

class State(Enum):
    PREARM = 1
    TAKEOFF_MC = 2
    TAKEOFF_TRANSITIONED = 3
    FW = 4
    OFFBOARD = 5
    LANDING_LOITER = 6
    LANDING_ALT_LOITER = 7
    LANDING_APPROACH = 8
    LANDING_SLOW_DOWN = 9
    LANDING_TRANSITION = 10
    LANDING_RTL = 11
    LANDING_FINAL = 12

class PX4Whisperer(Node):

    def __init__(self):
        super().__init__('PX4Whisperer')

        # Check and initalize clock
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        if use_sim_time:
            self.get_logger().info("Simulated time is enabled.")
        else:
            self.get_logger().info("Simulated time is disabled.")
        self.clock = Clock()

        # Home position
        self.home_lat=47.397971057728974
        self.home_lon=8.546163739800146
        self.home_alt=0.0
        # Desired take-off/landing approach bearing
        self.des_bear = 20.0 

        # Timing variables
        self.time_of_last_command_ms = None
        self.time_of_fw_transition_ms = None
        self.time_of_offboard_start_ms = None

        # FSM
        self.aircraft_state = State.PREARM

        # Publishers
        qos_profile = QoSProfile(
                            depth=10,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL # or DurabilityPolicy.VOLATILE
                            )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            "/Drone1/fmu/in/vehicle_command",
            qos_profile)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/Drone1/fmu/in/offboard_control_mode",
            qos_profile)
        self.attitude_ref_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            "/Drone1/fmu/in/vehicle_attitude_setpoint",
            qos_profile)
        self.rates_ref_pub = self.create_publisher(
            VehicleRatesSetpoint,
            "/Drone1/fmu/in/vehicle_rates_setpoint",
            qos_profile)
        self.traj_ref_pub = self.create_publisher(
            TrajectorySetpoint,
            "/Drone1/fmu/in/trajectory_setpoint",
            qos_profile)

        # Callback groups
        self.callback_group_subscriber = ReentrantCallbackGroup()  # Listen to subscribers in parallel
        self.callback_group_service = MutuallyExclusiveCallbackGroup()  # Only one service call at a time

        # Subscribers
        qos_profile = QoSProfile(
                            history=HistoryPolicy.KEEP_LAST,
                            depth=10,
                            reliability=ReliabilityPolicy.BEST_EFFORT
                            )
        self.subscription = self.create_subscription(
            VehicleGlobalPosition,
            "/Drone1/fmu/out/vehicle_global_position",
            self.subscribe_callback_pos,
            qos_profile,
            callback_group=self.callback_group_subscriber)
        self.subscription = self.create_subscription(
            VtolVehicleStatus,
            "/Drone1/fmu/out/vtol_vehicle_status",
            self.subscribe_callback_vtol_status,
            qos_profile,
            callback_group=self.callback_group_subscriber)
        self.subscription = self.create_subscription(
            VehicleStatus,
            "/Drone1/fmu/out/vehicle_status",
            self.subscribe_callback_status,
            qos_profile,
            callback_group=self.callback_group_subscriber)
        self.subscription = self.create_subscription(
            AirspeedValidated,
            "/Drone1/fmu/out/airspeed_validated",
            self.subscribe_callback_airspeed_validated,
            qos_profile,
            callback_group=self.callback_group_subscriber)

        # Services
        self.srv = self.create_service(
            GetParameters, 'do_the_thing', 
            self.service_callback,
            # self.threaded_service_callback,
            callback_group=self.callback_group_service)

        # Status/sensors readings
        self.lat = -1
        self.lon = -1
        self.alt = -1
        self.vtol_status = -1
        self.arming_state = -1
        self.tas_validated = -1

        self.printer_counter = 0

    def subscribe_callback_vtol_status(self, msg):
        self.vtol_status = msg.vehicle_vtol_state

    def subscribe_callback_status(self, msg):
        self.arming_state = msg.arming_state

    def subscribe_callback_airspeed_validated(self, msg):
        self.tas_validated = msg.true_airspeed_m_s

    def subscribe_callback_pos(self, msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt

        if self.printer_counter % 400 == 0:
            self.get_logger().info(f'Pos: {self.lat:.2f} {self.lon:.2f} {self.alt:.2f} TAS: {self.tas_validated:.2f} VTOL Status: {self.vtol_status}')
        self.printer_counter += 1

    # def threaded_service_callback(self, request, response): # alternatively, use different callback groups for subscribers and services
    #     self.service_thread = Thread(target=self.service_callback, args=(request, response))
    #     self.service_thread.start()
    #     return response

    def service_callback(self, request, response):
        self.get_logger().info(f'Start service: {request.names[0]}')
        ans = "Jolly good, old chap!"

        if request.names[0] == 'takeoff':
            self.takeoff_request = True
            while self.takeoff_request:
                current_time_ms = self.clock.now().nanoseconds / 1e6
                if self.aircraft_state == State.PREARM:
                    self.do_takeoff(alt=60, yaw=self.des_bear + 180.0) # Note: desired yaw requires custom PX4
                    self.change_aircraft_state(State.TAKEOFF_MC)
                elif self.aircraft_state == State.TAKEOFF_MC:
                    if self.vtol_status == 4:
                        self.change_aircraft_state(State.TAKEOFF_TRANSITIONED)
                elif self.aircraft_state == State.TAKEOFF_TRANSITIONED and (current_time_ms > (self.time_of_fw_transition_ms + 10.0*1e3)):
                    des_lat, des_lon = self.get_coord_from_cart(x_offset=500.0, y_offset=-150.0)
                    self.do_orbit(lat=des_lat, lon=des_lon, alt=100.0, r=300.0, speed=15.0)
                    self.change_aircraft_state(State.FW)
                    self.takeoff_request = False

        elif request.names[0] == 'change_alt':
            self.do_change_altitude(alt=150.0)
            self.change_aircraft_state(State.FW)

        elif request.names[0] == 'change_speed':
            self.do_change_speed(speed=17.5)
            self.change_aircraft_state(State.FW)

        elif request.names[0] == 'reposition':
            des_lat, des_lon = self.get_coord_from_cart(x_offset=-3000.0, y_offset=300.0)
            self.do_reposition(lat=des_lat, lon=des_lon, alt=200.0)
            self.change_aircraft_state(State.FW)

        elif request.names[0] == 'roll':
            self.roll_request = True
            maneuver_time = 0.33
            while self.roll_request:
                current_time_ms = self.clock.now().nanoseconds / 1e6
                if self.time_of_offboard_start_ms is None:
                    self.time_of_offboard_start_ms = current_time_ms
                if (current_time_ms < (self.time_of_offboard_start_ms + maneuver_time*1e3)):
                    self.do_offboard(off_type='rat', roll=4.0, thrust=0.5) # aileron roll
                    self.aircraft_state = State.OFFBOARD  
                else:          
                    self.change_aircraft_state(State.FW)
                    self.time_of_offboard_start_ms = None
                    self.roll_request = False

        elif request.names[0] == 'dive':
            self.dive_request = True
            maneuver_time = 1.0
            while self.dive_request:
                current_time_ms = self.clock.now().nanoseconds / 1e6
                if self.time_of_offboard_start_ms is None:
                    self.time_of_offboard_start_ms = current_time_ms
                if (current_time_ms < (self.time_of_offboard_start_ms + maneuver_time*1e3)):
                    self.do_offboard(off_type='att', pitch=-20.0, thrust=0.5) # dive
                    self.aircraft_state = State.OFFBOARD  
                else:          
                    self.change_aircraft_state(State.FW)
                    self.time_of_offboard_start_ms = None
                    self.dive_request = False

        elif request.names[0] == 'traj':
            self.traj_request = True
            maneuver_time = 5.0
            while self.traj_request:
                current_time_ms = self.clock.now().nanoseconds / 1e6
                if self.time_of_offboard_start_ms is None:
                    self.time_of_offboard_start_ms = current_time_ms
                if (current_time_ms < (self.time_of_offboard_start_ms + maneuver_time*1e3)):
                    self.do_offboard(off_type='traj', ref=[250.0, -500.0, 0.0]) # see TrajectorySetpoint.msg, NED local world frame
                    self.aircraft_state = State.OFFBOARD  
                else:
                    self.change_aircraft_state(State.FW)
                    self.time_of_offboard_start_ms = None
                    self.traj_request = False

        elif request.names[0] == 'land':
            self.land_request = True
            while self.land_request:
                if self.aircraft_state == State.FW:
                    des_lat, des_lon = self.get_coord_from_polar(dist=500.0,bear=self.des_bear-26.6)
                    self.do_orbit(lat=des_lat, lon=des_lon, alt=150.0, r=250.0, speed=15.0)
                    self.change_aircraft_state(State.LANDING_LOITER)

                elif self.aircraft_state == State.LANDING_LOITER:
                    des_lat, des_lon = self.get_coord_from_polar(dist=500.0,bear=self.des_bear-26.6)
                    distance_in_meters = geodesic((self.lat, self.lon), (des_lat, des_lon)).meters
                    if distance_in_meters < 300.0 and distance_in_meters > 200.0:
                        self.do_orbit(lat=des_lat, lon=des_lon, alt=80.0, r=250.0, speed=15.0)
                        self.change_aircraft_state(State.LANDING_ALT_LOITER)

                elif self.aircraft_state == State.LANDING_ALT_LOITER:
                    exit_lat, exit_lon = self.get_coord_from_polar(dist=500.0,bear=self.des_bear)
                    distance_from_exit_in_meters = geodesic((self.lat, self.lon), (exit_lat, exit_lon)).meters
                    if distance_from_exit_in_meters < 30.0 and abs(self.alt - (self.home_alt + 80.0)) < 10.0:
                        des_lat, des_lon = self.get_coord_from_polar(dist=500.0,bear=self.des_bear+180.0)
                        self.do_reposition(lat=des_lat, lon=des_lon, alt=60.0)
                        self.change_aircraft_state(State.LANDING_APPROACH)

                elif self.aircraft_state == State.LANDING_APPROACH:
                    self.do_change_speed(speed=13.0)
                    self.change_aircraft_state(State.LANDING_SLOW_DOWN)

                elif self.aircraft_state == State.LANDING_SLOW_DOWN:
                    distance_in_meters = geodesic((self.lat, self.lon), (self.home_lat, self.home_lon)).meters
                    if distance_in_meters < 100.0: # might miss if the loiter exit was in the wrong place
                        self.do_vtol_transition(trans_type=3.0)
                        self.change_aircraft_state(State.LANDING_TRANSITION)

                elif self.aircraft_state == State.LANDING_TRANSITION and self.vtol_status == 3:
                    # self.do_rtl()
                    self.do_reposition(lat=self.home_lat, lon=self.home_lon, alt=60.0)
                    self.change_aircraft_state(State.LANDING_RTL)

                elif self.aircraft_state == State.LANDING_RTL:
                    distance_in_meters = geodesic((self.lat, self.lon), (self.home_lat, self.home_lon)).meters
                    if distance_in_meters < 3.0:
                        self.do_land()
                        self.change_aircraft_state(State.LANDING_FINAL)
                        self.land_request = False

        else:
            self.get_logger().info("Invalid /do_the_thing request")
            ans = "It's dead, Jim"

        param_value = ParameterValue()
        param_value.string_value = ans
        response.values.append(param_value)
        return response

    def change_aircraft_state(self, new_state):
        if (self.aircraft_state == State.TAKEOFF_MC) and (new_state == State.TAKEOFF_TRANSITIONED):
            self.time_of_fw_transition_ms = self.clock.now().nanoseconds / 1e6
        else:
            self.time_of_last_command_ms = self.clock.now().nanoseconds / 1e6
        self.aircraft_state = new_state

    def do_takeoff(self, alt=50.0, yaw=0.0):
        for i in range(3):
            self.send_vehicle_command(
                                    400, # MAV_CMD_COMPONENT_ARM_DISARM
                                    param1=1.0, # arm
                                    param2=21196.0, # force
                                    conf=i if i <= 255 else 255
                                    )
        self.send_vehicle_command(
                                84, # VEHICLE_CMD_NAV_VTOL_TAKEOFF
                                param2=3.0, # takeoff mode 3: specified (custom PX4 only)
                                param4=yaw, # unused heading https://docs.px4.io/main/en/flight_modes_vtol/mission.html
                                # MAV_CMD_NAV_VTOL_TAKEOFF.param2 is ignored, heading to the next wp is used for the transition heading.
                                # Custom PX4 fix in navigator_main.cpp and vtol_takeoff.cpp circumvents this
                                param5=self.home_lat,
                                param6=self.home_lon,
                                param7=self.home_alt + alt,
                                conf=0
                                )

    def do_orbit(self, lat, lon, alt=100.0, r=300.0, speed=15.0, loops=0.0):
        self.send_vehicle_command(
                                34, # VEHICLE_CMD_DO_ORBIT
                                param1=r, # radius
                                param2=speed, # speed
                                param4=loops, # orbits in radians, 0 is forever
                                param5=lat,
                                param6=lon,
                                param7=self.home_alt + alt,
                                conf=0
                                )

    def do_change_altitude(self, alt=100.0):
        self.send_vehicle_command(
                                186, # VEHICLE_CMD_DO_CHANGE_ALTITUDE
                                param1=self.home_alt + alt,
                                param2=1.0, # global frame
                                conf=0
                                ) 

    def do_change_speed(self, speed=15.0):
        self.send_vehicle_command(
                                178, # VEHICLE_CMD_DO_CHANGE_SPEED
                                param1=0.0, # airspeed
                                param2=speed, # setpoint
                                conf=0
                                )

    def do_reposition(self, lat, lon, alt=100.0):
        self.send_vehicle_command(
                                192, # MAV_CMD_DO_REPOSITION
                                param5=lat,
                                param6=lon,
                                param7=self.home_alt + alt,
                                conf=0
                                )

    def do_vtol_transition(self, trans_type):
        self.send_vehicle_command(
                                3000, # VEHICLE_CMD_DO_VTOL_TRANSITION
                                # vtol_att_control_main.cpp denies transition from MC to FW in Takeoff, Land, RTL and Orbit
                                param1=trans_type, # 1: to fw, 2: to mc, 3: mc, 4: fw
                                param2=0.0,
                                conf=0
                                )
    def do_rtl(self):
        self.send_vehicle_command(
                                20, # VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
                                conf=0
                                )

    def do_land(self):
        self.send_vehicle_command(
                                21, # VEHICLE_CMD_NAV_LAND
                                param5=self.home_lat,
                                param6=self.home_lon,
                                param7=self.home_alt, # param7 is ground altitude
                                conf=0
                                )

    def do_offboard(self, off_type, roll=None, pitch=None, ref=None, thrust=0.5):
        current_time_ms = self.clock.now().nanoseconds / 1e6
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = int(current_time_ms)
        if off_type == 'att':
            offboard_mode.attitude = True
            attitude_ref = VehicleAttitudeSetpoint()
            attitude_ref.timestamp = int(current_time_ms)
            if roll is not None:
                attitude_ref.roll_body = roll
            if pitch is not None:
                attitude_ref.pitch_body = pitch
            attitude_ref.thrust_body = [thrust, 0.0, 0.0]
            self.attitude_ref_pub.publish(attitude_ref)
        elif off_type == 'rat':
            offboard_mode.body_rate = True
            rates_ref = VehicleRatesSetpoint()
            rates_ref.timestamp = int(current_time_ms)
            if roll is not None:
                rates_ref.roll = roll # rad/s
            if pitch is not None:
                rates_ref.pitch = pitch # rad/s
            rates_ref.thrust_body = [thrust, 0.0, 0.0]
            self.rates_ref_pub.publish(rates_ref)
        elif off_type == 'traj':
            offboard_mode.position = True
            traj_ref = TrajectorySetpoint()
            traj_ref.timestamp = int(current_time_ms)
            traj_ref.velocity = ref
            self.traj_ref_pub.publish(traj_ref)
        else:
            print("Unsupported offboard mode")
            exit()
        self.offboard_mode_pub.publish(offboard_mode)
        self.send_vehicle_command(
                            176, # MAV_CMD_DO_SET_MODE
                            param1=1.0,
                            param2=6.0, # TODO not needed for attitude/rates references
                            conf=0
                            )

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0, conf=0):
        vehicle_command = VehicleCommand()
        vehicle_command.command = command
        current_time_ms = self.clock.now().nanoseconds / 1e6
        vehicle_command.timestamp = int(current_time_ms)
        vehicle_command.param1 = param1
        vehicle_command.param2 = param2
        vehicle_command.param3 = param3
        vehicle_command.param4 = param4
        vehicle_command.param5 = param5  # Latitude
        vehicle_command.param6 = param6  # Longitude
        vehicle_command.param7 = param7  # Altitude
        vehicle_command.target_system = 1 # for SITL, this is the PX4 instance (from 0) plus 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 255
        vehicle_command.source_component = 0
        vehicle_command.confirmation = conf
        vehicle_command.from_external = True

        self.command_pub.publish(vehicle_command)
        if command != 176: # do not spam set_mode
            self.get_logger().info(f"Sent VehicleCommand: {command}")

    def get_coord_from_cart(self, x_offset, y_offset):
        start_point = Point(self.home_lat, self.home_lon)
        new_point = distance(meters=abs(y_offset)).destination(start_point, bearing=0 if y_offset >= 0 else 180) # north-south offset, positive y means north, negative means south
        new_point = distance(meters=abs(x_offset)).destination(new_point, bearing=90 if x_offset >= 0 else 270) # east-west offset offset. Positive x means east, negative means west
        return new_point.latitude, new_point.longitude

    def get_coord_from_polar(self, dist, bear):
        start_point = Point(self.home_lat, self.home_lon)
        new_point = distance(meters=dist).destination(start_point, bearing=bear)
        return new_point.latitude, new_point.longitude

def main(args=None):
    rclpy.init(args=args)
    px4 = PX4Whisperer()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(px4)
    executor.spin()

    px4.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
