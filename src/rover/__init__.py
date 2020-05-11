from .serialprotocol import Connection, build_packet
import time
import rospy
import threading 

DEFAULT_ROVER_PORT = '/dev/ttyACM0'
DEFAULT_ROVER_BAUDRATE = 115200

class MotorStateController:
    def __init__(self, rover):
        self.motor_current_phase = 0
        self.motor_state = 0 # 0 Idle, 1 = Executing      
        self.rover = rover

    def move( self, intervals, motor_values ):
        if self.motor_state == 0:
            self.phase_parameters = [
                [ 0, [ motor_values[0][0], motor_values[0][1] ] ],
                [ intervals[0], [ motor_values[1][0], motor_values[1][1] ] ],
                [ intervals[1], [0, 0] ]
            ]
            self.motor_state = 1
            self.current_phase = 0      
            start_time = threading.Timer(  
                self.phase_parameters[self.current_phase][0], 
                self.execute_phase )
            start_time.start()  
            return True
        else:
            return False

    def execute_phase(self):
        self.rover.update_motor_speeds(
            self.phase_parameters[self.current_phase][1], RoverClient.MOTOR_A | RoverClient.MOTOR_B
        )        
        self.current_phase += 1
        if self.current_phase < len(self.phase_parameters):
            start_time = threading.Timer( 
                self.phase_parameters[self.current_phase][0], 
                self.execute_phase )
            start_time.start()  
        else:
            self.motor_state = 0



class RoverClient(Connection):        
    CMD_REQUEST_TMY = 0
    CMD_LED_ON = 1
    CMD_LED_OFF = 2
    CMD_UPDATE_MOTOR_SPEEDS = 3

    MOTOR_A = 0x01
    MOTOR_B = 0x02

    def __init__(self, port, baudrate, report_handlers):  
        Connection.__init__(self,port,baudrate, self.on_data_received)
        self.report_handlers = report_handlers
        self.motor_state_ctl = MotorStateController(self)

    # Basic Protocol Leds and TMY
    def request_tmy(self, report_type=0):
        pkt = build_packet([
            RoverClient.CMD_REQUEST_TMY,
            report_type
            ])
        self.send_packet(pkt)

    def led_on(self):
        pkt = build_packet([RoverClient.CMD_LED_ON])
        self.send_packet(pkt)
                
    def led_off(self):
        pkt = build_packet([RoverClient.CMD_LED_OFF])
        self.send_packet(pkt)

    # Rover specific

    def update_motor_speeds(self, speeds, flags ):
        """
            speeds[2] each speed -255 to 255
        """
        pkt = build_packet(
            [
                RoverClient.CMD_UPDATE_MOTOR_SPEEDS,
                (speeds[0] >> 8) & 0xFF,
                (speeds[0] >> 0) & 0xFF,
                (speeds[1] >> 8) & 0xFF,
                (speeds[1] >> 0) & 0xFF,
                flags
            ] )
        self.send_packet(pkt)

    # Response handler
    def on_data_received(self, payload):
        if(len(payload)>1):
            report_id = payload[0] - 0x80
            if report_id < len(self.report_handlers):
                self.report_handlers[report_id](payload)
            else:
                rospy.logerror("Unknown report type: ", report_id)
                

    def move(self,intervals, motor_speeds):
        """
        mpe.move( 
            [ 0.5, 5], 
            [
                [100, 100],
                [50, 50]
            ]
        )                
        """
        return self.motor_state_ctl.move(intervals, motor_speeds)

