from .serialprotocol import Connection, build_packet
import time
import rospy

DEFAULT_ROVER_PORT = '/dev/ttyACM0'
DEFAULT_ROVER_BAUDRATE = 115200

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
                