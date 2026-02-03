# servo_controller.py
from machine import UART, Pin
import time
import config

class ServoController:
    def __init__(self):
        self.uart = UART(config.UART_ID, 
                        baudrate=config.UART_BAUD,
                        tx=Pin(config.UART_TX),
                        rx=Pin(config.UART_RX))
        time.sleep_ms(50)
        
    def angle_to_pwm(self, angle_deg):
        """Convert angle (0-180) to PWM pulse width (500-2500µs)"""
        angle = max(0, min(180, int(angle_deg)))
        return int(500 + (angle * 2000) // 180)
    
    def apply_calibration(self, channel, angle_deg):
        """Apply servo inversion and offset calibration"""
        # Apply offset
        calibrated = float(angle_deg) + config.SERVO_OFFSET[channel]
        
        # Apply inversion if needed
        if config.SERVO_INVERT[channel]:
            calibrated = 180.0 - calibrated
            
        # Apply limits
        calibrated = max(config.SERVO_MIN_A[channel], 
                        min(config.SERVO_MAX_A[channel], calibrated))
        return calibrated
    
    def send_servo_command(self, channel, angle_deg, move_time_ms):
        """Send single servo command"""
        calibrated = self.apply_calibration(channel, angle_deg)
        pwm = self.angle_to_pwm(calibrated)
        cmd = f"#{channel:03d}P{pwm:04d}T{int(move_time_ms):04d}!"
        self.uart.write(cmd.encode())
        
        if config.UART_SPACING_MS:
            time.sleep_ms(config.UART_SPACING_MS)
    
    def send_multiple_commands(self, servo_angles, move_time_ms, chunk_size=6):
        """Send multiple servo commands efficiently"""
        for i in range(0, len(servo_angles), chunk_size):
            chunk = servo_angles[i:i + chunk_size]
            for channel, angle in chunk:
                self.send_servo_command(channel, angle, move_time_ms)
    
    def neutral_position(self, move_time_ms=240):
        """Move all servos to 90° neutral position"""
        commands = [(i, 90.0) for i in range(24)]
        self.send_multiple_commands(commands, move_time_ms)
    
    def lock_unused_servos(self, move_time_ms=300):
        """Lock unused servo channels at neutral"""
        commands = [(i, 90.0) for i in config.UNUSED_SERVOS]
        self.send_multiple_commands(commands, move_time_ms)