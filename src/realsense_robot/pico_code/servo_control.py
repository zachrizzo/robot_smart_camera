from machine import Pin, PWM
import json
import sys

# Configure servo PWM
servo_pin = 15  # GP15 - you can change this to any suitable GPIO pin
servo = PWM(Pin(servo_pin))
servo.freq(50)  # Standard servo frequency is 50Hz

# Function to convert angle (0-180) to duty cycle (roughly 1000-9000)
def angle_to_duty(angle):
    min_duty = 1000  # Corresponds to 0 degrees
    max_duty = 9000  # Corresponds to 180 degrees
    return min_duty + (max_duty - min_duty) * angle / 180

# Initialize servo to middle position
current_angle = 90
servo.duty_u16(int(angle_to_duty(current_angle)))

# Main loop
while True:
    try:
        if sys.stdin.in_waiting():  # Check if there's data available
            line = sys.stdin.readline().strip()
            try:
                data = json.loads(line)
                if 'cmd' in data and data['cmd'] == 'servo':
                    angle = float(data['angle'])
                    # Ensure angle is within bounds
                    angle = max(0, min(180, angle))
                    # Update servo position
                    servo.duty_u16(int(angle_to_duty(angle)))
                    current_angle = angle
                    # Send back current position
                    print(json.dumps({'position': current_angle}))
            except ValueError:
                pass
    except Exception as e:
        pass 