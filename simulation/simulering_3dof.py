
from vpython import sphere, vector, color, rate, canvas, cylinder, cos, sin, radians, slider, wtext


# Constants
g = 9.81  # Acceleration due to gravity (m/s^2)
floor_radius = 10  # Radius of the floor circle
kp, ki, kd = 0.4, 0.02, 0.8  # PID coefficients
error_sum_x, error_sum_y, last_error_x, last_error_y = 0, 0, 0, 0  # PID terms initialization
max_tilt = 10  # Maximum tilt angle in degrees
tilt_threshold = 0.01  # Threshold for considering tilt to be effectively zero

# Initialize the scene
scene = canvas(title='3D Ball Rolling Simulation', width=800, height=600, align='left')
scene.camera.pos = vector(0, -20, 30)
scene.camera.axis = vector(0, 20, -30)


# Floor setup
floor = cylinder(pos=vector(0, 0, 0), axis=vector(0, 0, -0.1), radius=floor_radius, color=color.green)

# Ball setup
ball = sphere(pos=vector(0, 0, 1), radius=0.5, color=color.blue)
ball.velocity = vector(0, 0, 0)  # Initial velocity


# Target position initially set to (5, 5)
target_position = vector(5, 5, 1)

# Controls for target position
x_slider = slider(min=-10, max=10, value=5, step=0.1, left=10, bind=lambda: update_target_position('x'))
x_title = wtext(text='Target X: 5.0\n')

y_slider = slider(min=-10, max=10, value=5, step=0.1, left=10, bind=lambda: update_target_position('y'))
y_title = wtext(text='Target Y: 5.0\n')

def update_target_position(axis):
    if axis == 'x':
        target_position.x = x_slider.value
        x_title.text = f"Target X: {x_slider.value:.1f}\n"
    elif axis == 'y':
        target_position.y = y_slider.value
        y_title.text = f"Target Y: {y_slider.value:.1f}\n"

# PID controller function
def pid_controller(target, current, error_sum, last_error, kp, ki, kd, dt):
    error = target - current
    error_sum += error * dt
    delta_error = (error - last_error) / dt
    p = kp * error
    i = ki * error_sum
    d = kd * delta_error
    output = p + i + d
    return output, error, error_sum, error  # returning current error as last_error for next iteration

# Clamp function to limit angles and apply threshold
def clamp(value, min_value, max_value, threshold):
    clamped_value = max(min_value, min(value, max_value))
    return 0 if abs(clamped_value) < threshold else clamped_value

# Main simulation update function
def update_simulation(dt):
    global error_sum_x, error_sum_y, last_error_x, last_error_y, tilt_x, tilt_y, error_x, error_y
    
    # PID control for X-axis
    output_x, error_x, error_sum_x, new_last_error_x = pid_controller(target_position.x, ball.pos.x, error_sum_x, last_error_x, kp, ki, kd, dt)
    last_error_x = new_last_error_x
    tilt_x = clamp(output_x, -max_tilt, max_tilt, tilt_threshold)

    # PID control for Y-axis
    output_y, error_y, error_sum_y, new_last_error_y = pid_controller(target_position.y, ball.pos.y, error_sum_y, last_error_y, kp, ki, kd, dt)
    last_error_y = new_last_error_y
    tilt_y = clamp(output_y, -max_tilt, max_tilt, tilt_threshold)

    # Update floor tilt based on PID outputs
    tilt_x_rad = radians(tilt_x)*10
    tilt_y_rad = radians(tilt_y)*10
    new_axis = vector(sin(tilt_x_rad), sin(tilt_y_rad), cos(tilt_x_rad)*cos(tilt_y_rad))
    floor.axis = new_axis.norm() * 0.1

    # Correct ball movement relative to floor tilt
    gravity_component_x = g * sin(tilt_x_rad) if abs(tilt_x) > tilt_threshold else 0
    gravity_component_y = g * sin(tilt_y_rad) if abs(tilt_y) > tilt_threshold else 0
    ball.velocity.x += gravity_component_x * dt
    ball.velocity.y += gravity_component_y * dt
    ball.pos.x += ball.velocity.x * dt
    ball.pos.y += ball.velocity.y * dt
    adjust_ball_z_position()

def adjust_ball_z_position():
    a = sin(radians(tilt_x))
    b = sin(radians(tilt_y))
    c = cos(radians(tilt_x)) * cos(radians(tilt_y))
    ball.pos.z = (ball.radius - a * ball.pos.x - b * ball.pos.y) / c

def get_simulation_status():
    return ball.pos.x, ball.pos.y, ball.pos.z, tilt_x, tilt_y

# Animation loop
dt = 0.05  # Time step for the simulation
while True:
    rate(20)
    update_simulation(dt)
