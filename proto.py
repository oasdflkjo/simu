import pygame
import math
import time
import configparser

# Initialize Pygame
pygame.init()

# Screen settings
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Ball Balancing Simulation")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
GRAY = (128, 128, 128)

# Ball settings
BALL_RADIUS = 15
ball_speed = 0

# Plane settings
PLANE_LENGTH = SCREEN_WIDTH
PLANE_HEIGHT = 10
PLANE_ANGLE = 0.0  # Make sure this is a float
ANGLE_SPEED = 0.5
TARGET_POS = PLANE_LENGTH / 2  # Center position

# Fixed pivot point (at left edge)
pivot_x = 0
pivot_y = SCREEN_HEIGHT // 2

# Physics settings
GRAVITY = 0.6
FRICTION = 0.995  # Less friction to allow natural oscillation
RESTITUTION = 0.6

# PID Controller settings
def load_pid_values():
    config = configparser.ConfigParser()
    config.read('parameters.ini')
    return (
        float(config['PID']['kp']),
        float(config['PID']['ki']),
        float(config['PID']['kd'])
    )

class PIDController:
    def __init__(self):
        kp, ki, kd = load_pid_values()
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_error = 0
        self.last_error = 0
        self.last_time = time.time()
        self.MAX_INTEGRAL = 2000

    def adjust(self, param, delta):
        if param == 'p':
            self.kp = max(0, self.kp + delta * 100)
        elif param == 'i':
            self.ki = max(0, self.ki + delta * 100)
        elif param == 'd':
            self.kd = max(0, self.kd + delta * 100)

    def update(self, current_pos, target_pos):
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Calculate error
        error = target_pos - current_pos
        
        # Proportional term with modified error weighting
        # Much more aggressive when close to center
        if abs(error) < 50:  # Smaller zone for fine control
            error_weight = 3 + (50 - abs(error)) / 10  # More aggressive near center
        else:
            error_weight = 1 + abs(error) / 200
        p_term = self.kp * error * error_weight
        
        # Integral term - more active near center
        if abs(error) < 100:
            integral_weight = 2.0  # Stronger integral effect near center
        else:
            integral_weight = 0.5  # Reduced integral effect when far
        self.integral_error += error * dt * integral_weight
        self.integral_error = max(min(self.integral_error, self.MAX_INTEGRAL), -self.MAX_INTEGRAL)
        i_term = self.ki * self.integral_error
        
        # Derivative term focused on damping oscillations
        if dt > 0:
            derivative = (error - self.last_error) / dt
            # Stronger derivative effect when speed is high
            d_weight = 1 + abs(derivative) / 1000
            d_term = self.kd * derivative * d_weight
        else:
            d_term = 0
            
        # Update last values
        self.last_error = error
        self.last_time = current_time
        
        # Calculate total control output
        control = p_term + i_term + d_term
        
        # More aggressive angle limits
        return max(min(control, 45), -45)

# Initialize PID controller
pid = PIDController()

# Disturbance settings
IMPULSE_FORCE = 50
applying_force = False

# Clock for controlling FPS
clock = pygame.time.Clock()
FPS = 160

# Add these constants near the top with other settings
MAX_ANGLE_CHANGE_PER_TICK = 1.0  # Maximum degrees the plane can rotate per tick

def get_plane_end_point():
    """Calculate the end point of the plane based on angle"""
    angle_rad = math.radians(PLANE_ANGLE)
    end_x = pivot_x + PLANE_LENGTH * math.cos(angle_rad)
    end_y = pivot_y + PLANE_LENGTH * math.sin(angle_rad)
    return (end_x, end_y)

def get_ball_screen_position(plane_position):
    """Convert plane position to screen coordinates with ball on top of plane"""
    angle_rad = math.radians(PLANE_ANGLE)
    x = pivot_x + plane_position * math.cos(angle_rad)
    y = pivot_y + plane_position * math.sin(angle_rad)
    x += math.sin(angle_rad) * (BALL_RADIUS + PLANE_HEIGHT/2)
    y -= math.cos(angle_rad) * (BALL_RADIUS + PLANE_HEIGHT/2)
    return (x, y)

# Before the main loop, add:
last_time = time.time()

# Main game loop
running = True
ball_plane_pos = PLANE_LENGTH / 2  # Start ball in middle of plane
manual_control = False  # Toggle between manual and PI control

while running:
    screen.fill(WHITE)
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                manual_control = not manual_control
                pid.integral_error = 0  # Reset integral when switching modes
            elif event.key == pygame.K_f:
                applying_force = True
            # PID tuning controls
            elif event.key == pygame.K_q:  # Increase P
                pid.adjust('p', 0.0001)
            elif event.key == pygame.K_a:  # Decrease P
                pid.adjust('p', -0.0001)
            elif event.key == pygame.K_w:  # Increase I
                pid.adjust('i', 0.00001)
            elif event.key == pygame.K_s:  # Decrease I
                pid.adjust('i', -0.00001)
            elif event.key == pygame.K_e:  # Increase D
                pid.adjust('d', 0.0001)
            elif event.key == pygame.K_d:  # Decrease D
                pid.adjust('d', -0.0001)
    
    # Control handling
    keys = pygame.key.get_pressed()
    target_angle = PLANE_ANGLE  # Store current angle as default target
    
    if manual_control:
        if keys[pygame.K_UP]:
            target_angle = max(PLANE_ANGLE - ANGLE_SPEED, -45)
        if keys[pygame.K_DOWN]:
            target_angle = min(PLANE_ANGLE + ANGLE_SPEED, 45)
    else:
        target_angle = pid.update(ball_plane_pos, TARGET_POS)
    
    # Limit the rate of angle change
    angle_diff = target_angle - PLANE_ANGLE
    if abs(angle_diff) > MAX_ANGLE_CHANGE_PER_TICK:
        if angle_diff > 0:
            PLANE_ANGLE += MAX_ANGLE_CHANGE_PER_TICK
        else:
            PLANE_ANGLE -= MAX_ANGLE_CHANGE_PER_TICK
    else:
        PLANE_ANGLE = target_angle
    
    # Get end point of plane
    end_point = get_plane_end_point()
    
    # Ball physics
    acceleration = GRAVITY * math.sin(math.radians(PLANE_ANGLE))
    
    # Apply disturbance force if requested
    if applying_force:
        ball_speed += IMPULSE_FORCE
        applying_force = False
    
    # More realistic physics update
    ball_speed += acceleration
    
    # Only apply friction when ball is moving
    if abs(ball_speed) > 0.01:
        ball_speed *= FRICTION
    
    # Update ball position along the plane
    ball_plane_pos += ball_speed
    
    # Check plane boundaries
    if ball_plane_pos < 0:
        ball_plane_pos = 0
        ball_speed *= -RESTITUTION
    elif ball_plane_pos > PLANE_LENGTH:
        ball_plane_pos = PLANE_LENGTH
        ball_speed *= -RESTITUTION
    
    # Convert ball position to screen coordinates
    ball_x, ball_y = get_ball_screen_position(ball_plane_pos)
    
    # Draw the plane
    pygame.draw.line(screen, BLACK, (pivot_x, pivot_y), end_point, PLANE_HEIGHT)
    
    # Draw the ball
    pygame.draw.circle(screen, RED, (int(ball_x), int(ball_y)), BALL_RADIUS)
    
    # Draw the target position
    target_x, target_y = get_ball_screen_position(TARGET_POS)
    pygame.draw.line(screen, GREEN, (target_x, target_y - 20), 
                    (target_x, target_y + 20), 2)
    
    # Display information
    font = pygame.font.SysFont(None, 24)
    mode = "Manual" if manual_control else "PID Control"
    info_text = [
        f"Mode: {mode}",
        f"Angle: {PLANE_ANGLE:.1f}°  Speed: {ball_speed:.1f}",
        f"P: {pid.kp:.6f} (Q/A)",
        f"I: {pid.ki:.6f} (W/S)",
        f"D: {pid.kd:.6f} (E/D)"
    ]
    
    for i, text in enumerate(info_text):
        text_surface = font.render(text, True, BLUE)
        screen.blit(text_surface, (10, 10 + i * 25))
    
    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
