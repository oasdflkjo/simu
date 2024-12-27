import configparser
import math
import numpy as np
from multiprocessing import Pool, cpu_count
from tqdm import tqdm
from dataclasses import dataclass
from itertools import product

@dataclass
class SimulationResult:
    kp: float
    ki: float
    kd: float
    settling_time: float
    overshoot: float
    score: float
    fell_off: bool

class BallSimulation:
    def __init__(self):
        # Physics constants
        self.GRAVITY = 0.6
        self.FRICTION = 0.995
        self.PLANE_LENGTH = 800
        self.MAX_ANGLE = 45
        self.MAX_ANGLE_CHANGE_PER_TICK = 2.0  # Increased for faster response
        self.TARGET_POS = self.PLANE_LENGTH / 2
        
        # Adjusted simulation settings
        self.MAX_SIMULATION_TIME = 8.0  # Longer simulation time
        self.TIME_STEP = 1/60  # Reduced time resolution
        self.SETTLING_THRESHOLD = 20
        self.SPEED_THRESHOLD = 0.5      # Stricter speed requirement
        self.STABLE_TIME_REQUIRED = 1.0  # Require longer stability
        
    def calculate_score(self, settling_time, max_overshoot, final_error, final_speed):
        # New scoring system that considers final position and speed
        if settling_time >= self.MAX_SIMULATION_TIME:
            return float('inf')
            
        score = (
            settling_time * 1.0 +        # Base settling time
            max_overshoot * 0.01 +       # Penalize overshoot
            abs(final_error) * 0.1 +     # Penalize final position error
            abs(final_speed) * 0.5       # Penalize final speed
        )
        return score

    def run_single_simulation(self, kp, ki, kd):
        # Initial conditions
        ball_pos = self.PLANE_LENGTH / 4
        ball_speed = 0
        plane_angle = 0
        integral_error = 0
        last_error = 0
        stable_time = 0
        max_overshoot = 0
        
        steps = int(self.MAX_SIMULATION_TIME / self.TIME_STEP)
        
        # Track stability over time
        stable_positions = []  # Track last few positions
        
        for _ in range(steps):
            # Calculate error
            error = self.TARGET_POS - ball_pos
            current_overshoot = abs(error)
            max_overshoot = max(max_overshoot, current_overshoot)
            
            # PID calculations
            integral_error += error * self.TIME_STEP
            derivative = (error - last_error) / self.TIME_STEP if last_error != 0 else 0
            
            # Calculate target angle
            target_angle = (kp * error + ki * integral_error + kd * derivative)
            target_angle = max(min(target_angle, self.MAX_ANGLE), -self.MAX_ANGLE)
            
            # Apply angle rate limiting
            angle_diff = target_angle - plane_angle
            if abs(angle_diff) > self.MAX_ANGLE_CHANGE_PER_TICK:
                plane_angle += self.MAX_ANGLE_CHANGE_PER_TICK * (1 if angle_diff > 0 else -1)
            else:
                plane_angle = target_angle
            
            # Quick fall-off check
            if abs(plane_angle) > 35 or ball_pos < 0 or ball_pos > self.PLANE_LENGTH:
                return SimulationResult(
                    kp=kp, ki=ki, kd=kd,
                    settling_time=self.MAX_SIMULATION_TIME,
                    overshoot=max_overshoot,
                    score=float('inf'),
                    fell_off=True
                )
            
            # Ball physics
            ball_speed += self.GRAVITY * math.sin(math.radians(plane_angle))
            ball_speed *= self.FRICTION
            ball_pos += ball_speed
            
            # Store last few positions for oscillation detection
            stable_positions.append(ball_pos)
            if len(stable_positions) > 30:  # Keep last 0.5 seconds
                stable_positions.pop(0)
            
            # Enhanced stability check
            position_stable = abs(error) < self.SETTLING_THRESHOLD
            speed_stable = abs(ball_speed) < self.SPEED_THRESHOLD
            
            # Check for oscillations
            if len(stable_positions) >= 30:
                position_variance = np.var(stable_positions)
                oscillating = position_variance > 100  # Threshold for oscillation
            else:
                oscillating = False
            
            if position_stable and speed_stable and not oscillating:
                stable_time += self.TIME_STEP
                if stable_time >= self.STABLE_TIME_REQUIRED:
                    score = self.calculate_score(
                        settling_time=_ * self.TIME_STEP,
                        max_overshoot=max_overshoot,
                        final_error=error,
                        final_speed=ball_speed
                    )
                    return SimulationResult(
                        kp=kp, ki=ki, kd=kd,
                        settling_time=_ * self.TIME_STEP,
                        overshoot=max_overshoot,
                        score=score,
                        fell_off=False
                    )
            else:
                stable_time = 0
            
            last_error = error
        
        # Timeout case
        return SimulationResult(
            kp=kp, ki=ki, kd=kd,
            settling_time=self.MAX_SIMULATION_TIME,
            overshoot=max_overshoot,
            score=float('inf'),
            fell_off=False
        )

def simulate_combination(params):
    kp, ki, kd = params
    simulator = BallSimulation()
    return simulator.run_single_simulation(kp, ki, kd)

def main():
    # Much finer granularity for better optimization
    num_values_p = 50  # For P values
    num_values_i = 40  # For I values
    num_values_d = 50  # For D values
    
    # P value range: focus on middle range
    kp_values = np.concatenate([
        np.linspace(0.05, 0.2, num_values_p//2),    # Fine control in lower range
        np.linspace(0.2, 0.5, num_values_p//2)      # Broader range for higher values
    ])
    
    # I value range: very small values
    ki_values = np.concatenate([
        np.linspace(0.00001, 0.0001, num_values_i//2),  # Very fine control
        np.linspace(0.0001, 0.001, num_values_i//2)     # Slightly larger values
    ])
    
    # D value range: focus on damping
    kd_values = np.concatenate([
        np.linspace(0.1, 0.3, num_values_d//2),     # Fine control in useful range
        np.linspace(0.3, 0.6, num_values_d//2)      # Higher damping values
    ])
    
    # Remove duplicates and sort
    kp_values = np.unique(kp_values)
    ki_values = np.unique(ki_values)
    kd_values = np.unique(kd_values)
    
    combinations = list(product(kp_values, ki_values, kd_values))
    total_combinations = len(combinations)
    
    print(f"Testing {total_combinations} combinations using {cpu_count()} CPU cores...")
    print(f"P values: {len(kp_values)}")
    print(f"I values: {len(ki_values)}")
    print(f"D values: {len(kd_values)}")
    
    with Pool(processes=cpu_count()) as pool:
        results = list(tqdm(
            pool.imap(simulate_combination, combinations, chunksize=200),  # Increased chunk size
            total=total_combinations,
            desc="Simulating"
        ))
    
    successful_results = [r for r in results if not r.fell_off]
    best_result = min(successful_results, key=lambda x: x.score, default=None)
    
    if best_result:
        print(f"\nBest result found:")
        print(f"P: {best_result.kp:.6f}")
        print(f"I: {best_result.ki:.6f}")
        print(f"D: {best_result.kd:.6f}")
        print(f"Settling time: {best_result.settling_time:.2f}s")
        print(f"Max overshoot: {best_result.overshoot:.1f}")
        print(f"Score: {best_result.score:.2f}")
        
        # Save top 10 results to a file
        top_results = sorted(successful_results, key=lambda x: x.score)[:10]
        with open('top_results.txt', 'w') as f:
            f.write("Top 10 PID combinations:\n")
            for i, result in enumerate(top_results, 1):
                f.write(f"\n{i}. Score: {result.score:.3f}\n")
                f.write(f"P: {result.kp:.6f}\n")
                f.write(f"I: {result.ki:.6f}\n")
                f.write(f"D: {result.kd:.6f}\n")
                f.write(f"Settling time: {result.settling_time:.2f}s\n")
                f.write(f"Overshoot: {result.overshoot:.1f}\n")
        
        # Save best to parameters.ini
        config = configparser.ConfigParser()
        config['PID'] = {
            'kp': str(best_result.kp),
            'ki': str(best_result.ki),
            'kd': str(best_result.kd)
        }
        with open('parameters.ini', 'w') as configfile:
            config.write(configfile)
    
    total_successful = len(successful_results)
    print(f"\nSimulation complete!")
    print(f"Tested {total_combinations} combinations")
    print(f"Successful attempts: {total_successful} ({(total_successful/total_combinations)*100:.1f}%)")
    print(f"Failed attempts: {total_combinations - total_successful}")
    print("\nTop 10 results saved to 'top_results.txt'")

if __name__ == "__main__":
    main() 