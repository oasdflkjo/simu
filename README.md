# PID Control System Simulator

## Work in Progress

This project is a prototype/proof of concept demonstrating an approach to solving PID control problems through simulation and automated parameter optimization.

### Current State
- Basic physics simulation of a ball balancing on a tilting plane
- Automated PID parameter optimization through brute force testing
- Multiprocessing support for faster simulation
- Visualization of the control system in action

### Components
- `proto.py`: Visual simulation of the control system
- `simulation.py`: PID parameter optimization through parallel processing
- `parameters.ini`: Best found PID values
- `top_results.txt`: Top 10 performing PID configurations

### Purpose
This project serves as a framework for:
1. Creating mathematical models of physical control systems
2. Testing control parameters in simulation before real-world implementation
3. Automated discovery of optimal PID values

### Note
This is not a finalized system but rather a demonstration of the approach to solving PID control problems through simulation. The current implementation uses simplified physics and may not perfectly reflect real-world conditions.

### Future Improvements
- More accurate physics simulation
- Better stability criteria
- More sophisticated parameter optimization methods
- Real-world validation of simulated results

## Usage
1. Run `simulation.py` to find optimal PID values
2. Run `proto.py` to visualize the control system in action
3. Check `top_results.txt` for alternative PID configurations 