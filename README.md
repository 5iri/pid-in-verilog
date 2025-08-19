# PID Controller in Verilog

This repository contains a complete implementation of a digital PID (Proportional-Integral-Derivative) controller in Verilog, featuring both a basic PID module and a practical line-following robot simulation.

## Features

- **Parameterizable PID controller** - Configurable data widths and fixed-point precision
- **Line-following robot simulation** - Realistic robot dynamics with smooth analog-like visualization
- **Anti-windup protection** - Prevents integral term from growing unbounded
- **Output saturation** - Configurable min/max output limits
- **Fixed-point arithmetic** - Efficient implementation for FPGA resources
- **GTKWave visualization** - Smooth analog waveform display for line-following behavior
- **Debug outputs** - Access to individual P, I, D terms for analysis

## Files

- `pid_controller.v` - Main PID controller module (digital implementation)
- `pid_controller_tb.v` - Basic testbench for PID controller
- `line_following_test.v` - Line-following robot simulation with realistic dynamics
- `line_track.gtkw` - GTKWave save file for analog-style visualization
- `Makefile` - Build and simulation scripts

## PID Controller Parameters

The controller implements the discrete PID equation:
```
u[n] = Kp * e[n] + Ki * Σe[i] + Kd * (e[n] - e[n-1])
```

Where:
- `u[n]` = control output at time n
- `e[n]` = error at time n (setpoint - process_variable)
- `Kp` = proportional gain
- `Ki` = integral gain  
- `Kd` = derivative gain

## Module Interface

```verilog
module pid_controller #(
    parameter DATA_WIDTH = 16,      // Width of data signals
    parameter FRAC_BITS = 8,        // Fractional bits for fixed-point
    parameter KP_WIDTH = 16,        // Proportional gain width
    parameter KI_WIDTH = 16,        // Integral gain width
    parameter KD_WIDTH = 16,        // Derivative gain width
    parameter OUTPUT_MIN = -32768,  // Minimum output value
    parameter OUTPUT_MAX = 32767    // Maximum output value
)(
    input wire clk,                 // Clock signal
    input wire rst_n,               // Active low reset
    input wire enable,              // Enable PID computation
    
    // PID gains (fixed-point format)
    input wire signed [KP_WIDTH-1:0] kp,
    input wire signed [KI_WIDTH-1:0] ki,
    input wire signed [KD_WIDTH-1:0] kd,
    
    // Control signals
    input wire signed [DATA_WIDTH-1:0] setpoint,
    input wire signed [DATA_WIDTH-1:0] process_var,
    
    // Output
    output reg signed [DATA_WIDTH-1:0] control_output,
    
    // Debug outputs
    output wire signed [DATA_WIDTH-1:0] error_out,
    output wire signed [DATA_WIDTH*2-1:0] p_term_out,
    output wire signed [DATA_WIDTH*2-1:0] i_term_out,
    output wire signed [DATA_WIDTH*2-1:0] d_term_out
);
```

## Fixed-Point Format

The controller uses fixed-point arithmetic for efficiency. With `FRAC_BITS = 8`:
- 1.0 is represented as 256 (0x0100)
- 0.5 is represented as 128 (0x0080)
- 2.0 is represented as 512 (0x0200)

## Usage Example

```verilog
pid_controller #(
    .DATA_WIDTH(16),
    .FRAC_BITS(8),
    .OUTPUT_MIN(0),
    .OUTPUT_MAX(4095)
) my_pid (
    .clk(clk),
    .rst_n(rst_n),
    .enable(1'b1),
    .kp(16'h0100),      // Kp = 1.0
    .ki(16'h0040),      // Ki = 0.25
    .kd(16'h0080),      // Kd = 0.5
    .setpoint(desired_value),
    .process_var(sensor_reading),
    .control_output(actuator_control),
    // Debug outputs can be left unconnected
    .error_out(),
    .p_term_out(),
    .i_term_out(),
    .d_term_out()
);
```

## Simulation

### Prerequisites
- Icarus Verilog (`brew install icarus-verilog` on macOS)
- GTKWave for viewing waveforms (`brew install gtkwave` on macOS)

### Running the simulation
```bash
# Run line-following robot simulation (default)
make line

# Run basic PID controller simulation
make sim

# View waveforms (line-following)
make view

# Clean generated files
make clean

# Get help
make help
```

## Line-Following Robot Simulation

The main feature of this project is the `line_following_test.v` simulation, which models a robot following a center line using PID control. The simulation includes:

- **Realistic robot dynamics** - Robot position, velocity, and momentum
- **Track scenarios** - Straight sections, curves, and S-curves
- **Smooth analog visualization** - Using real-valued signals for clean waveforms
- **Configurable PID gains** - Tuned specifically for line-following behavior

### Robot Model
- Robot starts 6 units off the center line (setpoint = 0)
- Includes momentum and velocity dynamics
- Responds to control signals with realistic lag
- Simulates different track conditions over time

### Visualization
The simulation generates smooth analog-like waveforms viewable in GTKWave:
- Robot position relative to center line
- Control signal (steering output) 
- Error signal
- Individual P, I, D terms
- Robot velocity and track disturbances

The `line_track.gtkw` file provides a pre-configured view optimized for line-following analysis.

## PID Tuning Guidelines

The line-following simulation uses these tuned PID parameters:
- **Kp = 2.0** - Strong proportional response for quick correction
- **Ki = 0.4** - Moderate integral action to eliminate steady-state error
- **Kd = 0.8** - Derivative action to reduce overshoot and improve stability

### General PID Tuning Process:
1. **Start with Kp only** - Set Ki=0, Kd=0, increase Kp until system oscillates
2. **Add integral term** - Set Ki to small value, increase until steady-state error is eliminated
3. **Add derivative term** - Set Kd to reduce overshoot and improve stability
4. **Fine-tune** - Adjust all gains for desired response

### Typical starting values for other applications:
- Kp: 0.5 - 2.0
- Ki: 0.1 - 0.5  
- Kd: 0.1 - 1.0

## Project Status

**Current Implementation:**
- ✅ Basic PID controller module (`pid_controller.v`)
- ✅ Line-following robot simulation with realistic dynamics
- ✅ Smooth analog-like visualization in GTKWave
- ✅ Configurable track scenarios (straight, curves, S-curves)
- ✅ Tuned PID parameters for stable line-following
- ✅ Makefile with automated simulation and visualization

**Simulation Results:**
The line-following robot successfully converges to the center line from an initial 6-unit offset, demonstrating effective PID control with realistic robot dynamics including momentum and velocity.

## Anti-Windup Protection

The controller includes anti-windup protection that prevents the integral term from accumulating when:
- Output is saturated at maximum and error is positive
- Output is saturated at minimum and error is negative

This prevents the controller from becoming unresponsive after saturation.

## FPGA Resource Usage

See the screenshot below for FPGA resource usage details.

![Screenshot](/Screenshot%202025-08-19%20at%208.08.15 PM.png)

## Applications

This PID controller implementation is suitable for:
- **Line-following robots** - As demonstrated in the included simulation
- Motor speed control
- Temperature control
- Servo positioning
- Power regulation
- Any closed-loop control system

The line-following simulation serves as a practical example showing how PID control can be applied to robotic navigation problems.

## License

This code is provided as-is for educational and commercial use.
