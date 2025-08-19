/*
 * Clean Line Following PID Controller Test
 * 
 * Simple testbench showing a robot following a center line (setpoint = 0)
 * with smooth analog-like behavior visible in GTKWave.
 */

`timescale 1ns / 1ps

module line_following_test;

    // Simple parameters
    parameter CLK_PERIOD = 100;     // 10MHz for smooth visualization
    parameter SIM_TIME = 5000000;   // 5ms simulation
    
    // Signals
    reg clk, rst_n;
    reg [31:0] time_us;
    
    // PID controller signals (using real for smooth analog display)
    real robot_position;    // Robot position relative to center line
    real setpoint;         // Always 0 (center line)
    real error;            // Position error
    real control_signal;   // Steering output
    real robot_velocity;   // Robot lateral velocity
    
    // PID gains (tuned for line following)
    real kp = 2.0, ki = 0.4, kd = 0.8;
    
    // PID state
    real integral, prev_error;
    real p_term, i_term, d_term;
    
    // Track conditions
    real track_disturbance;
    
    // Clock generation
    always #(CLK_PERIOD/2) clk = ~clk;
    
    // Time tracking in microseconds
    always @(posedge clk) begin
        if (!rst_n) 
            time_us <= 0;
        else 
            time_us <= time_us + (CLK_PERIOD/1000);
    end
    
    // Generate track scenarios (different line conditions)
    always @(*) begin
        setpoint = 0.0;  // Always center line for line following
        
        case (time_us / 1000)  // Change scenario every 1ms
            0: track_disturbance = 0.0;          // Straight start
            1: track_disturbance = -3.0;         // Left curve
            2: track_disturbance = 4.0;          // Right turn  
            3: track_disturbance = 0.0;          // Back to straight
            4: track_disturbance = 2.0 * $sin(2*3.14159*time_us/500); // S-curve
            default: track_disturbance = 0.0;    // Final straight
        endcase
    end
    
    // PID controller and robot simulation
    always @(posedge clk) begin
        if (!rst_n) begin
            robot_position <= 6.0;  // Start 6 units off center line
            robot_velocity <= 0.0;
            integral <= 0.0;
            prev_error <= 0.0;
        end else begin
            // Calculate error (distance from center line)
            error <= setpoint - robot_position;
            
            // PID calculation
            p_term <= kp * error;
            integral <= integral + error * 0.0001;  // Small dt
            i_term <= ki * integral;
            d_term <= kd * (error - prev_error) / 0.0001;
            
            control_signal <= p_term + i_term + d_term;
            prev_error <= error;
            
            // Robot dynamics (simple model with momentum)
            robot_velocity <= robot_velocity * 0.95 + control_signal * 0.05;
            robot_position <= robot_position + robot_velocity * 0.0001 + track_disturbance * 0.001;
        end
    end
    
    // Test sequence
    initial begin
        clk = 0;
        rst_n = 0;
        
        #1000;
        rst_n = 1;
        
        $display("Line Following Robot PID Test");
        $display("Robot starts 6 units off center, should converge to 0");
        
        #SIM_TIME;
        
        $display("Final position: %0.2f (target: 0.0)", robot_position);
        $finish;
    end
    
    // Generate clean VCD file
    initial begin
        $dumpfile("line_following.vcd");
        $dumpvars(0, line_following_test);
        
        // Focus on key signals for line following visualization
        $dumpvars(1, robot_position, setpoint, error, control_signal);
        $dumpvars(1, p_term, i_term, d_term, robot_velocity, track_disturbance);
        $dumpvars(1, time_us);
    end

endmodule
