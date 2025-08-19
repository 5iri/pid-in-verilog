/*
 * PID Controller in Verilog
 * 
 * This module implements a digital PID controller with the following features:
 * - Configurable Kp, Ki, Kd gains
 * - Anti-windup protection for integral term
 * - Saturation protection for output
 * - Fixed-point arithmetic for precision
 */

module pid_controller #(
    parameter DATA_WIDTH = 16,      // Width of data signals
    parameter FRAC_BITS = 8,        // Number of fractional bits for fixed-point
    parameter KP_WIDTH = 16,        // Width of proportional gain
    parameter KI_WIDTH = 16,        // Width of integral gain
    parameter KD_WIDTH = 16,        // Width of derivative gain
    parameter OUTPUT_MIN = -32768,  // Minimum output value
    parameter OUTPUT_MAX = 32767    // Maximum output value
)(
    input wire clk,                 // Clock signal
    input wire rst_n,               // Active low reset
    input wire enable,              // Enable PID computation
    
    // PID gains (fixed-point format)
    input wire signed [KP_WIDTH-1:0] kp,  // Proportional gain
    input wire signed [KI_WIDTH-1:0] ki,  // Integral gain
    input wire signed [KD_WIDTH-1:0] kd,  // Derivative gain
    
    // Control signals
    input wire signed [DATA_WIDTH-1:0] setpoint,    // Desired value
    input wire signed [DATA_WIDTH-1:0] process_var, // Current process variable
    
    // Output
    output reg signed [DATA_WIDTH-1:0] control_output,
    
    // Debug outputs (optional)
    output wire signed [DATA_WIDTH-1:0] error_out,
    output wire signed [DATA_WIDTH*2-1:0] p_term_out,
    output wire signed [DATA_WIDTH*2-1:0] i_term_out,
    output wire signed [DATA_WIDTH*2-1:0] d_term_out
);

    // Internal signals
    reg signed [DATA_WIDTH-1:0] error, prev_error;
    reg signed [DATA_WIDTH*2-1:0] integral;
    reg signed [DATA_WIDTH-1:0] derivative;
    
    // PID terms
    wire signed [DATA_WIDTH*2-1:0] p_term, i_term, d_term;
    wire signed [DATA_WIDTH*2-1:0] pid_sum;
    
    // Intermediate calculations
    wire signed [DATA_WIDTH*2-1:0] kp_mult, ki_mult, kd_mult;
    wire signed [DATA_WIDTH*2-1:0] integral_next;
    
    // Calculate error
    always @(*) begin
        error = setpoint - process_var;
    end
    
    // Multiply gains with respective terms
    assign kp_mult = (kp * error) >>> FRAC_BITS;
    assign ki_mult = (ki * integral) >>> (FRAC_BITS * 2);
    assign kd_mult = (kd * derivative) >>> FRAC_BITS;
    
    // PID terms
    assign p_term = kp_mult;
    assign i_term = ki_mult;
    assign d_term = kd_mult;
    
    // Sum all terms
    assign pid_sum = p_term + i_term + d_term;
    
    // Integral calculation with anti-windup
    assign integral_next = integral + (error <<< FRAC_BITS);
    
    // Main PID computation
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_error <= 0;
            integral <= 0;
            derivative <= 0;
            control_output <= 0;
        end else if (enable) begin
            // Update previous error
            prev_error <= error;
            
            // Calculate derivative
            derivative <= error - prev_error;
            
            // Update integral with anti-windup protection
            if ((pid_sum >= OUTPUT_MIN && pid_sum <= OUTPUT_MAX) || 
                (pid_sum < OUTPUT_MIN && error < 0) || 
                (pid_sum > OUTPUT_MAX && error > 0)) begin
                integral <= integral_next;
            end
            // If output is saturated and error would increase saturation, don't update integral
            
            // Apply output saturation
            if (pid_sum > OUTPUT_MAX) begin
                control_output <= OUTPUT_MAX;
            end else if (pid_sum < OUTPUT_MIN) begin
                control_output <= OUTPUT_MIN;
            end else begin
                control_output <= pid_sum[DATA_WIDTH-1:0];
            end
        end
    end
    
    // Debug outputs
    assign error_out = error;
    assign p_term_out = p_term;
    assign i_term_out = i_term;
    assign d_term_out = d_term;

endmodule
