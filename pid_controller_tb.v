/*
 * Testbench for PID Controller
 * 
 * This testbench demonstrates the PID controller with a simple plant model
 * and verifies its step response characteristics.
 */

`timescale 1ns / 1ps

module pid_controller_tb;

    // Parameters
    parameter DATA_WIDTH = 16;
    parameter FRAC_BITS = 8;
    parameter CLK_PERIOD = 10; // 100MHz clock
    
    // Testbench signals
    reg clk;
    reg rst_n;
    reg enable;
    
    // PID gains (fixed-point: 8.8 format)
    reg signed [15:0] kp = 16'h0100;  // Kp = 1.0
    reg signed [15:0] ki = 16'h0040;  // Ki = 0.25
    reg signed [15:0] kd = 16'h0080;  // Kd = 0.5
    
    // Control signals
    reg signed [DATA_WIDTH-1:0] setpoint;
    wire signed [DATA_WIDTH-1:0] process_var;
    wire signed [DATA_WIDTH-1:0] control_output;
    
    // Debug signals
    wire signed [DATA_WIDTH-1:0] error_out;
    wire signed [DATA_WIDTH*2-1:0] p_term_out;
    wire signed [DATA_WIDTH*2-1:0] i_term_out;
    wire signed [DATA_WIDTH*2-1:0] d_term_out;
    
    // Simple plant model (first-order system)
    reg signed [DATA_WIDTH-1:0] plant_output;
    reg signed [31:0] plant_state;
    
    // Assign process variable
    assign process_var = plant_output;
    
    // Instantiate PID controller
    pid_controller #(
        .DATA_WIDTH(DATA_WIDTH),
        .FRAC_BITS(FRAC_BITS),
        .OUTPUT_MIN(-1000),
        .OUTPUT_MAX(1000)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .kp(kp),
        .ki(ki),
        .kd(kd),
        .setpoint(setpoint),
        .process_var(process_var),
        .control_output(control_output),
        .error_out(error_out),
        .p_term_out(p_term_out),
        .i_term_out(i_term_out),
        .d_term_out(d_term_out)
    );
    
    // Clock generation
    always #(CLK_PERIOD/2) clk = ~clk;
    
    // Simple plant model (integrator with gain)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            plant_state <= 0;
            plant_output <= 0;
        end else if (enable) begin
            // Simple first-order plant: G(s) = K/(Ts + 1)
            // Discrete approximation: y[n+1] = 0.9*y[n] + 0.1*u[n]
            plant_state <= (plant_state * 9) / 10 + control_output / 10;
            plant_output <= plant_state[DATA_WIDTH-1:0];
        end
    end
    
    // Test sequence
    initial begin
        // Initialize
        clk = 0;
        rst_n = 0;
        enable = 0;
        setpoint = 0;
        
        // Reset sequence
        #(CLK_PERIOD * 5);
        rst_n = 1;
        #(CLK_PERIOD * 2);
        enable = 1;
        
        $display("Starting PID Controller Test");
        $display("Time\tSetpoint\tProcess\tControl\tError\tP-term\tI-term\tD-term");
        
        // Test 1: Step response
        setpoint = 100;
        #(CLK_PERIOD * 50);
        
        // Test 2: Change setpoint
        setpoint = 200;
        #(CLK_PERIOD * 50);
        
        // Test 3: Return to zero
        setpoint = 0;
        #(CLK_PERIOD * 50);
        
        // Test 4: Negative setpoint
        setpoint = -150;
        #(CLK_PERIOD * 50);
        
        $display("Test completed");
        $finish;
    end
    
    // Monitor signals
    always @(posedge clk) begin
        if (enable) begin
            $display("%0t\t%0d\t%0d\t%0d\t%0d\t%0d\t%0d\t%0d", 
                     $time, setpoint, process_var, control_output, error_out,
                     p_term_out, i_term_out, d_term_out);
        end
    end
    
    // Generate VCD file for waveform viewing
    initial begin
        $dumpfile("pid_controller_tb.vcd");
        $dumpvars(0, pid_controller_tb);
    end

endmodule
