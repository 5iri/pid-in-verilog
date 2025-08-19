# Makefile for PID Controller Simulation
# Clean and focused version

# Default simulator
SIM ?= iverilog

# Source files
PID_SOURCES = pid_controller.v
LINE_SOURCES = line_following_test.v
TESTBENCH = pid_controller_tb.v

# Output files  
PID_VCD = pid_controller_tb.vcd
LINE_VCD = line_following.vcd
PID_EXEC = pid_sim
LINE_EXEC = line_sim

.PHONY: all clean sim line view help

all: line

# Basic PID simulation
sim: $(PID_EXEC)
	./$(PID_EXEC)

$(PID_EXEC): $(PID_SOURCES) $(TESTBENCH)
	iverilog -o $(PID_EXEC) $(PID_SOURCES) $(TESTBENCH)

# Line following simulation with analog display
line: $(LINE_EXEC)
	./$(LINE_EXEC)
	@echo "Opening GTKWave with analog signals..."
	gtkwave $(LINE_VCD) line_track.gtkw &

$(LINE_EXEC): $(LINE_SOURCES)
	iverilog -o $(LINE_EXEC) $(LINE_SOURCES)

# View waveforms
view: $(LINE_VCD)
	gtkwave $(LINE_VCD) line_track.gtkw

# Clean generated files
clean:
	rm -f $(PID_EXEC) $(LINE_EXEC) $(PID_VCD) $(LINE_VCD)

# Help
help:
	@echo "Available targets:"
	@echo "  line    - Run line following simulation (default)"
	@echo "  sim     - Run basic PID simulation" 
	@echo "  view    - View line following waveforms"
	@echo "  clean   - Remove generated files"
	@echo "  help    - Show this help"
