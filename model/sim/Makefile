# Makefile for the host-side water pump simulator (great for macOS)
#
# This builds a native binary that runs the real firmware logic
# (flow_processor + pump_state_machine) against plant-generated traces.

APP_DIR := ../../app

CFLAGS := -I$(APP_DIR)/include -I$(APP_DIR)/include/zbus -I$(APP_DIR)/src -Wall -O2 -g

all: host_sim

host_sim: host_sim.c host_shims.c
	$(CC) $(CFLAGS) host_sim.c host_shims.c \
		$(APP_DIR)/src/flow_processor.c \
		$(APP_DIR)/src/pump_state_machine.c \
		$(APP_DIR)/src/pump/pump_demand.c \
		$(APP_DIR)/src/pump/flow_stabilization_contract.c \
		$(APP_DIR)/src/flow_analyzer_core.c \
		-o host_sim -lm
# PR4: pures sufficient (demand+contract+sm+processor+analyzer_core); timer pure
# is header-only (pump_data_types.h) and passed as NULLable to demand_evaluate.

clean:
	rm -f host_sim

# Run with a trace file (generate one first with the Python plant model)
run: host_sim
	@if [ -f plant_trace.csv ]; then \
		./host_sim plant_trace.csv; \
	else \
		echo "No plant_trace.csv found. Generate one with:"; \
		echo "  cd ../plant && python generate_pulse_train.py --output-csv ../../model/sim/plant_trace.csv"; \
		echo "Then run 'make run' again."; \
	fi

# Quick end-to-end: generate a watering trace and run the simulator
demo:
	cd ../plant && python generate_pulse_train.py --scenario watering --duration 30 \
		--output-csv ../../model/sim/plant_trace.csv
	$(MAKE) -C . run

help:
	@echo "Common targets:"
	@echo "  make            - build host_sim"
	@echo "  make run        - run with plant_trace.csv if present"
	@echo "  make demo       - generate a fresh trace + run simulation"
	@echo "  ./host_sim foo.csv   - run against any plant trace"
