# Opencode Agent Instructions for Zephyr Water Pump Project

## Build Instructions

### Prerequisites

- Zephyr development environment set up
- Python virtual environment at `../../venv`

### Build Command

```bash
source ../../venv/bin/activate
west build -p always -b esp32_devkitc/esp32/procpu app
```

### Flash Command

```bash
west flash
```

## Development Workflow

### Build Process
- **Always wait for builds to complete** before proceeding with changes
- **Fix all compilation errors immediately** - do not proceed until the code compiles successfully

