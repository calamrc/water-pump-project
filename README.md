# Zephyr Water Pump Application

[![Zephyr](https://img.shields.io/badge/Zephyr-RTOS-00AEEF.svg)](https://zephyrproject.org)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

A reference implementation demonstrating **Zephyr RTOS** features with an intelligent water pump control system using flow rate feedback.

## 🚀 Quick Start

```bash
west init -m https://github.com/calamrc/water-pump-project
cd water-pump-project
west update
west build -b your_board app
west flash
```

## ✨ Latest Features (master branch)

- Full UI: Rotary encoder + display + buttons
- Auditory feedback relay (500ms pulse on countdown)
- Supervisor thread with health monitoring
- Advanced plateau detection & safety logic
- Demand-based pump control with YF-S201C sensor

## 📋 Hardware

See [doc/HARDWARE.md](doc/HARDWARE.md) for full wiring, BOM and photos.

## 🛠️ Build & Run

Full instructions in the docs folder.

## 📊 Demo

(Video coming soon)

Made with ❤️ using Zephyr RTOS