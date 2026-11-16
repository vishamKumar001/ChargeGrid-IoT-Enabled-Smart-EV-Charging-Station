# ChargeGrid – IoT-Enabled Smart EV Charging Station

An IoT-enabled dual-port EV charging station featuring grid-aware energy management, MQTT-based communication, and priority-based charging allocation.

## Project Overview

ChargeGrid is a smart EV charging management system designed to coordinate multiple charging ports under limited grid power availability.

The system integrates ESP32-based grid monitoring, voltage and current sensing, Wi-Fi communication using MQTT, and a MATLAB-based energy management model to dynamically manage charging priorities.

The proposed approach considers estimated remaining charging time and waiting time to prioritize EV charging while respecting available source-power constraints.

## Key Features

- ESP32-based grid voltage and current monitoring.
- MQTT-based wireless communication between monitoring and charging subsystems.
- Dual-port EV battery charging architecture.
- Priority-based charging allocation using remaining charging time and waiting time.
- Grid-status classification into NORMAL, CONGESTION, and OVERLOAD.
- Adaptive port switching between 1C, 0.5C, and OFF charging states.
- MATLAB-based energy management and charging allocation simulation.
- Parameter optimization using MATLAB alpha-beta weight sweep.

## System Architecture

The system consists of two major subsystems:

### 1. Grid Monitoring Unit
- ESP32 microcontroller.
- Voltage and current sensing.
- Grid power estimation.
- Grid-status classification.
- MQTT communication over Wi-Fi.

### 2. Dual-Port Charging Station
- Two independent charging channels.
- Battery voltage and current monitoring.
- State of Charge (SOC) estimation.
- Priority-based charging allocation.
- Grid-aware charging control.

## Technologies Used

| Category | Technologies |
|---|---|
| Microcontroller | ESP32 |
| Programming | Embedded C / C++ |
| Communication | Wi-Fi, MQTT |
| Sensors | Voltage and Current Sensors |
| Simulation | MATLAB |
| Energy Management | Priority-Based Allocation |
| Battery Monitoring | SOC Estimation |
| Charging Control | 1C / 0.5C / OFF |

## Charging Management Strategy

ChargeGrid calculates a priority score for each eligible EV based on estimated remaining charging time and waiting time.

The priority function is:

S_i = α(T_scale / T_remaining,i) + β min(W_i / W_scale, 1)

The MATLAB parameter sweep evaluated 19 combinations of α and β.

The selected weights were:

- α = 0.99
- β = 0.01

The grid-status-based charging logic operates in three states:

| Grid Condition | Port Operation |
|---|---|
| NORMAL | Both ports at 1C |
| CONGESTION | Priority port at 1C; other port at 0.5C |
| OVERLOAD | Both ports OFF |

## Simulation Results

The MATLAB model evaluated charging allocation under five operating scenarios:

1. Normal grid supply
2. Grid congestion
3. Severe power shortage
4. Voltage sag
5. Supply recovery

### Key Findings

- Achieved a simulated 9.1% reduction in mean arrival-to-completion time compared with the initial priority weights.
- Under severe power shortage, one EV reached 100% SOC within 58 minutes using ChargeGrid, while neither EV completed charging with equal sharing during the 75-minute simulation.
- Under grid congestion, the priority-based strategy reduced EV1 completion time by 5 minutes.
- No modeled grid power-budget violations occurred in the five simulation scenarios.

These findings are based on the MATLAB model and the reported prototype monitoring tests.

## Repository Structure

- `Firmware/` – ESP32 firmware and communication implementation.
- `MATLAB/` – Energy management and charging allocation simulations.
- `Hardware/` – Circuit diagrams, components and prototype documentation.
- `Documentation/` – Project report and technical documentation.
- `Results/` – Simulation graphs and dashboard screenshots.

## Future Improvements

- Hardware validation of the complete charging allocation algorithm.
- Improved SOC estimation using current integration.
- Enhanced battery monitoring and protection.
- Integration of renewable energy sources.
- Advanced charging optimization under dynamic grid conditions.

## Project Documentation

Refer to the `Documentation/` directory for the detailed technical report.

## Contributors

- Athulya Sakthi M
- Siddhaarth Balaji
- Visham Kumar V

Electrical and Electronics Engineering  
Amrita Vishwa Vidyapeetham, Coimbatore, India
