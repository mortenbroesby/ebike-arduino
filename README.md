# E-Bike Arduino

WIP - This project is a work in progress.

## Code

> [Main entry-point](src/main.cpp).

# E-Bike Arduino Throttle Controller

This project is an Arduino-based controller for eBike throttles. It intercepts, processes, and adjusts throttle input signals to provide better control and integrate additional functionalities like safety measures, state management, and button-based adjustments.

## Features

-   **Throttle Adjustment**: Dynamically adjusts throttle output based on configured voltage levels and speed levels.
-   **Voltage Modes**: Supports 3.3V and 5V throttle systems.
-   **Speed Levels**: Configurable speed levels from idle (0%) to full power (100%).
-   **Button Controls**: Adjust speed levels with up and down buttons.
-   **Brake Handling**: Active brake detection and response.
-   **Finite State Machine (FSM)**: Manages system states like detection, braking, and error handling.
-   **Smoothing**: Smooths throttle input using a median filter and gradual transitions.
-   **PAS Monitoring**: Monitors pedal assist system (PAS) activity for timeout and pedaling direction.
-   **LED Feedback**: LED indicator for active throttle states.
-   **Task Scheduler**: Efficiently handles periodic tasks like PAS monitoring, LED updates, and button debouncing.
