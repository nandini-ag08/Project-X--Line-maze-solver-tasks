# LMS / Meshmerize — Fresher Selection Tasks

Submission for all nine selection tasks: an ESP32 line-following and maze-solving robot, covering
component selection, power design, wiring, diagnostics, algorithms and working firmware.

---

## Repository structure

```
.
├── README.md
├── BOM/
│   ├── BOM.csv
│   ├── Power block diagram.jpg
│   ├── Power Budget Table.csv
│   ├── Wiring Diagram Task-6.jpeg
│   ├── Task-7 Data flow.jpeg
│   └── Task-8 example maze diagram.jpeg
├── docs/
│   └── Notes.pdf
└── src/
    ├── task1_basic.py
    ├── Task 3 pseudocode.cpp
    ├── Task 8 pseudo code.cpp
    └── maze_dfs_line_follower_final code.ino
```

## Where to find each task

| Task | Title | Deliverable |
|------|-------|-------------|
| 1 | Basic Coding | [`src/task1_basic.py`](src/task1_basic.py) |
| 2 | Datasheet Literacy | [`docs/Notes.pdf`](docs/Notes.pdf) |
| 3 | PID Control | Explanation in [`docs/Notes.pdf`](docs/Notes.pdf); code sketch in [`src/Task 3 pseudocode.cpp`](src/Task%203%20pseudocode.cpp) |
| 4 | BOM Preparation | [`BOM/BOM.csv`](BOM/BOM.csv) and [`BOM/Power block diagram.jpg`](BOM/Power%20block%20diagram.jpg) |
| 5 | Power Budget & Battery Selection | [`BOM/Power Budget Table.csv`](BOM/Power%20Budget%20Table.csv); runtime calculation and chemistry comparison in [`docs/Notes.pdf`](docs/Notes.pdf) |
| 6 | Electronics & Hardware | [`BOM/Wiring Diagram Task-6.jpeg`](BOM/Wiring%20Diagram%20Task-6.jpeg); troubleshooting analysis in [`docs/Notes.pdf`](docs/Notes.pdf) |
| 7 | Wireless Sensor Diagnostics | [`BOM/Task-7 Data flow.jpeg`](BOM/Task-7%20Data%20flow.jpeg); protocol justification in [`docs/Notes.pdf`](docs/Notes.pdf) |
| 8 | Maze-Solving Algorithms | [`BOM/Task-8 example maze diagram.jpeg`](BOM/Task-8%20example%20maze%20diagram.jpeg); analysis in [`docs/Notes.pdf`](docs/Notes.pdf); pseudocode in [`src/Task 8 pseudo code.cpp`](src/Task%208%20pseudo%20code.cpp) |
| 9 | LMS Implementation Challenge | [`src/maze_dfs_line_follower_final code.ino`](src/maze_dfs_line_follower_final%20code.ino) |

## File guide

### `BOM/`
| File | Contents |
|------|----------|
| `BOM.csv` | Bill of Materials within the ₹10,000 budget: model, quantity, unit price, supplier link, specifications and rationale, including shipping and contingency |
| `Power block diagram.jpg` | Power-block diagram: battery, motor driver, regulator, ESP32, sensors and motors with the voltage on each rail |
| `Power Budget Table.csv` | Idle and peak (stall) current of every powered component, summed to a worst-case total |
| `Wiring Diagram Task-6.jpeg` | Labelled wiring diagram: controller, sensor array, L298N, motors, supply voltages and common ground |
| `Task-7 Data flow.jpeg` | Block diagram of the wireless diagnostics data flow, from sensor read on the ESP32 to visualisation on a phone or PC |
| `Task-8 example maze diagram.jpeg` | Annotated example maze showing junctions as nodes, paths as edges and the traversal |

### `docs/`
| File | Contents |
|------|----------|
| `Notes.pdf` | Written notes: datasheet analysis, PID control explanation, troubleshooting analysis, wireless protocol justification, maze-algorithm comparison and worked example, power runtime and chemistry comparison |

### `src/`
| File | Contents |
|------|----------|
| `task1_basic.py` | Task 1: weighted-centroid line position (all-zero case handled explicitly) and motor-command clamping that preserves sign, with example input and output |
| `Task 3 pseudocode.cpp` | Task 3: PID code sketch for line following |
| `Task 8 pseudo code.cpp` | Task 8: pseudocode for the chosen maze-solving method (DFS, priority left, forward, right) |
| `maze_dfs_line_follower_final code.ino` | Task 9: complete ESP32 firmware (see below) |

## How to run

### Task 1 (Python)
```
python src/task1_basic.py
```
Each function prints an example input and its output.

### Task 9 (ESP32 firmware)

**Requirements:** Arduino IDE 2.x and the **esp32 by Espressif Systems** board package
(works with core 2.x and 3.x; the PWM call adapts automatically). Board: **ESP32 Dev Module**.

1. Copy `maze_dfs_line_follower_final code.ino` into a new folder named `maze_dfs_line_follower`
   and rename the file to `maze_dfs_line_follower.ino`. The Arduino IDE requires a sketch to sit in a
   folder with the same name, and sketch names cannot contain spaces.
2. Open the sketch and check the top of the file has `#define SIMULATION 1`.
3. Select the board and COM port, then upload.
4. Open the Serial Monitor at **115200 baud**.

**Simulation mode (`SIMULATION 1`, no hardware needed)** prints two things:
1. **PID self-test:** ten hard-coded raw ADC arrays go through the centroid, PID and motor mixing,
   showing position, correction and left/right motor values. One sample is an all-white reading
   that exercises lost-line recovery.
2. **DFS maze simulation:** a built-in maze with a dead end on the left, a second dead end and a
   loop is solved. Each node, the exits found, the exit taken, every backtrack and the final path
   are printed.

**On the robot (`SIMULATION 0`):** place the robot on the start line facing into the maze. After a
2 s delay it spins in place for 3 s to calibrate the sensors, then follows the line and solves the
maze. Every control loop streams `S1,S2,S3,S4,S5,error,motorL,motorR` over serial.

**Settings at the top of the sketch**

| Setting | Purpose |
|---------|---------|
| `SIMULATION` | 1 = software validation, 0 = real robot |
| `TELEMETRY` | 1 = print values every loop |
| `CAL_AUTO_SPIN` | 0 = sweep the sensors over line and floor by hand during calibration |
| `INVERT_LEFT`, `INVERT_RIGHT` | flip if a motor spins the wrong way |
| `KP`, `KI`, `KD`, `BASE_SPEED` | PID gains and base speed |
| `ADVANCE_MS`, `TURN_MIN_90_MS`, `TURN_MIN_180_MS` | junction and turn timing, to be tuned on the real track |

## Design summary

- **Line position:** weighted centroid of min-max calibrated analog readings, weights `[-2, -1, 0, 1, 2]`,
  giving a position from -2000 to +2000.
- **Control:** PID on the centroid error, applied as differential speed
  (`left = base + correction`, `right = base − correction`). The error is positive when the line is on the right.
- **Lost line:** a short gap is ridden out by steering toward the last side the line was seen; a confirmed
  loss is treated as a dead end (U-turn and backtrack); if the U-turn cannot find the line again,
  the robot performs a safe stop.
- **Maze solving:** depth-first search with exit priority **left → forward → right**, a visited map to
  prevent loops, and a stack for backtracking.
- **Telemetry:** sensor values, error and motor commands are streamed every loop at 115200 baud.

## Hardware (Appendix A)

| Item | Detail |
|------|--------|
| Controller | ESP32-WROOM-32, Arduino framework |
| Sensors | 5 analog IR sensors on GPIO 34, 35, 32, 33, 25 (12-bit ADC, higher value = darker line) |
| Motor driver | L298N: ENA 14, IN1 27, IN2 26, ENB 13, IN3 12, IN4 11 |
| Power | 3S LiPo 11.1 V to the L298N; 5 V to ESP32 Vin; 3.3 V for sensors and logic |

## Assumptions

- The maze is a grid of line junctions, and each edge between two junctions counts as one step.
- The goal is a solid black area (all five sensors see the line after the robot creeps forward).
- The robot starts on the start cell facing into the maze.
- Pin assignments and power rails are taken exactly from the Appendix A mock specification.
- Sensors are calibrated at startup over the line and the floor.

## Limitations

- **Validated in simulation only.** Hard-coded sensor arrays and a software maze were used, as the task
  permits. Timing constants and PID gains are starting values and need tuning on the real robot.
- **DFS finds a path, not necessarily the shortest.** Optimal-route replay is discussed in the notes
  but is not part of the firmware.
- **Pin caveat in the mock spec:** on a real ESP32-WROOM-32, GPIO 11 is connected to the internal flash and
  GPIO 12 is a boot strapping pin. The mock pins were used as given; on a physical board, move IN3 and IN4
  to free pins.
- Position tracking assumes uniform grid spacing.
- With only five sensors the position resolution is limited, so very sharp or closely spaced turns may need
  a lower base speed.

## References

- Espressif, *ESP32-WROOM-32 Datasheet*
- STMicroelectronics, *L298 Dual Full-Bridge Driver Datasheet*
- Arduino-ESP32 documentation: <https://docs.espressif.com/projects/arduino-esp32/>
- Task sheet: *LMS / Meshmerize — Fresher Selection Tasks*, including Appendix A
