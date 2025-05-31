# Arduino Drowsiness-Detection-system with MediaPipe


A real-time eye blink detection system integrating MediaPipe-based computer vision with Arduino hardware to monitor prolonged eye closure for applications like driver drowsiness detection. A Python script processes webcam input to detect eye status and sends serial commands to an Arduino Uno, which controls a motor (via a motor control module) and triggers a buzzer when eyes are closed for over 2 seconds.

## Table of Contents
- [Project Overview](#project-overview)
- [Features](#features)
- [Hardware Components](#hardware-components)
- [Software Requirements](#software-requirements)
  - [Python Libraries](#python-libraries)
  - [Arduino IDE](#arduino-ide)
- [Setup Instructions](#setup-instructions)
  - [Creating a Python Virtual Environment in VS Code](#creating-a-python-virtual-environment-in-vs-code)
  - [Installing Python Libraries](#installing-python-libraries)
  - [Setting Up Arduino IDE](#setting-up-arduino-ide)
  - [Hardware Connections](#hardware-connections)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Testing and Results](#testing-and-results)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Project Overview
This project combines advanced computer vision with embedded systems to detect prolonged eye closure, enhancing safety in automotive settings. A Python script running on a computer uses MediaPipe to calculate the Eye Aspect Ratio (EAR) from webcam input, determining eye status (open, closed, or no face detected). The status is sent via serial communication to an Arduino Uno, which stops a motor and activates a buzzer if eyes are closed for more than 2 seconds, indicating potential drowsiness. The system achieves ~95% accuracy in controlled environments and showcases skills in computer vision, serial communication, and hardware-software integration.

Developed as prototype which is ideal for IoT, wearable devices, and real-time monitoring applications.

## Features
- **Real-Time Detection**: Processes webcam frames to detect eye status using MediaPipe.
- **Serial Communication**: Sends eye status (‘N’ for normal/open, ‘A’ for alert/closed) from Python to Arduino.
- **Alert System**: Stops motor and activates buzzer for prolonged eye closure (>2 seconds).
- **High Accuracy**: ~95% detection rate for prolonged closures in controlled tests.
- **Adjustable Parameters**: Configurable EAR threshold and closure duration.
- **Debugging Support**: Serial Monitor output for both Python and Arduino.

## Hardware Components
- **Arduino Uno**: Microcontroller board for processing serial commands and controlling outputs.
- **Motor Control Module**: L298N or similar module to control a DC motor (enable, direction pins).
- **DC Motor**: 5V–12V motor for simulating vehicle motion or other mechanical output.
- **Buzzer**: Active or passive 5V buzzer for audible alerts.
- **Jumper Wires**: For connecting components.
- **Webcam**: USB or built-in webcam for computer vision input.
- **Power Supply**: USB cable for Arduino; external 12V supply for motor module (if required).
- **Breadboard**: For prototyping connections.

## Software Requirements

### Python Libraries
The Python script uses the following libraries:

| Library         | Purpose                                      | Installation Command                     |
|-----------------|----------------------------------------------|-----------------------------------------|
| `opencv-python` | Captures webcam input and displays frames    | `pip install opencv-python`             |
| `mediapipe`     | Detects facial landmarks for EAR calculation | `pip install mediapipe`                 |
| `numpy`         | Performs numerical computations for EAR      | `pip install numpy`                     |
| `pyserial`      | Enables serial communication with Arduino    | `pip install pyserial`                  |

### Arduino IDE
- **Version**: 2.3.2 or later (download from [arduino.cc](https://www.arduino.cc/en/software)).
- **Purpose**: Used to write, compile, and upload the Arduino sketch to the board.
- **No External Libraries**: Uses standard Arduino libraries (`Serial`, `digitalRead`, `digitalWrite`).

## Setup Instructions

### Creating a Python Virtual Environment in VS Code
To manage Python dependencies, create a virtual environment in VS Code:

1. **Install Python Extension**:
   - Open VS Code and go to Extensions (`Ctrl+Shift+X`).
   - Install the **Python** extension by Microsoft.

2. **Select Python Interpreter**:
   - Open the Command Palette (`Ctrl+Shift+P`).
   - Select **Python: Select Interpreter** and choose a Python version (e.g., `Python 3.9` from Anaconda or system Python).

3. **Create Virtual Environment**:
   - Open a terminal in VS Code (`Ctrl+`` or Terminal > New Terminal`).
   - Create a virtual environment:
     ```bash
     python -m venv .venv
     ```
   - Activate the virtual environment:
     - **Windows**:
       ```bash
       .venv\Scripts\activate
       ```
     - **macOS/Linux**:
       ```bash
       source .venv/bin/activate
       ```
   - Verify activation (terminal prompt should show `(.venv)`).

4. **Select Virtual Environment in VS Code**:
   - Reopen the Command Palette and select **Python: Select Interpreter**.
   - Choose the interpreter from `.venv` (e.g., `./.venv/Scripts/python.exe` on Windows).

### Installing Python Libraries
With the virtual environment activated, install the required libraries:
```bash
pip install opencv-python mediapipe numpy pyserial
```
Verify installation:
```bash
python -c "import cv2, mediapipe, numpy, serial; print(cv2.__version__, mediapipe.__version__, numpy.__version__, serial.__version__)"
```
Expected output: `4.10.0 0.10.14 1.26.4 3.5.2` (versions may vary).

### Setting Up Arduino IDE
1. **Download and Install**:
   - Download the Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software).
   - Install and launch the IDE.

2. **Configure Board and Port**:
   - Connect your Arduino Uno via USB.
   - In the IDE, go to **Tools > Board** and select **Arduino Uno**.
   - Go to **Tools > Port** and select the port (e.g., `COM3` on Windows).

3. **Test Connection**:
   - Open a blank sketch and click **Upload** to ensure the board is recognized.

### Hardware Connections
Connect the components to the Arduino Uno:

| Component               | Arduino Pin | Notes                     |
|-------------------------|-------------|---------------------------|
| Motor Module ENA        | Digital 5   | Motor speed control       |
| Motor Module IN1        | Digital 6   | Motor direction control   |
| Motor Module IN2        | Digital 7   | Motor direction control   |
| Buzzer Positive         | Digital 8   | PWM-capable pin           |
| Buzzer Negative         | GND         | Ground                    |
| Motor Module VCC        | 12V/VIN*    | External power (if needed)|
| Motor Module GND        | GND         | Ground                    |

*Note*: If the motor module requires >5V, connect an external 12V power supply to the module’s VCC and share a common GND with the Arduino. Ensure the webcam is connected to the computer running the Python script.

## Usage
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/your-username/arduino-eye-blink-detection.git
   cd arduino-eye-blink-detection
   ```

2. **Upload Arduino Code**:
   - Open `src/eye_detection_serial.ino` in the Arduino IDE.
   - Ensure the correct board and port are selected.
   - Click **Upload** to flash the code.
   - Open the Serial Monitor (9600 baud) to verify Arduino is ready.

3. **Run Python Script**:
   - Activate the virtual environment:
     ```bash
     .venv\Scripts\activate  # Windows
     source .venv/bin/activate  # macOS/Linux
     ```
   - Update the serial port in `src/eye_detection_serial.py` (e.g., `serial.Serial('COM3', 9600)`).
   - Run the script:
     ```bash
     python src/eye_detection_serial.py
     ```
   - A webcam window opens, showing eye status and facial landmarks.
   - Press `q` to quit.

4. **Monitor Output**:
   - **Python**: Displays “Eye: Open (EAR: X.XX)” or “Eye: Closed” and sends ‘N’ or ‘A’ to Arduino.
   - **Arduino**: Stops motor and activates buzzer on receiving ‘A’ (closed eyes); runs motor on ‘N’ (open eyes).
   - Serial Monitor shows “Received: Closed” or “Received: Open”.

5. **Fine-Tuning**:
   - Adjust `EAR_THRESHOLD` in the Python script (default: 0.25) for sensitivity.
   - Modify `closeThreshold` in the Arduino code (default: 2000ms) for alert duration.

**Example Serial Output (Arduino)**:
```
Arduino Ready
Received: Open
Received: Closed
Eyes Closed - Alert!
```

## Project Structure
```
arduino-eye-blink-detection/
├── src/
│   ├── eye_detection_serial.ino  # Arduino sketch
│   └── eye_detection_serial.py   # Python script
├── assets/
│   └── banner.jpg               # Project banner image
├── docs/
│   └── circuit_diagram.png      # Circuit diagram (Fritzing)
│   └── flowchart.png            # Workflow flowchart
├── README.md                    # Project documentation
└── LICENSE                      # License file
```

## Testing and Results
- **Lab Testing**: Conducted with 10 subjects in controlled lighting, achieving 95% accuracy for prolonged closures.
- **Real-World Testing**: Simulated vehicle environment, with 90% reliability under moderate lighting.
- **Challenges**:
  - Webcam lighting sensitivity mitigated by adjusting `min_detection_confidence`.
  - Serial port conflicts resolved by closing Serial Monitor before running Python.
- **Limitations**: Requires stable lighting for webcam; motor power may need external supply.

**Sample Test Data**:
| Test Case         | Duration (s) | Detected | Accuracy (%) |
|-------------------|--------------|----------|--------------|
| Normal Blink      | 0.2–0.5      | No Alert | 98           |
| Prolonged Closure | 2–3          | Alert    | 95           |
| Low Light         | 2–3          | Alert    | 85           |

## Contributing
Contributions are welcome! To contribute:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/your-feature`).
3. Commit changes (`git commit -m "Add your feature"`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a Pull Request.

Please follow the [Contributor Covenant](https://www.contributor-covenant.org/) code of conduct.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements
- **MediaPipe Team**: For robust facial landmark detection.
- **Arduino Community**: For extensive documentation.
- **Thesis Advisor**: For project guidance.
- **Open-Source Tools**: Fritzing, VS Code, Python.

---

