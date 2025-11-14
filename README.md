# LeRobot SO-101 – Simulation and Hardware Control
Developer: Din Sopheak Panha

--------------------------------------------------------------------------------
📝 Credits & Acknowledgements
Huge thanks to LeRobot, LycheeAI, and FTServoPython for the libraries and packages that made this project possible.

ALL NOTES, DEVELOPED CODES AND SETUP ARE WRITTEN BY DIN SOPHEAK PANHA ONLY

Sources:
- LeRobot : https://huggingface.co/docs/lerobot/en/so101
- LycheeAI : https://github.com/MuammerBay/SO-ARM101_MoveIt_IsaacSim
- FTServoPython : https://github.com/ftservo/FTServo_Python
- Moveit2 : https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html

--------------------------------------------------------------------------------
📦 Requirements
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10.12
- OpenCV 4.10.0

--------------------------------------------------------------------------------
🔧 Installation

1) Install Feetech Servo SDK:

    ```pip install feetech-servo-sdk```
   
    More info: https://pypi.org/project/feetech-servo-sdk/

2) Install system packages:
   
   ```
   
    sudo apt update
    sudo apt install build-essential cmake python3-dev
    sudo apt install libopenblas-dev liblapack-dev
    sudo apt install libx11-dev libgtk-3-dev

   ```

3) Install Dlib:
   
    ```pip install dlib```
   
    If slow:
   
   ``` pip install dlib --no-cache-dir```

4) Install OpenCV:

   ```
   
    pip install opencv-python
    pip install opencv-contrib-python
    Or upgrade:
    pip install --upgrade opencv-python opencv-contrib-python

   ```

--------------------------------------------------------------------------------
Install LeRobot:

```
    git clone https://github.com/huggingface/lerobot.git
    cd lerobot
    pip install 'lerobot[all]'

```

Install ROS2 Humble:
    Follow: https://docs.ros.org/en/humble/Installation.html

Install MoveIt2:
    Follow: https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html

--------------------------------------------------------------------------------
🎮 Manual Teleoperation
Follow: https://huggingface.co/docs/lerobot/en/il_robots

--------------------------------------------------------------------------------
🎥 Teleoperation With Camera

Before running teleoperation with camera, check:

1) Serial Port:
    PORT = '/dev/ttyACM0'

2) Predictor Path:
    predictor_path = "/home/panha/LeRobot-SO101-simulation-and-hardware/src/data/shape_predictor_68_face_landmarks.dat"

3) Camera Index:
    cap = cv2.VideoCapture(2)
    To check your camera index:

   ```
        cd ~/LeRobot-SO101-simulation-and-hardware/src
        python3 check_cam_index.py

   ```

4) Calibration:
    Make sure to use the MIDDLE POSE as homing, NOT rest pose.

5) Run teleoperation with camera:

```
    cd ~/LeRobot-SO101-simulation-and-hardware/src
    python3 teleoperation_with_camera.py

```

--------------------------------------------------------------------------------
🛠 Adjust Motor Initial Pose & Limits

Example values:

DEFAULT_SPEED = 1500
DEFAULT_ACCEL = 255

init_pose = [1989, 2144, 1048, 1021, 1129, 2040]

MIN_MOTOR_ID1 = 750
MAX_MOTOR_ID1 = 3278

MIN_MOTOR_ID4 = 1639
MAX_MOTOR_ID4 = 795

MIN_MOTOR_ID6 = 2040
MAX_MOTOR_ID6 = 3432

--------------------------------------------------------------------------------
🔍 Read Encoder Values

To read pulses from each motor:

```
    cd ~/LeRobot-SO101-simulation-and-hardware/src
    python3 read_encoder_values.py

```

Make sure to adjust inside script:
    portHandler = PortHandler('/dev/ttyACM0')
    packetHandler.ReadPosSpeed(1)   <-- motor ID
    print("[ID:001] PresPos:(value) PresSpd:(value)")

--------------------------------------------------------------------------------
🤖 Control Using ROS2 MoveIt2

1) Run Python MoveIt2 controller:
  
   ```
    cd ~/LeRobot-SO101-simulation-and-hardware/src
    python3 moveit2_arm_control.py

   ```

(wait 1 second)

2) Launch MoveIt2 demo:
  
   ```
    cd ~/LeRobot-SO101-simulation-and-hardware
    colcon build
    source install/setup.bash
    ros2 launch so_arm_moveit_config demo.launch.py

   ```

In RViz:
- Click "Joints"
- Adjust values
- Click "Plan"
- Robot moves to the pose

--------------------------------------------------------------------------------
📬 Contact
Developer: Din Sopheak Panha
All development notes, setup procedures, and code are created solely by the author.

