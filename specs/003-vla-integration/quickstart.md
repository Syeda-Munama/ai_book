# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites

Before starting with the Physical AI & Humanoid Robotics textbook, ensure you have the following prerequisites installed:

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or equivalent Linux distribution
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 50GB free space
- **GPU**: NVIDIA GPU with RTX 4070+ or equivalent for Isaac Sim (optional but recommended)
- **Docker**: Version 20.10 or higher

### Software Dependencies
- Node.js 18+ and npm 8+
- Python 3.10+
- ROS 2 Humble Hawksbill
- Git

## Setup Environment

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-book.git
cd physical-ai-book
```

### 2. Install Docusaurus Dependencies
```bash
npm install
```

### 3. Set up ROS 2 Environment
```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update

# Source ROS environment
source /opt/ros/humble/setup.bash
```

### 4. Install Simulation Environments

#### For Isaac Sim (recommended for advanced modules):
1. Download Isaac Sim from NVIDIA Developer website
2. Follow installation instructions for your system
3. Ensure Isaac Sim can connect to ROS 2 via Isaac ROS bridge

#### For Gazebo (alternative):
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

### 5. Configure API Keys (for VLA module)
```bash
# Create .env file in the root directory
cp .env.example .env
# Edit .env and add your OpenAI API key
nano .env
```

## Running the Textbook Locally

### 1. Start the Development Server
```bash
npm start
```

This will start the Docusaurus development server at `http://localhost:3000` with live reloading.

### 2. Running Code Examples

Each module contains code examples that can be run in the appropriate environment:

#### ROS 2 Examples:
```bash
# In a new terminal, source ROS environment
source /opt/ros/humble/setup.bash
source install/setup.bash  # If you have built packages

# Run ROS 2 nodes
ros2 run package_name executable_name
```

#### Isaac Sim Examples:
1. Launch Isaac Sim
2. Load the appropriate scene
3. Run the provided Python scripts via the Code Editor or in terminal

### 3. Testing Code Snippets in Docker

To test code snippets in an isolated environment:

```bash
# Build the testing container
docker build -t physical-ai-book-test -f Dockerfile .

# Run the container
docker run -it --rm -v $(pwd):/book physical-ai-book-test
```

## Module-Specific Setup

### Module 1: The Robotic Nervous System (ROS 2)
```bash
# Install additional ROS 2 packages
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs
```

### Module 2: The Digital Twin (Gazebo & Unity)
```bash
# Install Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros-gz
```

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- Ensure Isaac Sim is properly installed
- Install Isaac ROS packages:
```bash
sudo apt install ros-humble-isaac-ros-*  # All Isaac ROS packages
```

### Module 4: Vision-Language-Action (VLA)
```bash
# Install Python dependencies for VLA
pip3 install openai speechrecognition pyaudio
```

## Building for Production

To build the textbook for deployment:

```bash
npm run build
```

The static files will be generated in the `build/` directory and can be served by any static hosting service.

## Troubleshooting

### Common Issues:

1. **ROS 2 Commands Not Found**:
   - Make sure you've sourced the ROS environment: `source /opt/ros/humble/setup.bash`

2. **Docusaurus Fails to Start**:
   - Clear npm cache: `npm start -- --clear-cache`
   - Check Node.js version: `node --version` (should be 18+)

3. **Isaac Sim Connection Issues**:
   - Verify Isaac Sim is running
   - Check that Isaac ROS bridge is properly installed
   - Ensure both systems are on the same network if running remotely

4. **Docker Build Failures**:
   - Ensure sufficient disk space
   - Check Docker daemon is running: `sudo systemctl status docker`

## Next Steps

After completing the setup:

1. Navigate to the first module: [Module 1: The Robotic Nervous System](../docs/modules/module-01-ros2-nervous-system/README.md)
2. Follow the learning objectives and practical exercises
3. Complete the quiz at the end of each module
4. Work through the hands-on exercises
5. Build toward the capstone project in Module 8