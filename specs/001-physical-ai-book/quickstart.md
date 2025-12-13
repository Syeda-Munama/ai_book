# Quickstart Guide: Physical AI and Robotics Book Development

## Prerequisites

Before you begin developing the Physical AI and Robotics book content, ensure you have the following installed:

- **Node.js** (v16 or higher)
- **npm** or **yarn**
- **Python 3.10+** (for ROS 2 examples)
- **ROS 2 Humble** installed on Ubuntu 22.04
- **Git**

## Initial Setup

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd ai_hackathon
   ```

2. **Navigate to the physical-ai-book directory**:
   ```bash
   cd physical-ai-book
   ```

3. **Install Docusaurus dependencies**:
   ```bash
   npm install
   ```

4. **Start the development server**:
   ```bash
   npm start
   ```

This will start a local development server at `http://localhost:3000` where you can preview the documentation.

## Adding Content

1. **Create new chapters** in the `docs/` directory following the structure:
   ```
   docs/
   ├── chapter-1-ros2-architecture/
   │   ├── index.md
   │   └── concepts.md
   ├── chapter-2-nodes-topics-services/
   │   ├── index.md
   │   └── examples.md
   ```

2. **Update the sidebar** in `sidebars.ts` to include your new chapters.

3. **Add code snippets** using Docusaurus code blocks with appropriate language tags:
   ```python
   # ROS 2 Python example
   import rclpy
   from rclpy.node import Node

   class MinimalPublisher(Node):
       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher = self.create_publisher(String, 'topic', 10)
   ```

## Building for Production

To build the static site for deployment:

```bash
npm run build
```

## Testing ROS 2 Examples

To test the ROS 2 code examples:

1. Ensure ROS 2 Humble is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Test code snippets in a ROS 2 workspace following the examples provided.

## Running Documentation Tests

To check for broken links and other issues:

```bash
npm run serve
```