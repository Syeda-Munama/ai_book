#!/usr/bin/env python3
"""
Voice-to-Action Pipeline for Vision-Language-Action (VLA) Systems
This example demonstrates the complete pipeline from voice command to robotic action
"""

import rospy
import speech_recognition as sr
import openai
import json
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class VoiceToActionPipeline:
    def __init__(self):
        rospy.init_node('voice_to_action_pipeline')

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Calibrate for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Initialize OpenAI API
        openai.api_key = rospy.get_param('openai_api_key', '')

        # Initialize ROS components
        self.voice_command_sub = rospy.Subscriber('/voice_command', String, self.voice_callback)
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server()

        # Location mapping for navigation
        self.location_map = {
            'kitchen': (1.0, 2.0, 0.0, 1.0),  # (x, y, z, w)
            'living room': (-1.0, 1.0, 0.0, 1.0),
            'bedroom': (0.0, -2.0, 0.0, 1.0),
            'office': (2.0, 0.0, 0.0, 1.0),
        }

    def listen_for_voice_command(self):
        """Listen for a voice command from the microphone"""
        with self.microphone as source:
            print("Listening for voice command...")
            audio = self.recognizer.listen(source)

        try:
            # Use Whisper API for speech recognition
            text = self.recognizer.recognize_whisper_api(
                audio,
                api_key=openai.api_key,
                model="whisper-1"
            )
            print(f"Heard command: {text}")
            return text
        except sr.RequestError as e:
            print(f"API error: {e}")
            return None
        except sr.UnknownValueError:
            print("Could not understand audio")
            return None

    def process_command_with_llm(self, command):
        """Use LLM to process the command and extract action"""
        prompt = f"""
        You are a command processing system for a humanoid robot.
        Parse the following command and extract the intended action.

        Command: {command}

        Return a JSON object with:
        - action_type: The type of action (navigate, grasp, speak, etc.)
        - parameters: Parameters for the action (location, object, etc.)

        Example responses:
        For "Go to the kitchen": {{"action_type": "navigate", "parameters": {{"location": "kitchen"}}}}
        For "Move to the bedroom": {{"action_type": "navigate", "parameters": {{"location": "bedroom"}}}}
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            content = response.choices[0].message.content
            # Extract JSON from response
            start = content.find('{')
            end = content.rfind('}') + 1
            if start != -1 and end != 0:
                json_str = content[start:end]
                result = json.loads(json_str)
                return result
        except Exception as e:
            print(f"Error processing command with LLM: {e}")
            return None

    def execute_navigation(self, location):
        """Execute navigation to specified location"""
        if location not in self.location_map:
            print(f"Unknown location: {location}")
            return False

        x, y, z, w = self.location_map[location]

        # Create navigation goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w

        # Send goal to navigation stack
        print(f"Navigating to {location} at ({x}, {y})")
        self.nav_client.send_goal(goal)
        wait = self.nav_client.wait_for_result(rospy.Duration(60))

        if not wait:
            print("Navigation action server not available!")
            return False
        else:
            result = self.nav_client.get_result()
            if result:
                print(f"Successfully navigated to {location}")
                return True
            else:
                print(f"Failed to navigate to {location}")
                return False

    def voice_callback(self, msg):
        """Callback for voice command messages"""
        command = msg.data
        print(f"Processing voice command: {command}")

        # Process command with LLM
        action_plan = self.process_command_with_llm(command)
        if not action_plan:
            print("Could not process command with LLM")
            return

        action_type = action_plan.get('action_type')
        parameters = action_plan.get('parameters', {})

        if action_type == 'navigate':
            location = parameters.get('location')
            if location:
                success = self.execute_navigation(location)
                if success:
                    print(f"Successfully executed navigation to {location}")
                else:
                    print(f"Failed to execute navigation to {location}")
            else:
                print("No location specified in navigation command")
        else:
            print(f"Action type '{action_type}' not implemented in this example")

    def run(self):
        """Main run loop - continuously listen for voice commands"""
        print("Voice-to-Action pipeline running...")
        print("Press Ctrl+C to stop")

        rate = rospy.Rate(1)  # Check for new commands at 1 Hz

        while not rospy.is_shutdown():
            # Listen for a voice command
            command = self.listen_for_voice_command()

            if command:
                # Process the command
                action_plan = self.process_command_with_llm(command)
                if action_plan:
                    action_type = action_plan.get('action_type')
                    parameters = action_plan.get('parameters', {})

                    if action_type == 'navigate':
                        location = parameters.get('location')
                        if location:
                            self.execute_navigation(location)

            rate.sleep()

if __name__ == '__main__':
    try:
        pipeline = VoiceToActionPipeline()
        pipeline.run()
    except rospy.ROSInterruptException:
        print("Voice-to-Action pipeline interrupted")
    except KeyboardInterrupt:
        print("Voice-to-Action pipeline stopped by user")