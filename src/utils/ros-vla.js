/**
 * ROS 2 utilities for Vision-Language-Action (VLA) systems
 * Provides interfaces for navigation, manipulation, and perception
 */

class ROSVLAInterface {
  constructor(ros) {
    this.ros = ros; // ROSLIB.Ros instance
    this.actionClients = new Map();
    this.services = new Map();
  }

  /**
   * Initialize navigation action client
   * @param {string} actionName - Name of the navigation action
   * @returns {Promise} Promise that resolves when client is initialized
   */
  initNavigation(actionName = '/navigate_to_pose') {
    return new Promise((resolve, reject) => {
      const navigationActionClient = new ROSLIB.ActionClient({
        ros: this.ros,
        serverName: actionName,
        actionName: 'nav2_msgs/action/NavigateToPose',
      });

      // Store the client
      this.actionClients.set('navigation', navigationActionClient);

      // Test connection by checking if server is available
      navigationActionClient.goals = []; // Initialize goals array
      resolve(navigationActionClient);
    });
  }

  /**
   * Send navigation goal
   * @param {Object} goal - Navigation goal with position and orientation
   * @returns {Promise} Promise that resolves when navigation is complete
   */
  navigateToPose(goal) {
    const navigationClient = this.actionClients.get('navigation');
    if (!navigationClient) {
      throw new Error('Navigation client not initialized');
    }

    return new Promise((resolve, reject) => {
      const navigationGoal = new ROSLIB.Goal({
        actionClient: navigationClient,
        goalMessage: {
          pose: {
            header: {
              frame_id: goal.frame_id || 'map'
            },
            pose: {
              position: {
                x: goal.x || 0,
                y: goal.y || 0,
                z: goal.z || 0
              },
              orientation: {
                x: goal.orientation_x || 0,
                y: goal.orientation_y || 0,
                z: goal.orientation_z || 0,
                w: goal.orientation_w || 1
              }
            }
          }
        }
      });

      navigationGoal.on('feedback', (feedback) => {
        console.log('Navigation feedback:', feedback);
      });

      navigationGoal.on('result', (result) => {
        console.log('Navigation result:', result);
        resolve(result);
      });

      navigationGoal.send();
    });
  }

  /**
   * Initialize manipulation action client
   * @param {string} actionName - Name of the manipulation action
   * @returns {Promise} Promise that resolves when client is initialized
   */
  initManipulation(actionName = '/manipulation_controller') {
    return new Promise((resolve, reject) => {
      const manipulationActionClient = new ROSLIB.ActionClient({
        ros: this.ros,
        serverName: actionName,
        actionName: 'control_msgs/action/FollowJointTrajectory',
      });

      // Store the client
      this.actionClients.set('manipulation', manipulationActionClient);
      resolve(manipulationActionClient);
    });
  }

  /**
   * Send manipulation command
   * @param {Object} command - Manipulation command
   * @returns {Promise} Promise that resolves when manipulation is complete
   */
  manipulate(command) {
    const manipulationClient = this.actionClients.get('manipulation');
    if (!manipulationClient) {
      throw new Error('Manipulation client not initialized');
    }

    return new Promise((resolve, reject) => {
      const manipulationGoal = new ROSLIB.Goal({
        actionClient: manipulationClient,
        goalMessage: {
          trajectory: command.trajectory || {},
          path_tolerance: command.path_tolerance || [],
          goal_tolerance: command.goal_tolerance || [],
          goal_time_tolerance: command.goal_time_tolerance || { secs: 10, nsecs: 0 }
        }
      });

      manipulationGoal.on('feedback', (feedback) => {
        console.log('Manipulation feedback:', feedback);
      });

      manipulationGoal.on('result', (result) => {
        console.log('Manipulation result:', result);
        resolve(result);
      });

      manipulationGoal.send();
    });
  }

  /**
   * Call perception service
   * @param {string} serviceName - Name of the perception service
   * @param {Object} request - Service request parameters
   * @returns {Promise} Promise that resolves with service response
   */
  callPerceptionService(serviceName, request) {
    return new Promise((resolve, reject) => {
      const service = new ROSLIB.Service({
        ros: this.ros,
        name: serviceName,
        serviceType: 'object_recognition_msgs/srv/RecognizeObjects' // Generic type, adjust as needed
      });

      const serviceRequest = new ROSLIB.ServiceRequest(request);

      service.callService(serviceRequest, (result) => {
        resolve(result);
      });
    });
  }

  /**
   * Subscribe to camera feed
   * @param {string} topicName - Name of the camera topic
   * @param {function} callback - Callback function for image data
   */
  subscribeToCamera(topicName = '/camera/color/image_raw', callback) {
    const imageTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: 'sensor_msgs/msg/Image'
    });

    imageTopic.subscribe((message) => {
      callback(message);
    });

    return imageTopic;
  }

  /**
   * Publish command to robot
   * @param {string} topicName - Name of the command topic
   * @param {Object} message - Command message to publish
   */
  publishCommand(topicName, message) {
    const commandTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: 'std_msgs/msg/String' // Adjust as needed
    });

    commandTopic.publish(message);
  }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = ROSVLAInterface;
} else if (typeof window !== 'undefined') {
  window.ROSVLAInterface = ROSVLAInterface;
}