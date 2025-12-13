/**
 * LLM processing utilities for cognitive planning in VLA systems
 * Handles task decomposition, planning, and natural language understanding
 */

class LLMProcessor {
  constructor(apiKey, options = {}) {
    this.apiKey = apiKey;
    this.baseUrl = options.baseUrl || 'https://api.openai.com/v1';
    this.defaultModel = options.model || 'gpt-4';
    this.temperature = options.temperature || 0.3;
  }

  /**
   * Decompose a high-level task into executable steps
   * @param {string} task - High-level task description
   * @returns {Promise<Array>} Array of executable steps
   */
  async decomposeTask(task) {
    const prompt = `
      You are a cognitive planning system for a humanoid robot.
      Decompose the following high-level task into a sequence of executable robotic actions.

      Task: ${task}

      Return a JSON list of steps, each with an action type and parameters.
      Action types should include: navigate, detect_object, grasp, place, speak, wait
      Each step should have appropriate parameters for the action.

      Example format:
      [
        {"action": "navigate", "params": {"location": "kitchen"}},
        {"action": "detect_object", "params": {"object_type": "mug", "color": "red"}},
        {"action": "grasp", "params": {"object_id": "red_mug_123"}},
        {"action": "navigate", "params": {"location": "table"}},
        {"action": "place", "params": {"location": "table"}}
      ]

      Be specific about locations, objects, and parameters.
    `;

    try {
      const response = await fetch(`${this.baseUrl}/chat/completions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.apiKey}`,
        },
        body: JSON.stringify({
          model: this.defaultModel,
          messages: [{ role: 'user', content: prompt }],
          temperature: this.temperature,
          response_format: { type: 'json_object' }
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      const content = data.choices[0].message.content;
      const parsed = JSON.parse(content);
      return parsed;
    } catch (error) {
      console.error('Error decomposing task:', error);
      throw error;
    }
  }

  /**
   * Process natural language command and extract intent
   * @param {string} command - Natural language command
   * @returns {Promise<Object>} Parsed command with intent and parameters
   */
  async processCommand(command) {
    const prompt = `
      You are a natural language understanding system for a humanoid robot.
      Parse the following command and extract the intent and parameters.

      Command: ${command}

      Return a JSON object with:
      - intent: The main action (navigate, grasp, speak, etc.)
      - parameters: Relevant parameters for the action
      - entities: Identified objects, locations, people, etc.

      Example:
      {
        "intent": "grasp_object",
        "parameters": {
          "object_color": "red",
          "object_type": "cup",
          "location": "kitchen"
        },
        "entities": ["red cup", "kitchen"]
      }
    `;

    try {
      const response = await fetch(`${this.baseUrl}/chat/completions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.apiKey}`,
        },
        body: JSON.stringify({
          model: this.defaultModel,
          messages: [{ role: 'user', content: prompt }],
          temperature: this.temperature,
          response_format: { type: 'json_object' }
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      const content = data.choices[0].message.content;
      const parsed = JSON.parse(content);
      return parsed;
    } catch (error) {
      console.error('Error processing command:', error);
      throw error;
    }
  }

  /**
   * Generate cognitive plan with context awareness
   * @param {string} goal - Goal to achieve
   * @param {Object} context - Current context (robot state, environment, etc.)
   * @returns {Promise<Object>} Cognitive plan
   */
  async generateCognitivePlan(goal, context = {}) {
    const contextStr = JSON.stringify(context, null, 2);

    const prompt = `
      You are a cognitive planning system for a humanoid robot with awareness of context.
      Create a cognitive plan to achieve the goal considering the current context.

      Goal: ${goal}

      Current Context:
      ${contextStr}

      Return a JSON object with:
      - plan: Array of steps to achieve the goal
      - considerations: Important factors to consider
      - potential_obstacles: Potential obstacles based on context
      - alternatives: Alternative approaches if primary plan fails

      The plan should be detailed enough for a robotic system to execute.
    `;

    try {
      const response = await fetch(`${this.baseUrl}/chat/completions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.apiKey}`,
        },
        body: JSON.stringify({
          model: this.defaultModel,
          messages: [{ role: 'user', content: prompt }],
          temperature: this.temperature,
          response_format: { type: 'json_object' }
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      const content = data.choices[0].message.content;
      const parsed = JSON.parse(content);
      return parsed;
    } catch (error) {
      console.error('Error generating cognitive plan:', error);
      throw error;
    }
  }

  /**
   * Validate a plan for feasibility
   * @param {Array} plan - Plan to validate
   * @param {Object} constraints - Constraints to consider
   * @returns {Promise<Object>} Validation results
   */
  async validatePlan(plan, constraints = {}) {
    const planStr = JSON.stringify(plan, null, 2);
    const constraintsStr = JSON.stringify(constraints, null, 2);

    const prompt = `
      You are a plan validation system for a humanoid robot.
      Analyze the following plan for feasibility and safety.

      Plan:
      ${planStr}

      Constraints:
      ${constraintsStr}

      Return a JSON object with:
      - is_valid: Whether the plan is feasible
      - issues: Any identified issues with the plan
      - suggestions: Suggestions for improving the plan
      - risk_level: Overall risk level (low, medium, high)
    `;

    try {
      const response = await fetch(`${this.baseUrl}/chat/completions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.apiKey}`,
        },
        body: JSON.stringify({
          model: this.defaultModel,
          messages: [{ role: 'user', content: prompt }],
          temperature: this.temperature,
          response_format: { type: 'json_object' }
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      const content = data.choices[0].message.content;
      const parsed = JSON.parse(content);
      return parsed;
    } catch (error) {
      console.error('Error validating plan:', error);
      throw error;
    }
  }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = LLMProcessor;
} else if (typeof window !== 'undefined') {
  window.LLMProcessor = LLMProcessor;
}