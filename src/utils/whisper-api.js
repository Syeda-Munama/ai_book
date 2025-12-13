/**
 * Utility functions for OpenAI Whisper API integration
 * Used in Voice-Language-Action (VLA) systems for speech recognition
 */

class WhisperAPI {
  constructor(apiKey) {
    this.apiKey = apiKey;
    this.baseUrl = 'https://api.openai.com/v1';
  }

  /**
   * Transcribe audio using Whisper API
   * @param {Blob|File} audioFile - Audio file to transcribe
   * @param {Object} options - Transcription options
   * @returns {Promise<string>} Transcribed text
   */
  async transcribe(audioFile, options = {}) {
    const formData = new FormData();
    formData.append('file', audioFile);
    formData.append('model', options.model || 'whisper-1');

    if (options.response_format) {
      formData.append('response_format', options.response_format);
    }

    if (options.language) {
      formData.append('language', options.language);
    }

    if (options.temperature !== undefined) {
      formData.append('temperature', options.temperature.toString());
    }

    try {
      const response = await fetch(`${this.baseUrl}/audio/transcriptions`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${this.apiKey}`,
        },
        body: formData,
      });

      if (!response.ok) {
        throw new Error(`API request failed: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      return data.text;
    } catch (error) {
      console.error('Error transcribing audio:', error);
      throw error;
    }
  }

  /**
   * Translate audio to English using Whisper API
   * @param {Blob|File} audioFile - Audio file to translate
   * @param {Object} options - Translation options
   * @returns {Promise<string>} Translated text
   */
  async translate(audioFile, options = {}) {
    const formData = new FormData();
    formData.append('file', audioFile);
    formData.append('model', options.model || 'whisper-1');

    if (options.response_format) {
      formData.append('response_format', options.response_format);
    }

    if (options.temperature !== undefined) {
      formData.append('temperature', options.temperature.toString());
    }

    try {
      const response = await fetch(`${this.baseUrl}/audio/translations`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${this.apiKey}`,
        },
        body: formData,
      });

      if (!response.ok) {
        throw new Error(`API request failed: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      return data.text;
    } catch (error) {
      console.error('Error translating audio:', error);
      throw error;
    }
  }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = WhisperAPI;
} else if (typeof window !== 'undefined') {
  window.WhisperAPI = WhisperAPI;
}