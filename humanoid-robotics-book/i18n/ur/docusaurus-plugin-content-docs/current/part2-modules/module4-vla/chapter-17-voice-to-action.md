# Chapter 17: Voice-to-Action (Whisper → LLM → Plan)

## Learning Objectives

By the end of this chapter, you will:

1. Understand Whisper, OpenAI's automatic speech recognition (ASR) model
2. Learn how to integrate Whisper with ROS 2 for real-time speech recognition
3. Explore the voice-to-action pipeline architecture
4. Understand how LLMs convert natural language to robot action plans
5. Learn about multimodal command processing (voice + vision + context)
6. Explore error handling and ambiguity resolution in voice commands
7. Understand real-world deployment considerations for voice-controlled robots

## Prerequisites

Before starting this chapter, you should understand:

- **VLA fundamentals**: Vision-language-action architecture (Chapter 16)
- **ROS 2 topics and services**: Pub/sub patterns and service calls (Chapter 5)
- **Python programming**: For integrating Whisper and LLMs
- **Basic audio processing concepts**: Sampling rate, spectrograms, audio formats
- **Large language models**: Prompting, few-shot learning, API usage (helpful but not required)

Recommended setup:
- ROS 2 Humble on Ubuntu 22.04
- Python 3.10+ with torch, transformers, openai libraries
- Microphone or audio input device
- OpenAI API key (for GPT models) or local LLM (LLaMA, Mistral)

## Introduction: Why This Matters

Imagine controlling a humanoid robot with your voice - no programming, no remote control, just natural speech:

*"Hey robot, can you bring me a glass of water?"*

The robot must:
1. **Hear** the command (capture audio, filter noise)
2. **Transcribe** speech to text (Whisper ASR)
3. **Understand** the instruction (LLM parses intent, identifies objects, constraints)
4. **Plan** the action sequence (LLM generates step-by-step plan: navigate → grasp → pour → deliver)
5. **Execute** the plan (VLA model executes each step)

This chapter focuses on steps 1-4: the **voice-to-action pipeline** that converts spoken commands into executable robot plans. We'll use **Whisper** for speech recognition and **LLMs** for intent understanding and planning, then bridge these to the VLA execution layer you learned in Chapter 16.

### Why Voice Control for Humanoid Robots?

**1. Natural human-robot interaction**: Humans prefer speaking over typing or using controllers
**2. Hands-free operation**: Useful when human operator is occupied (e.g., surgeon in operating room)
**3. Accessibility**: Enables robot control for users with limited mobility
**4. Context-aware commands**: Voice can convey urgency, emotion, and nuance (e.g., "Hurry!")
**5. Multi-user scenarios**: Multiple people can give commands without passing a physical controller

```mermaid
graph LR
    A[Human Speech<br/>"Bring me water"] --> B[Microphone Array<br/>Audio Capture]
    B --> C[Whisper ASR<br/>Speech-to-Text]
    C --> D[Text: "Bring me water"]
    D --> E[LLM Planner<br/>GPT-4 / Claude]
    E --> F[Action Plan<br/>Steps 1-5]
    F --> G[VLA Executor<br/>Robot Actions]
    G --> H[Humanoid Robot]

    style C fill:#e1f5ff
    style E fill:#ffe1f5
    style G fill:#e1ffe1
```

## Whisper: Automatic Speech Recognition

**Whisper** is OpenAI's open-source speech recognition model, released in September 2022. It's trained on 680,000 hours of multilingual and multitask supervised data from the web, making it robust to accents, background noise, and technical language.

### Whisper Architecture

Whisper is an **encoder-decoder transformer**:
- **Encoder**: Processes audio spectrogram (visual representation of sound)
- **Decoder**: Generates transcription text token-by-token

```mermaid
graph TD
    A[Raw Audio Waveform] --> B[Preprocessing<br/>16kHz resampling]
    B --> C[Log-Mel Spectrogram<br/>80 mel bins, 3000 frames]
    C --> D[Whisper Encoder<br/>Transformer]
    D --> E[Audio Embeddings<br/>1500 x 512]
    E --> F[Whisper Decoder<br/>Transformer + Cross-Attention]
    F --> G[Text Tokens<br/>"bring me water"]
    G --> H[Transcription Output]

    style D fill:#e1f5ff
    style F fill:#ffe1f5
```

### Whisper Model Variants

Whisper comes in multiple sizes, trading off accuracy vs. speed:

| Model | Parameters | Relative Speed | Use Case |
|-------|------------|----------------|----------|
| tiny  | 39M | 32x | Real-time embedded devices |
| base  | 74M | 16x | Lightweight real-time applications |
| small | 244M | 6x | Good balance for robotics |
| medium | 769M | 2x | High accuracy, moderate latency |
| large-v2 | 1550M | 1x | Best accuracy, high latency |

**For humanoid robotics**, we recommend:
- **small** or **medium** for real-time operation (100-300ms latency)
- **large-v2** for offline processing or when latency is not critical

### Whisper Capabilities

**1. Multilingual support**: 99 languages (English, Spanish, Mandarin, etc.)
**2. Noise robustness**: Works in noisy environments (factories, streets)
**3. Accent handling**: Trained on diverse speakers
**4. Punctuation and formatting**: Outputs properly formatted text
**5. Timestamps**: Can provide word-level timestamps for alignment
**6. Special tokens**: Language detection, task type (transcription vs. translation)

### Using Whisper in Python

```python
import whisper
import numpy as np

# Load Whisper model
model = whisper.load_model("small")  # or "tiny", "base", "medium", "large-v2"

# Transcribe audio file
result = model.transcribe("command.wav")
print(result["text"])
# Output: "Bring me a glass of water."

# Transcribe with language detection
result = model.transcribe("command.wav", language="en", fp16=False)
print(f"Detected language: {result['language']}")
print(f"Transcription: {result['text']}")

# Get word-level timestamps
result = model.transcribe("command.wav", word_timestamps=True)
for segment in result["segments"]:
    for word in segment["words"]:
        print(f"{word['word']}: {word['start']:.2f}s - {word['end']:.2f}s")
```

### Real-Time Audio Streaming with Whisper

For robot control, we need **real-time transcription** from a microphone:

```python
import pyaudio
import whisper
import numpy as np
from collections import deque

# Audio parameters
CHUNK = 1024  # Samples per frame
FORMAT = pyaudio.paInt16  # 16-bit audio
CHANNELS = 1  # Mono
RATE = 16000  # 16kHz sample rate
RECORD_SECONDS = 3  # Buffer length

# Initialize Whisper
model = whisper.load_model("small")

# Initialize audio stream
p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE,
                input=True, frames_per_buffer=CHUNK)

print("Listening for commands...")

# Circular buffer for audio chunks
audio_buffer = deque(maxlen=int(RATE / CHUNK * RECORD_SECONDS))

try:
    while True:
        # Read audio chunk
        data = stream.read(CHUNK, exception_on_overflow=False)
        audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        audio_buffer.append(audio_chunk)

        # Detect voice activity (simple energy threshold)
        energy = np.sum(audio_chunk ** 2)
        if energy > 0.01:  # Voice detected
            # Concatenate buffer
            audio_array = np.concatenate(list(audio_buffer))

            # Transcribe
            result = model.transcribe(audio_array, fp16=False, language="en")
            transcription = result["text"].strip()

            if transcription:
                print(f"You said: {transcription}")
                # TODO: Send to LLM for planning

except KeyboardInterrupt:
    print("Stopping...")
finally:
    stream.stop_stream()
    stream.close()
    p.terminate()
```

**Optimization tips**:
1. **Use GPU**: Whisper runs 10x faster on GPU (`device="cuda"`)
2. **Reduce buffer size**: Trade latency for accuracy (1-2 seconds is often enough)
3. **Voice Activity Detection (VAD)**: Only transcribe when speech is detected (saves computation)
4. **Quantization**: Use int8 quantization for 2x speedup with minimal accuracy loss

## Integrating Whisper with ROS 2

To make Whisper available to other ROS 2 nodes, we'll create a **Whisper ASR Node** that publishes transcriptions.

### ROS 2 Whisper Node Architecture

```mermaid
graph TD
    A[Microphone] --> B[Audio Driver<br/>PulseAudio / ALSA]
    B --> C[/audio/raw<br/>audio_msgs/Audio topic]
    C --> D[WhisperNode<br/>rclpy node]
    D --> E[Whisper Model<br/>Transcription]
    E --> F[/speech/transcription<br/>std_msgs/String topic]
    F --> G[LLM Planner Node]
    G --> H[/robot/action_plan<br/>custom ActionPlan msg]

    style D fill:#e1f5ff
    style G fill:#ffe1f5
```

### Code: ROS 2 Whisper Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioStamped
import whisper
import numpy as np
from collections import deque

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Parameters
        self.declare_parameter('model_size', 'small')
        self.declare_parameter('language', 'en')
        self.declare_parameter('buffer_seconds', 3.0)

        model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        buffer_seconds = self.get_parameter('buffer_seconds').value

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = whisper.load_model(model_size)

        # Audio buffer
        self.audio_buffer = deque(maxlen=int(16000 * buffer_seconds))

        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioStamped, '/audio/raw', self.audio_callback, 10
        )

        # Publishers
        self.transcription_pub = self.create_publisher(String, '/speech/transcription', 10)

        self.get_logger().info('Whisper ASR Node ready')

    def audio_callback(self, msg):
        # Convert ROS audio message to numpy array
        audio_chunk = np.frombuffer(msg.audio.data, dtype=np.int16).astype(np.float32) / 32768.0
        self.audio_buffer.append(audio_chunk)

        # Voice activity detection (simple energy threshold)
        energy = np.sum(audio_chunk ** 2)
        if energy > 0.01:
            # Transcribe accumulated audio
            audio_array = np.concatenate(list(self.audio_buffer))
            result = self.model.transcribe(audio_array, language=self.language, fp16=False)
            transcription = result["text"].strip()

            if transcription:
                self.get_logger().info(f'Transcription: {transcription}')
                # Publish transcription
                msg = String()
                msg.data = transcription
                self.transcription_pub.publish(msg)

                # Clear buffer after successful transcription
                self.audio_buffer.clear()

def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File

```xml
<launch>
  <node pkg="whisper_ros" exec="whisper_node" name="whisper_asr">
    <param name="model_size" value="small"/>
    <param name="language" value="en"/>
    <param name="buffer_seconds" value="2.0"/>
  </node>

  <node pkg="audio_capture" exec="audio_capture_node" name="audio_capture">
    <remap from="audio" to="/audio/raw"/>
  </node>
</launch>
```

## LLM Planner: From Text to Action Plans

Once Whisper transcribes speech to text, we need an **LLM planner** to convert natural language commands into structured robot action plans.

### LLM Planning Architecture

```mermaid
graph TD
    A[Transcription<br/>"Bring me water"] --> B[LLM Planner Node]
    B --> C[System Prompt<br/>Robot capabilities]
    C --> D[GPT-4 / Claude API]
    B --> E[Visual Context<br/>Camera image optional]
    E --> D
    D --> F[Action Plan JSON<br/>Navigate, Grasp, Deliver]
    F --> G[Action Plan Topic<br/>/robot/action_plan]

    style D fill:#ffe1f5
```

### LLM System Prompt for Robot Planning

The key to effective LLM planning is a well-designed **system prompt** that describes:
1. Robot capabilities (what actions it can perform)
2. Environment context (current location, objects visible)
3. Output format (structured JSON plan)

```python
ROBOT_SYSTEM_PROMPT = """
You are an AI assistant controlling a humanoid robot. Your task is to convert natural language commands into executable action plans.

Robot Capabilities:
- navigate(location): Move to a location (kitchen, living_room, bedroom)
- detect_objects(query): Detect objects matching a description
- grasp(object_id): Grasp an object
- release(): Release the grasped object
- pour(target_id): Pour liquid into a target container
- handover(person_id): Hand the grasped object to a person

Environment:
- Locations: kitchen, living_room, bedroom, hallway
- Typical objects: mug, glass, bottle, plate, spoon, remote, book

Output Format:
Return a JSON list of actions with parameters:
[
  {"action": "navigate", "params": {"location": "kitchen"}},
  {"action": "detect_objects", "params": {"query": "glass"}},
  {"action": "grasp", "params": {"object_id": "glass_0"}},
  {"action": "navigate", "params": {"location": "living_room"}},
  {"action": "handover", "params": {"person_id": "user"}}
]

Be concise and generate only necessary actions. Handle ambiguous commands by making reasonable assumptions.
"""
```

### LLM Planner Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.msg import ActionPlan, ActionStep
import openai
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Parameters
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('model', 'gpt-4')

        api_key = self.get_parameter('openai_api_key').value
        self.model = self.get_parameter('model').value

        # Initialize OpenAI client
        openai.api_key = api_key

        # Subscribers
        self.transcription_sub = self.create_subscription(
            String, '/speech/transcription', self.transcription_callback, 10
        )

        # Publishers
        self.plan_pub = self.create_publisher(ActionPlan, '/robot/action_plan', 10)

        self.get_logger().info('LLM Planner Node ready')

    def transcription_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Generate action plan using LLM
        action_plan = self.generate_plan(command)

        if action_plan:
            # Publish action plan
            self.plan_pub.publish(action_plan)
            self.get_logger().info(f'Published action plan with {len(action_plan.steps)} steps')

    def generate_plan(self, command):
        try:
            # Call OpenAI API
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": ROBOT_SYSTEM_PROMPT},
                    {"role": "user", "content": command}
                ],
                temperature=0.3,  # Low temperature for deterministic output
                max_tokens=500
            )

            # Parse LLM response
            llm_output = response.choices[0].message.content
            self.get_logger().info(f'LLM response: {llm_output}')

            # Parse JSON action plan
            actions = json.loads(llm_output)

            # Convert to ROS message
            plan_msg = ActionPlan()
            for action in actions:
                step = ActionStep()
                step.action_type = action["action"]
                step.parameters = json.dumps(action["params"])
                plan_msg.steps.append(step)

            return plan_msg

        except Exception as e:
            self.get_logger().error(f'Failed to generate plan: {e}')
            return None

ROBOT_SYSTEM_PROMPT = """
[System prompt from above]
"""

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Command to Action Plan

**User command**: *"Can you bring me a glass of water from the kitchen?"*

**LLM output**:
```json
[
  {"action": "navigate", "params": {"location": "kitchen"}},
  {"action": "detect_objects", "params": {"query": "glass"}},
  {"action": "grasp", "params": {"object_id": "glass_0"}},
  {"action": "detect_objects", "params": {"query": "water bottle"}},
  {"action": "pour", "params": {"target_id": "glass_0", "source_id": "bottle_0"}},
  {"action": "navigate", "params": {"location": "living_room"}},
  {"action": "handover", "params": {"person_id": "user"}}
]
```

## Multimodal Command Processing

Voice commands are more effective when combined with visual context:

### Visual Grounding for Commands

**Problem**: "Pick up that cup" → Which cup?

**Solution**: Combine speech with:
1. **Gaze tracking**: Where is the human looking?
2. **Pointing detection**: Is the human pointing at an object?
3. **Object saliency**: Which object is most visually prominent?

```python
def resolve_ambiguous_reference(command, camera_image, gaze_point=None):
    # Detect objects in image
    objects = detect_objects(camera_image)

    # Parse command for demonstrative ("that", "this", "the red one")
    if "that" in command or "this" in command:
        # Use gaze or pointing to resolve reference
        if gaze_point:
            # Find object closest to gaze point
            target_object = find_closest_object(objects, gaze_point)
        else:
            # Default to most salient object
            target_object = find_most_salient_object(objects)

    # Update command with specific object ID
    resolved_command = command.replace("that", f"object_{target_object.id}")
    return resolved_command
```

### Context-Aware Planning

LLMs can incorporate **robot state** and **environment context** for better planning:

```python
def generate_plan_with_context(command, robot_state, camera_image):
    # Encode context
    context = {
        "robot_location": robot_state.location,
        "grasped_object": robot_state.grasped_object,
        "visible_objects": detect_objects(camera_image),
        "battery_level": robot_state.battery
    }

    # Enhanced system prompt with context
    prompt = f"""
    Robot Status:
    - Current location: {context['robot_location']}
    - Grasped object: {context['grasped_object']}
    - Visible objects: {context['visible_objects']}
    - Battery: {context['battery_level']}%

    User command: {command}

    Generate an action plan considering the current robot state.
    """

    # Call LLM
    plan = llm_api_call(prompt)
    return plan
```

**Example benefit**:
- Command: "Put it down"
- Without context: LLM doesn't know what "it" refers to
- With context: `grasped_object = "mug"` → LLM generates `release()` action

## Error Handling and Ambiguity Resolution

Voice commands are inherently ambiguous. Robust voice-to-action systems must handle:

### 1. Transcription Errors

**Problem**: Whisper mishears "glass" as "grass"

**Solution**: Semantic validation
```python
def validate_transcription(transcription, expected_domain):
    # Check if transcription contains domain-relevant words
    robot_vocabulary = ["bring", "pick", "move", "grasp", "navigate", "glass", "mug", "kitchen"]
    words = transcription.lower().split()

    if not any(word in robot_vocabulary for word in words):
        return False, "Transcription contains no robot-related words"

    return True, transcription
```

### 2. Ambiguous Commands

**Problem**: "Bring me something to drink"

**Solution**: LLM generates clarification question
```python
def handle_ambiguous_command(command):
    # Ask LLM to identify ambiguity
    prompt = f"Is this command ambiguous? '{command}'. If yes, suggest a clarification question."
    llm_response = llm_api_call(prompt)

    if "ambiguous" in llm_response.lower():
        clarification = llm_response.split("Question:")[1]
        # Use text-to-speech to ask user
        speak(clarification)
        # Wait for user response
        response = listen()
        # Refine command
        refined_command = f"{command}. Specifically: {response}"
        return refined_command

    return command
```

### 3. Impossible Commands

**Problem**: "Fly to the ceiling"

**Solution**: LLM detects infeasibility
```python
FEASIBILITY_CHECK_PROMPT = """
Given the robot capabilities:
- Navigate on the ground
- Grasp objects
- Manipulate objects

Is the following command feasible? If not, explain why and suggest an alternative.

Command: {command}
"""

def check_feasibility(command):
    response = llm_api_call(FEASIBILITY_CHECK_PROMPT.format(command=command))
    if "not feasible" in response.lower():
        speak("I cannot do that because " + response)
        return False
    return True
```

## Real-World Deployment Considerations

### 1. Latency Requirements

Voice-controlled robots must respond quickly to maintain natural interaction:

| Component | Target Latency | Optimization |
|-----------|----------------|--------------|
| Audio capture | &lt;10ms | Hardware buffering |
| Whisper ASR | 100-300ms | Use small/medium models, GPU |
| LLM planning | 500-2000ms | Cache common plans, use faster models |
| VLA execution | Variable | Depends on task (1-30s) |
| **Total (Hear to Act)** | &lt;3s | Acceptable for most tasks |

**Latency reduction strategies**:
- **Streaming ASR**: Start transcribing before user finishes speaking
- **Speculative planning**: Generate multiple plans in parallel, select best one
- **Plan caching**: Store common command-plan mappings (e.g., "bring water" → precomputed plan)

### 2. Robustness to Noise

Real-world environments are noisy (music, other people talking, machinery):

**Solutions**:
- **Directional microphones**: Focus on user's voice
- **Beamforming**: Use microphone array to spatially filter audio
- **Wake word detection**: Only transcribe after hearing "Hey robot" (reduces false activations)
- **Noise suppression**: Preprocess audio with RNNoise or similar

### 3. Privacy and Security

Voice commands may contain sensitive information:

**Best practices**:
- **On-device ASR**: Run Whisper locally, don't send audio to cloud
- **Ephemeral transcriptions**: Delete audio and transcriptions after processing
- **User authentication**: Verify user identity via voiceprint or visual confirmation before executing critical commands

### 4. Multi-User Management

How does the robot handle multiple people speaking?

**Solutions**:
- **Speaker diarization**: Identify who is speaking using voice embeddings
- **Priority system**: Designated "primary user" commands override others
- **Conflict resolution**: If two users give conflicting commands, ask for clarification

## Integration: Complete Voice-to-Action Pipeline

Bringing it all together:

```mermaid
graph TD
    A[User: "Bring me water"] --> B[Microphone Array]
    B --> C[Whisper Node<br/>/speech/transcription]
    C --> D[Text: "Bring me water"]
    D --> E[LLM Planner Node<br/>GPT-4]
    E --> F[ActionPlan Message<br/>7 steps]
    F --> G[Action Executor Node]

    G --> H{Action Type}
    H -->|navigate| I[Nav2 MoveBase<br/>Path planning]
    H -->|detect_objects| J[Vision Node<br/>Object detection]
    H -->|grasp| K[VLA Policy<br/>Manipulation]

    I --> L[Humanoid Robot<br/>Base motors]
    J --> M[Robot Cameras<br/>Perception]
    K --> N[Robot Arms<br/>Manipulation]

    M --> G
    N --> G
    L --> G

    style C fill:#e1f5ff
    style E fill:#ffe1f5
    style K fill:#e1ffe1
```

**Data flow**:
1. User speaks command → Microphone captures audio
2. Whisper Node transcribes audio → Publishes to `/speech/transcription`
3. LLM Planner Node receives transcription → Calls GPT-4 API → Publishes `ActionPlan` to `/robot/action_plan`
4. Action Executor Node receives plan → Executes actions sequentially
5. For each action:
   - `navigate`: Calls Nav2 service
   - `detect_objects`: Calls vision node service
   - `grasp`: Calls VLA policy node for manipulation
6. Executor monitors action completion → Proceeds to next action
7. Final action complete → Robot returns to idle state

## Questions and Answers

**Q1: How accurate is Whisper for robot commands?**

Whisper achieves ~5% Word Error Rate (WER) on clean English speech, but performance degrades with noise:
- Clean environment: 95-98% accuracy
- Moderate noise (office, home): 85-90% accuracy
- High noise (factory, street): 70-80% accuracy

For critical commands, implement **confirmation loops**: Robot repeats back what it heard and asks for confirmation before executing.

**Q2: Can I use Whisper for multiple languages?**

Yes! Whisper supports 99 languages. For multilingual robots:
```python
# Auto-detect language
result = model.transcribe(audio, task="transcribe")
detected_language = result["language"]

# Or specify language
result = model.transcribe(audio, language="es")  # Spanish
```

However, LLM planning may be less effective in non-English languages depending on your LLM's training data.

**Q3: How do I handle long commands or conversations?**

For extended interactions, maintain **conversation context**:
```python
conversation_history = []

def process_command(transcription):
    # Add user message to history
    conversation_history.append({"role": "user", "content": transcription})

    # Call LLM with full history
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "system", "content": ROBOT_SYSTEM_PROMPT}] + conversation_history
    )

    # Add assistant response to history
    assistant_message = response.choices[0].message.content
    conversation_history.append({"role": "assistant", "content": assistant_message})

    return assistant_message
```

This enables follow-up commands like:
- User: "Bring me a glass"
- Robot: "What would you like in the glass?"
- User: "Water"  ← Robot understands "water" refers to previous context

**Q4: What if the LLM generates an unsafe plan?**

Implement a **safety shield** that validates plans before execution:
```python
def validate_plan_safety(plan, robot_state):
    for action in plan.steps:
        # Check collision risk
        if action.action_type == "navigate":
            path = compute_path(robot_state.location, action.params["location"])
            if path_has_collision(path):
                return False, "Path would collide with obstacles"

        # Check battery
        if robot_state.battery < 20 and action.action_type == "navigate":
            return False, "Battery too low for navigation"

        # Check hardware limits
        if action.action_type == "grasp" and robot_state.grasped_object:
            return False, "Already grasping an object"

    return True, "Plan is safe"
```

**Q5: Can I use local LLMs instead of OpenAI API?**

Yes! Use open-source LLMs (LLaMA, Mistral, Phi) with **llama.cpp** or **Ollama**:
```python
import ollama

def generate_plan_local(command):
    response = ollama.chat(
        model="mistral:7b",
        messages=[
            {"role": "system", "content": ROBOT_SYSTEM_PROMPT},
            {"role": "user", "content": command}
        ]
    )
    return response["message"]["content"]
```

Benefits: Privacy, no API costs, offline operation
Tradeoffs: Lower quality than GPT-4, requires local GPU

## Connections to Other Modules

- **Chapter 5 (ROS 2 Communication)**: Voice-to-action pipeline uses ROS 2 topics and services for inter-node communication
- **Chapter 6 (Python-ROS Bridge)**: Whisper and LLM nodes are implemented in Python using rclpy
- **Chapter 16 (VLA Models)**: LLM-generated action plans are executed by VLA policies
- **Chapter 18 (LLM Task Planning)**: Deep dive into LLM planning algorithms and prompt engineering
- **Chapter 19 (Capstone)**: Integrates speech, vision, and action for complete autonomous humanoid system

## Summary

The voice-to-action pipeline enables natural language control of humanoid robots by converting spoken commands into executable action plans.

**Key takeaways**:
1. **Whisper ASR**: OpenAI's speech recognition model transcribes audio to text with 85-98% accuracy
2. **ROS 2 Integration**: Whisper Node publishes transcriptions to `/speech/transcription` topic
3. **LLM Planning**: GPT-4 or local LLMs convert natural language to structured action plans (JSON)
4. **Multimodal Grounding**: Combine speech with vision for resolving ambiguous references ("that cup")
5. **Error Handling**: Validate transcriptions, detect ambiguity, check plan feasibility
6. **Deployment**: Optimize for latency (&lt;3s), robustness to noise, privacy, and safety

Voice control transforms humanoid robots from programmable machines to **conversational agents** that anyone can instruct using natural language. By combining Whisper's robust speech recognition with LLMs' reasoning capabilities, we enable robots to understand commands, plan actions, and execute tasks in unstructured real-world environments.

The next chapter explores how LLMs generate these action plans in detail, including advanced prompting techniques, few-shot learning, and hierarchical task decomposition.

## References

1. Radford, A., Kim, J. W., Xu, T., et al. (2022). Robust Speech Recognition via Large-Scale Weak Supervision. *arXiv preprint arXiv:2212.04356* (Whisper).

2. Ahn, M., Brohan, A., Brown, N., et al. (2022). Do As I Can, Not As I Say: Grounding Language in Robotic Affordances. *Conference on Robot Learning (CoRL)* (PaLM-SayCan).

3. Huang, W., Xia, F., Shah, D., et al. (2023). Grounded Decoding: Guiding Text Generation with Grounded Models for Embodied AI. *arXiv preprint arXiv:2303.00855*.

4. Hu, R., Girdhar, R., Gupta, A., & Darrell, T. (2023). Prompter: Utilizing Large Language Model Prompting for a Data Efficient Embodied Instruction Following. *arXiv preprint arXiv:2211.03267*.

5. Liang, J., Huang, W., Xia, F., et al. (2023). Code as Policies: Language Model Programs for Embodied Control. *IEEE International Conference on Robotics and Automation (ICRA)*.

6. Stone, A., Xiao, T., Lu, Y., et al. (2023). Open-World Object Manipulation using Pre-trained Vision-Language Models. *arXiv preprint arXiv:2303.00905*.

7. Driess, D., Xia, F., Sajjadi, M. S., et al. (2023). PaLM-E: An Embodied Multimodal Language Model. *International Conference on Machine Learning (ICML)*.

8. Wake, N., Kanehira, A., Sasabuchi, K., et al. (2023). ChatGPT Empowered Long-Step Robot Control in Various Environments. *arXiv preprint arXiv:2304.03893*.

9. Singh, I., Blukis, V., Mousavian, A., et al. (2023). ProgPrompt: Generating Situated Robot Task Plans using Large Language Models. *IEEE International Conference on Robotics and Automation (ICRA)*.

10. Vemprala, S., Bonatti, R., Bucker, A., & Kapoor, A. (2023). ChatGPT for Robotics: Design Principles and Model Abilities. *Microsoft Research Technical Report*.
