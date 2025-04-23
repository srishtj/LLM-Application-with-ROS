# 🤖 LLM-Application-with-ROS

This project implements a ROS2 node that integrates a transformer-based language model for real-time text summarization. The node listens to a ROS topic for incoming messages, summarises the content using Hugging Face's `facebook/bart-large-cnn` model, and publishes the summarized output to another ROS topic.

## Features
- Real-time summarization using a large language model (BART)
- ROS2 integration for communication between nodes
- Publish-subscribe model with `/summary_input` and `/summary_output` topics
- Modular code with exception handling and terminal logging

## How It Works
1. Input text is published to `/summary_input` (of type `std_msgs/String`)
2. `SummariserNode` subscribes, receives the input, and generates a summary
3. The summarized text is published to `/summary_output`
4. Terminal displays logging info for debugging and confirmation

## Technologies Used
- ROS2 (rclpy)
- Python
- Hugging Face Transformers (`facebook/bart-large-cnn`)
- `std_msgs/String` for message passing

## Setup Instructions
1. Ensure you have a working ROS2 and Python environment
2. Clone this repo and navigate to the workspace:
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```
3. Run the summariser node:
   ```bash
   ros2 run summariser_pkg summariser_node
   ```
4. Publish test input:
   ```bash
   ros2 topic pub /summary_input std_msgs/String "data: 'Insert your text here to be summarized.'"
   ```
5. View the summarized result on:
   ```bash
   ros2 topic echo /summary_output
   ```

