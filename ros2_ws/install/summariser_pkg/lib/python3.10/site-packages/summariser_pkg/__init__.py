import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformers import pipeline

class SummariserNode(Node):
    def __init__(self):
        super().__init__('summariser_node')

        self.subscription = self.create_subscription(
            String,
            '/summary_input',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/summary_output',
            10
        )

        self.summariser = pipeline("summarization", model="facebook/bart-large-cnn")
        self.get_logger().info("Summariser node initialized.")

    def summarise_text(self, text):
        try:
            summary = self.summariser(text, max_length=130, min_length=30, do_sample=False)
            return summary[0]['summary_text']
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            return "Summary failed."

    def listener_callback(self, msg):
        input_text = msg.data
        summary = self.summarise_text(input_text)
        self.publisher.publish(String(data=summary))
        self.get_logger().info(f"Published summary: {summary}")

def main(args=None):
    rclpy.init(args=args)
    node = SummariserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()