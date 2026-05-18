#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr


WAKE_WORD = "robot"   # change if you prefer a different trigger word


class VoiceCommander(Node):
    def __init__(self):
        super().__init__('voice_commander')
        self.publisher_ = self.create_publisher(String, '/voice_commands', 10)
        self.recognizer = sr.Recognizer()

        # ── Pick the right microphone ────────────────────────────────────────
        self.get_logger().info(" Available microphones:")
        for i, name in enumerate(sr.Microphone.list_microphone_names()):
            self.get_logger().info(f"   [{i}] {name}")

        # Use default mic (index=None). Change to sr.Microphone(device_index=N)
        # if the wrong device is selected.
        self.mic = sr.Microphone()

        # ── Ambient noise calibration ────────────────────────────────────────
        self.get_logger().info(" Calibrating for ambient noise — stay quiet for 2 s...")
        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
        self.get_logger().info(
            f"Calibration done. Energy threshold = "
            f"{self.recognizer.energy_threshold:.0f}"
        )

        # ── Start background listening ───────────────────────────────────────
        self.stop_listening = self.recognizer.listen_in_background(
            self.mic, self._audio_callback, phrase_time_limit=5
        )
        self.get_logger().info(
            f"🚀 Listening! Say '{WAKE_WORD.upper()} <command>' to control the arm."
        )
        self.get_logger().info(
            "   Example commands:"
        )
        self.get_logger().info("     'Robot home'")
        self.get_logger().info("     'Robot move shoulder 30 degrees'")
        self.get_logger().info("     'Robot open gripper'")
        self.get_logger().info("     'Robot close gripper'")

    # ────────────────────────────────────────────────────────────────────────
    def _audio_callback(self, recognizer: sr.Recognizer, audio: sr.AudioData):
        """Called automatically every time speech is detected."""
        try:
            text = recognizer.recognize_google(audio).lower()
            self.get_logger().info(f"Heard: '{text}'")

            if WAKE_WORD in text:
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published to /voice_commands: '{text}'")
            else:
                self.get_logger().info(
                    f"   (wake word '{WAKE_WORD}' not found — ignoring)"
                )

        except sr.UnknownValueError:
            # Audio was captured but speech could not be understood — normal
            self.get_logger().debug("Audio captured but not understood")

        except sr.RequestError as e:
            # Google Speech API network/quota error
            self.get_logger().error(f"Google Speech API error: {e}")

        except Exception as e:
            self.get_logger().error(f"Unexpected error in audio callback: {e}")


# ── Entry point ───────────────────────────────────────────────────────────────
# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = VoiceCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down voice commander.")
    finally:
        node.stop_listening(wait_for_stop=False)
        node.destroy_node()
        
        # Check if rclpy is still running before trying to shut it down
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()