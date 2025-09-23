import queue
import sounddevice as sd
import json
from vosk import Model, KaldiRecognizer

class DummyVoiceListener:
    def __init__(self):
        print("Dummy Voice Listener Initialized.")
        self.commands = ["up", "down", "left", "right", "stop", "circle", "semi_circle", "full_circle"]
        print(f"Available commands: {', '.join(self.commands)}")

        # Initialize Vosk
        model_path = "/Users/manu/ROS/turtle_art/src/ninja_turtle/model"
        self.model = Model(model_path)
        self.q = queue.Queue()
        self.rec = KaldiRecognizer(self.model, 16000)

        # Start audio stream
        self.stream = sd.InputStream(samplerate=16000, channels=1, callback=self.audio_callback)
        self.stream.start()
        self.listen_loop()

    def audio_callback(self, indata, frames, time, status):
        if status:
            print(status)
        if indata.any():
            
            #print("Audio data received!") # Uncomment for verbose debugging
            pass
        self.q.put(bytes(indata))

    def listen_loop(self):
        print("Speak now...")
        while True:
            data = self.q.get()
            if self.rec.AcceptWaveform(data):
                result = json.loads(self.rec.Result())
                text = result.get("text", "").lower()
                print(f"Full Vosk Result: {result}") # Added for debugging
                if text:
                    print(f"Heard (raw): {text}")
                    cmd = self.parse_command(text)
                    if cmd:
                        print(f"Command recognized: {cmd}")
                # else:
                    # print(f"Vosk result had no text. Full result: {result}")
            else:
                partial = json.loads(self.rec.PartialResult())
                if partial.get("partial"): # Only print if there's an actual partial result
                    print(f"Partial: {partial.get('partial')}")

    def parse_command(self, text):
        for cmd in self.commands:
            if cmd in text:
                return cmd
        return None


def main():
    listener = DummyVoiceListener()

if __name__ == '__main__':
    main()