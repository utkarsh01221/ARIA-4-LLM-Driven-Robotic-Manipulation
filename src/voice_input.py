import speech_recognition as sr


class VoiceInput:

    def __init__(self):
        self.recognizer = sr.Recognizer()

        self.microphone = sr.Microphone()

        print("========================================")
        print("          VOICE INPUT READY")
        print("========================================")

        # Calibrate microphone for background noise
        with self.microphone as source:
            print("Calibrating microphone...")
            self.recognizer.adjust_for_ambient_noise(
                source,
                duration=1
            )

        print("Microphone calibrated.")

    def listen(self):

        with self.microphone as source:

            print("\n========================================")
            print("          LISTENING...")
            print("========================================")
            print("Speak your robot command.")

            try:
                audio = self.recognizer.listen(
                    source,
                    timeout=5,
                    phrase_time_limit=10
                )

            except sr.WaitTimeoutError:
                print("No speech detected.")
                return None

        print("Processing speech...")

        try:
            command = self.recognizer.recognize_google(
                audio
            )

            print("\nRecognized command:")
            print(command)

            return command

        except sr.UnknownValueError:

            print("Could not understand the speech.")
            return None

        except sr.RequestError as e:

            print(f"Speech recognition service error: {e}")
            return None