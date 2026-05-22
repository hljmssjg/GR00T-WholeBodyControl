"""Optional text-to-speech feedback for data collection.

Two backends:
  - "espeak": speak on the PC via pyttsx3/espeak (needs PC audio output).
  - "g1": speak through the Unitree G1's onboard speaker via the robot audio
    service. Hands-free for the teleop operator (no headphones/cable). The G1
    voice reads Chinese well; English comes out garbled, so messages routed
    here should be Chinese.
If the requested backend fails to initialise, we fall back to espeak, and if
that also fails, speaking is silently disabled (printing still works).
"""

import threading


class TextToSpeech:
    def __init__(
        self,
        rate: int = 150,
        volume: float = 1.0,
        backend: str = "espeak",
        g1_iface: str = "enp2s0",
        g1_speaker_id: int = 0,
    ):
        self.backend = backend
        self.engine = None  # espeak engine (pyttsx3)
        self._g1_client = None  # Unitree G1 audio client
        self._g1_speaker_id = g1_speaker_id

        if self.backend == "g1":
            try:
                from unitree_sdk2py.core.channel import ChannelFactoryInitialize
                from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

                ChannelFactoryInitialize(0, g1_iface)
                client = AudioClient()
                try:
                    client.SetTimeout(5.0)
                except Exception:
                    pass
                client.Init()
                self._g1_client = client
                print(f"[Text To Speech] G1 audio backend ready (iface={g1_iface})")
            except Exception as e:
                print(f"[Text To Speech] G1 audio init failed ({e}); falling back to espeak")
                self.backend = "espeak"

        if self.backend == "espeak":
            try:
                import pyttsx3

                self.engine = pyttsx3.init(driverName="espeak")
                self.engine.setProperty("rate", rate)
                self.engine.setProperty("volume", volume)
            except Exception as e:
                print(f"[Text To Speech] Initialization failed: {e}")
                self.engine = None

        self._speech_thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def _available(self) -> bool:
        return self._g1_client is not None or self.engine is not None

    def say(self, message: str, blocking: bool = False):
        if not self._available():
            return
        if blocking:
            self._say_blocking(message)
        else:
            thread = threading.Thread(target=self._say_blocking, args=(message,), daemon=True)
            thread.start()
            self._speech_thread = thread

    def _say_blocking(self, message: str):
        with self._lock:
            try:
                if self._g1_client is not None:
                    self._g1_client.TtsMaker(message, self._g1_speaker_id)
                elif self.engine is not None:
                    self.engine.say(message)
                    self.engine.runAndWait()
            except RuntimeError:
                pass
            except Exception as e:
                print(f"[Text To Speech] say failed: {e}")

    def wait_for_completion(self):
        if self._speech_thread and self._speech_thread.is_alive():
            self._speech_thread.join()

    def print_and_say(self, message: str, say: bool = True, blocking: bool = False):
        print(message)
        if say and self._available():
            self.say(message, blocking=blocking)
