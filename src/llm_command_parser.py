import requests
import json


class LLMCommandParser:

    OLLAMA_URL = "http://localhost:11434/api/chat"
    MODEL = "qwen3:4b"

    COLORS = {"red", "blue", "green", "yellow"}

    def parse(self, command):

        if not isinstance(command, str) or not command.strip():
            raise ValueError("Command cannot be empty.")

        prompt = f"""
You are an industrial robot command parser.

Your job is to convert the user's natural-language command
into a robot instruction.

Allowed action:
- pick_and_place

Allowed object colors:
- red
- blue
- green
- yellow

Interpret words such as:
- pick
- grab
- take
- move
- get

as the source object.

Interpret words such as:
- place
- put
- drop
- move onto
- move to

as the target object.

Return ONLY valid JSON.

Required JSON format:

{{
    "action": "pick_and_place",
    "source": "red",
    "target": "green"
}}

Rules:
1. source must be one of: red, blue, green, yellow
2. target must be one of: red, blue, green, yellow
3. source and target must be different
4. Do not invent colors.
5. Do not return explanations.
6. Do not return markdown.
7. Return exactly one JSON object.

User command:
{command}
"""

        payload = {
            "model": self.MODEL,
            "messages": [
                {
                    "role": "user",
                    "content": prompt
                }
            ],
            "stream": False,
            "format": "json",
            "think": False,
            "options": {
                "temperature": 0
            }
        }

        try:
            response = requests.post(
                self.OLLAMA_URL,
                json=payload,
                timeout=120
            )

            response.raise_for_status()

        except requests.RequestException as e:
            raise RuntimeError(
                f"Could not connect to Ollama: {e}"
            )

        try:
            data = response.json()
            content = data["message"]["content"]
            instruction = json.loads(content)

        except (KeyError, json.JSONDecodeError) as e:
            raise ValueError(
                f"Invalid JSON returned by LLM: {e}"
            )

        self.validate(instruction)

        print("\n========================================")
        print("        LLM COMMAND PARSED")
        print("========================================")
        print(f"Action : {instruction['action']}")
        print(f"Source : {instruction['source']}")
        print(f"Target : {instruction['target']}")
        print("========================================")

        return instruction

    def validate(self, instruction):

        if not isinstance(instruction, dict):
            raise ValueError("LLM output must be a JSON object.")

        if instruction.get("action") != "pick_and_place":
            raise ValueError("Unsupported robot action.")

        source = instruction.get("source")
        target = instruction.get("target")

        if source not in self.COLORS:
            raise ValueError(
                f"Invalid source color: {source}"
            )

        if target not in self.COLORS:
            raise ValueError(
                f"Invalid target color: {target}"
            )

        if source == target:
            raise ValueError(
                "Source and target cannot be the same."
            )