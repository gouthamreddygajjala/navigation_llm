"""
LLM Navigator Module for TurtleBot3 House Environment
Translates natural language instructions into precise Nav2 navigation goals.
Integrates with local Ollama Qwen 14B model with intelligent semantic fallback.
"""

import json
import math
import os
import re
import urllib.error
import urllib.request
from typing import Any, Dict, Optional, Tuple

HOUSE_ROOMS = {
    'living_room': {
        'id': 'living_room',
        'name': 'Living Room',
        'icon': '🛋️',
        'x': 5.20,
        'y': 4.67,
        'yaw': 0.0,
        'color': '#38bdf8',
        'description': 'Main living room with sofa, marble coffee table, and bookshelves.',
        'keywords': ['living room', 'lounge', 'sofa', 'tv', 'marble table', 'couch', 'relax', 'hall']
    },
    'dining_room': {
        'id': 'dining_room',
        'name': 'Dining Room / Cafe',
        'icon': '🍽️',
        'x': 7.95,
        'y': -3.73,
        'yaw': -1.57,
        'color': '#f59e0b',
        'description': 'Dining area equipped with cafe dining tables and chairs.',
        'keywords': ['dining room', 'dining', 'cafe', 'cafe table', 'eat', 'dinner', 'lunch', 'breakfast']
    },
    'kitchen': {
        'id': 'kitchen',
        'name': 'Kitchen',
        'icon': '🍳',
        'x': -4.50,
        'y': -2.43,
        'yaw': -1.57,
        'color': '#10b981',
        'description': 'Kitchen area with cooking cabinets, counters, and food storage.',
        'keywords': ['kitchen', 'cabinet', 'cooking', 'fridge', 'cook', 'food', 'drink', 'water', 'tea', 'coffee', 'pantry']
    },
    'office': {
        'id': 'office',
        'name': 'Office / Workspace',
        'icon': '💼',
        'x': -2.25,
        'y': 1.22,
        'yaw': 1.57,
        'color': '#8b5cf6',
        'description': 'Workspace with large office table, desk, and executive storage.',
        'keywords': ['office', 'desk', 'work', 'work room', 'meeting room', 'conference', 'table', 'computer']
    },
    'study': {
        'id': 'study',
        'name': 'Study / Library',
        'icon': '📚',
        'x': -4.20,
        'y': 4.02,
        'yaw': 1.57,
        'color': '#ec4899',
        'description': 'Quiet study room equipped with full-height bookshelves and reading desk.',
        'keywords': ['study', 'library', 'bookshelf', 'books', 'reading', 'read', 'bookcase', 'study room']
    },
    'foyer': {
        'id': 'foyer',
        'name': 'Foyer / Entrance',
        'icon': '🚪',
        'x': 0.00,
        'y': 0.60,
        'yaw': 0.0,
        'color': '#6366f1',
        'description': 'Main entrance hallway, start dock, and foyer with mailbox.',
        'keywords': ['foyer', 'hallway', 'entrance', 'dock', 'home', 'start', 'front door', 'mailbox', 'exit', 'corridor']
    }
}


class LLMNavigator:
    def __init__(self, ollama_url: str = 'http://localhost:11434', preferred_model: str = 'qwen2.5:14b'):
        self.ollama_url = ollama_url.rstrip('/')
        self.preferred_model = preferred_model

    def get_nearest_room(self, x: float, y: float) -> Tuple[str, float]:
        min_dist = float('inf')
        nearest = 'foyer'
        for room_id, r in HOUSE_ROOMS.items():
            d = math.hypot(x - r['x'], y - r['y'])
            if d < min_dist:
                min_dist = d
                nearest = room_id
        return nearest, min_dist

    def get_room_distance_list(self, x: float, y: float):
        result = []
        for room_id, r in HOUSE_ROOMS.items():
            dist = round(math.hypot(x - r['x'], y - r['y']), 2)
            result.append({
                'id': room_id,
                'name': r['name'],
                'icon': r['icon'],
                'x': r['x'],
                'y': r['y'],
                'yaw': r['yaw'],
                'color': r['color'],
                'distance': dist,
                'description': r['description']
            })
        return sorted(result, key=lambda it: it['distance'])

    def check_ollama_models(self):
        try:
            req = urllib.request.Request(f"{self.ollama_url}/api/tags")
            with urllib.request.urlopen(req, timeout=1.5) as resp:
                data = json.loads(resp.read().decode('utf-8'))
                models = [m.get('name', '') for m in data.get('models', [])]
                return models
        except Exception:
            return []

    def select_active_model(self):
        models = self.check_ollama_models()
        # Check preferred 14B model first
        for m in models:
            if '14b' in m.lower():
                return m, True
        # Check any other Qwen or LLM installed
        for m in models:
            if 'qwen' in m.lower() or 'llama' in m.lower():
                return m, True
        if models:
            return models[0], True
        return self.preferred_model, False

    def query(self, prompt: str, robot_pose: Dict[str, float]) -> Dict[str, Any]:
        rx = robot_pose.get('x', -2.0)
        ry = robot_pose.get('y', -0.5)
        nearest_room_id, dist_to_nearest = self.get_nearest_room(rx, ry)
        nearest_room = HOUSE_ROOMS.get(nearest_room_id, {})
        nearest_room_name = nearest_room.get('name', 'Foyer')

        active_model, is_installed = self.select_active_model()

        if is_installed:
            llm_result = self._call_ollama(prompt, active_model, rx, ry, nearest_room_name)
            if llm_result:
                return llm_result

        # Fast semantic keyword fallback if model is still downloading or Ollama is offline
        return self._semantic_parse(prompt, rx, ry, nearest_room_name, active_model, is_installed)

    def _call_ollama(self, prompt: str, model_name: str, rx: float, ry: float, nearest_name: str) -> Optional[Dict[str, Any]]:
        rooms_summary = "\n".join([
            f"- {k}: {v['name']} at ({v['x']}, {v['y']}). {v['description']}"
            for k, v in HOUSE_ROOMS.items()
        ])

        system_prompt = f"""You are the autonomous navigation agent for a TurtleBot3 robot inside a multi-room house.
The house contains the following rooms:
{rooms_summary}

Current Robot Location: x={rx:.2f}, y={ry:.2f} (Nearest Room: {nearest_name})

User Prompt: "{prompt}"

Instructions:
1. Determine if the user wants the robot to go to a specific room or location in the house.
2. If yes, specify the exact target_room id ('living_room', 'dining_room', 'kitchen', 'office', 'study', 'foyer').
3. If no (e.g. general question, greeting, or unclear), set target_room to null.
4. Respond ONLY with a valid JSON object matching this schema:
{{
  "target_room": "<room_id or null>",
  "confidence": <float 0.0 to 1.0>,
  "thought": "<concise 1-sentence reasoning>",
  "reply": "<friendly conversational response>"
}}"""

        try:
            req_data = json.dumps({
                "model": model_name,
                "prompt": system_prompt,
                "stream": False,
                "format": "json",
                "options": {
                    "temperature": 0.1,
                    "num_predict": 150
                }
            }).encode('utf-8')

            req = urllib.request.Request(
                f"{self.ollama_url}/api/generate",
                data=req_data,
                headers={"Content-Type": "application/json"}
            )
            with urllib.request.urlopen(req, timeout=45.0) as resp:
                res_json = json.loads(resp.read().decode('utf-8'))
                raw_response = res_json.get('response', '')
                parsed = json.loads(raw_response)

                target_room = parsed.get('target_room')
                if target_room and target_room in HOUSE_ROOMS:
                    target_info = HOUSE_ROOMS[target_room]
                    return {
                        'success': True,
                        'action': 'navigate',
                        'target_room': target_room,
                        'room_name': target_info['name'],
                        'x': target_info['x'],
                        'y': target_info['y'],
                        'yaw': target_info['yaw'],
                        'thought': parsed.get('thought', f"Matched target {target_info['name']}"),
                        'reply': parsed.get('reply', f"Heading over to the {target_info['name']}!"),
                        'model': model_name
                    }
                else:
                    return {
                        'success': True,
                        'action': 'chat',
                        'target_room': None,
                        'thought': parsed.get('thought', 'No navigation target specified'),
                        'reply': parsed.get('reply', raw_response),
                        'model': model_name
                    }
        except Exception:
            return None

    def _semantic_parse(self, prompt: str, rx: float, ry: float, nearest_name: str, model_name: str, is_installed: bool) -> Dict[str, Any]:
        p_lower = prompt.lower().strip()

        matched_room_id = None
        best_match_score = 0

        for room_id, r in HOUSE_ROOMS.items():
            for kw in r['keywords']:
                if re.search(r'\b' + re.escape(kw) + r'\b', p_lower):
                    score = len(kw)
                    if score > best_match_score:
                        best_match_score = score
                        matched_room_id = room_id

        model_status_note = ""
        if not is_installed:
            model_status_note = f" (Local {model_name} is downloading in background)"

        if matched_room_id:
            target_info = HOUSE_ROOMS[matched_room_id]
            dist = math.hypot(rx - target_info['x'], ry - target_info['y'])
            reply = f"Roger that! Navigating to the {target_info['name']} ({dist:.1f}m away).{model_status_note}"
            return {
                'success': True,
                'action': 'navigate',
                'target_room': matched_room_id,
                'room_name': target_info['name'],
                'x': target_info['x'],
                'y': target_info['y'],
                'yaw': target_info['yaw'],
                'thought': f"Matched keywords for {target_info['name']} from user command.",
                'reply': reply,
                'model': f"{model_name} (Fast Semantic Engine)"
            }

        # Handling greetings or queries
        if any(w in p_lower for w in ['where', 'location', 'position']):
            return {
                'success': True,
                'action': 'chat',
                'target_room': None,
                'thought': "User asked for robot location.",
                'reply': f"I am currently near the {nearest_name} at coordinates ({rx:.2f}, {ry:.2f}).{model_status_note}",
                'model': f"{model_name} (Fast Semantic Engine)"
            }

        if any(w in p_lower for w in ['hello', 'hi', 'hey']):
            return {
                'success': True,
                'action': 'chat',
                'target_room': None,
                'thought': "User greeting.",
                'reply': f"Hello! I am your TurtleBot3 autonomous house assistant. Ask me to navigate to any room (Living Room, Kitchen, Dining Room, Office, Study, or Foyer)!{model_status_note}",
                'model': f"{model_name} (Fast Semantic Engine)"
            }

        return {
            'success': True,
            'action': 'chat',
            'target_room': None,
            'thought': "Could not identify a target room in the prompt.",
            'reply': f"I could not identify which room you would like me to visit. You can tell me: 'Go to the Kitchen', 'Head to the Study', 'Take me to the Living Room', etc.{model_status_note}",
            'model': f"{model_name} (Fast Semantic Engine)"
        }
