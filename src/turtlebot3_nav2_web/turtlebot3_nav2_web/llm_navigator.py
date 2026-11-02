"""
LLM Navigator Module for TurtleBot3 House Environment
Translates natural language instructions into precise Nav2 navigation goals.
Supports:
1. Single room navigation ("Go to kitchen")
2. Multi-waypoint navigation ("Go to living room and then kitchen and then dining")
3. Relative motion ("Move forward 1 meter", "Rotate 20 degrees", "Move forward 1m and take left 1m")
Integrates with local Ollama Qwen 14B model with intelligent semantic fallback.
"""

import json
import math
import os
import re
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

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
        rx = float(robot_pose.get('x', 0.0))
        ry = float(robot_pose.get('y', 0.6))
        ryaw = float(robot_pose.get('yaw', 0.0))
        nearest_room_id, dist_to_nearest = self.get_nearest_room(rx, ry)
        nearest_room = HOUSE_ROOMS.get(nearest_room_id, {})
        nearest_room_name = nearest_room.get('name', 'Foyer')

        active_model, is_installed = self.select_active_model()

        if is_installed:
            llm_result = self._call_ollama(prompt, active_model, rx, ry, ryaw, nearest_room_name)
            if llm_result:
                return llm_result

        # Fast semantic fallback
        return self._semantic_parse(prompt, rx, ry, ryaw, nearest_room_name, active_model, is_installed)

    def _call_ollama(self, prompt: str, model_name: str, rx: float, ry: float, ryaw: float, nearest_name: str) -> Optional[Dict[str, Any]]:
        rooms_summary = "\n".join([
            f"- {k}: {v['name']} at ({v['x']}, {v['y']}). {v['description']}"
            for k, v in HOUSE_ROOMS.items()
        ])

        yaw_deg = round(math.degrees(ryaw), 1)

        system_prompt = f"""You are the autonomous navigation agent for a TurtleBot3 robot inside a multi-room house.
The house contains the following rooms:
{rooms_summary}

Current Robot State:
Position: x={rx:.2f}, y={ry:.2f}, heading_yaw={yaw_deg:.1f}° ({ryaw:.2f} rad)
Nearest Room: {nearest_name}

User Prompt: "{prompt}"

Determine the user's intent and choose ONE of the following actions:

1. Single Room Navigation ("action": "navigate"):
   User wants to go to one room.
   Specify "target_room": "<room_id>" ('living_room', 'dining_room', 'kitchen', 'office', 'study', 'foyer').

2. Multi-Waypoint Navigation ("action": "navigate_waypoints"):
   User wants to visit multiple rooms or waypoints in sequence (e.g. "go to kitchen and then dining room", "visit living room, office, and foyer").
   Specify "waypoints": ["<room_id_1>", "<room_id_2>", ...] in visited order.

3. Relative Motion ("action": "relative_motion"):
   User wants to move or turn relative to current pose (e.g. "move forward 1m", "rotate 20 degrees", "go forward 1m and take left 1m", "turn right 90 degrees", "move backward 0.5m").
   Specify:
   - "forward": <meters to move forward (positive) or backward (negative), default 0.0>
   - "left": <meters to move left (positive) or right (negative), default 0.0>
   - "rotate_deg": <degrees to rotate: positive for counter-clockwise / left, negative for clockwise / right, default 0.0>

4. General Chat ("action": "chat"):
   User is asking a question, greeting, or not giving a navigation command.

Respond ONLY with a valid JSON object matching this schema:
{{
  "action": "navigate" | "navigate_waypoints" | "relative_motion" | "chat",
  "target_room": "<room_id or null>",
  "waypoints": ["<room_id>", ...] or null,
  "forward": 0.0,
  "left": 0.0,
  "rotate_deg": 0.0,
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
                    "num_predict": 200
                }
            }).encode('utf-8')

            req = urllib.request.Request(
                f"{self.ollama_url}/api/generate",
                data=req_data,
                headers={"Content-Type": "application/json"}
            )
            with urllib.request.urlopen(req, timeout=40.0) as resp:
                res_json = json.loads(resp.read().decode('utf-8'))
                raw_response = res_json.get('response', '')
                parsed = json.loads(raw_response)

                action = parsed.get('action')

                # Case 1: Multi-waypoint navigation
                if action == 'navigate_waypoints' or (parsed.get('waypoints') and isinstance(parsed.get('waypoints'), list)):
                    raw_wps = parsed.get('waypoints', [])
                    resolved_wps = []
                    names = []
                    for item in raw_wps:
                        r_id = str(item).strip().lower()
                        if r_id in HOUSE_ROOMS:
                            info = HOUSE_ROOMS[r_id]
                            resolved_wps.append({
                                'id': r_id,
                                'name': info['name'],
                                'x': info['x'],
                                'y': info['y'],
                                'yaw': info['yaw']
                            })
                            names.append(info['name'])
                    if resolved_wps:
                        return {
                            'success': True,
                            'action': 'navigate_waypoints',
                            'waypoints': resolved_wps,
                            'thought': parsed.get('thought', f"Sequential tour across {len(resolved_wps)} rooms: {' -> '.join(names)}"),
                            'reply': parsed.get('reply', f"Executing tour: {' ➔ '.join(names)}! Starting navigation now."),
                            'model': model_name
                        }

                # Case 2: Relative step motion
                if action == 'relative_motion':
                    return self._calculate_relative_motion(parsed, rx, ry, ryaw, model_name)

                # Case 3: Single room navigation
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

                # Case 4: General Chat
                return {
                    'success': True,
                    'action': 'chat',
                    'thought': parsed.get('thought', 'Conversational inquiry or status request.'),
                    'reply': parsed.get('reply', raw_response),
                    'model': model_name
                }
        except Exception:
            return None

    def _calculate_relative_motion(self, parsed: Dict[str, Any], rx: float, ry: float, ryaw: float, model_name: str) -> Dict[str, Any]:
        """Calculates precise target coordinates and waypoints from relative motion instructions."""
        forward = float(parsed.get('forward', 0.0))
        left = float(parsed.get('left', 0.0))
        rotate_deg = float(parsed.get('rotate_deg', 0.0))

        rot_rad = math.radians(rotate_deg)
        target_yaw = math.atan2(math.sin(ryaw + rot_rad), math.cos(ryaw + rot_rad))

        cos_yaw = math.cos(ryaw)
        sin_yaw = math.sin(ryaw)

        # If user specified both forward AND lateral step (e.g., "forward 1m and take left 1m")
        # generate two sequential waypoints for a natural step-and-turn path
        if abs(forward) > 0.05 and abs(left) > 0.05:
            # Waypoint 1: forward translation
            wp1_x = round(rx + forward * cos_yaw, 3)
            wp1_y = round(ry + forward * sin_yaw, 3)
            # Waypoint 2: lateral translation from wp1
            wp2_x = round(wp1_x - left * sin_yaw, 3)
            wp2_y = round(wp1_y + left * cos_yaw, 3)

            wps = [
                {'name': f'Step 1 ({forward:+.1f}m fwd)', 'x': wp1_x, 'y': wp1_y, 'yaw': ryaw},
                {'name': f'Step 2 ({left:+.1f}m lateral)', 'x': wp2_x, 'y': wp2_y, 'yaw': target_yaw}
            ]
            return {
                'success': True,
                'action': 'navigate_waypoints',
                'waypoints': wps,
                'forward': forward,
                'left': left,
                'rotate_deg': rotate_deg,
                'thought': parsed.get('thought', f"Moving forward {forward:.1f}m then taking left {left:.1f}m."),
                'reply': parsed.get('reply', f"Moving forward {forward:.1f}m, then proceeding {abs(left):.1f}m {'left' if left > 0 else 'right'}!"),
                'model': model_name
            }

        # Single compound translation
        dx = forward * cos_yaw - left * sin_yaw
        dy = forward * sin_yaw + left * cos_yaw
        target_x = round(rx + dx, 3)
        target_y = round(ry + dy, 3)

        desc_parts = []
        if abs(forward) > 0.05:
            desc_parts.append(f"{'forward' if forward > 0 else 'backward'} {abs(forward):.1f}m")
        if abs(left) > 0.05:
            desc_parts.append(f"{'left' if left > 0 else 'right'} {abs(left):.1f}m")
        if abs(rotate_deg) > 1.0:
            desc_parts.append(f"rotate {abs(rotate_deg):.0f}° {'left (CCW)' if rotate_deg > 0 else 'right (CW)'}")

        desc_str = ", ".join(desc_parts) if desc_parts else "relative adjustment"

        return {
            'success': True,
            'action': 'navigate',
            'room_name': f'Relative: {desc_str}',
            'x': target_x,
            'y': target_y,
            'yaw': target_yaw,
            'forward': forward,
            'left': left,
            'rotate_deg': rotate_deg,
            'thought': parsed.get('thought', f"Executing relative motion: {desc_str}."),
            'reply': parsed.get('reply', f"Executing step: {desc_str} to ({target_x:.2f}, {target_y:.2f})."),
            'model': model_name
        }

    def _semantic_parse(self, prompt: str, rx: float, ry: float, ryaw: float, nearest_name: str, model_name: str, is_installed: bool) -> Dict[str, Any]:
        """Fast rule-based regex engine for multi-waypoints and relative motion when model is offline."""
        p_lower = prompt.lower().strip()
        model_status_note = "" if is_installed else f" (Local {model_name} is loading)"

        # --- A. Check for Relative Motion Patterns ---

        # 1. Forward and Left/Right combined (e.g. "move forward 1m and take left 1m")
        fwd_lat_match = re.search(
            r'(?:move|go|drive|step)?\s*forward\s*(?:by)?\s*([\d.]+)\s*(?:m|meter|meters)?\s*(?:and)?\s*(?:take|turn|go|move)?\s*(left|right)\s*(?:by)?\s*([\d.]+)\s*(?:m|meter|meters)?',
            p_lower
        )
        if fwd_lat_match:
            fwd_val = float(fwd_lat_match.group(1))
            lat_dir = fwd_lat_match.group(2)
            lat_val = float(fwd_lat_match.group(3))
            left_val = lat_val if lat_dir == 'left' else -lat_val
            parsed_rel = {'forward': fwd_val, 'left': left_val, 'rotate_deg': 0.0}
            return self._calculate_relative_motion(parsed_rel, rx, ry, ryaw, f"{model_name} (Fast Semantic Engine)")

        # 2. Pure forward/backward motion (e.g. "move forward 1 meter", "go back 0.5m", "move the robot by 1 meter")
        fwd_match = re.search(
            r'(?:move|go|drive|step)\s+(?:the\s+robot\s+)?(?:by\s+)?(?:forward|ahead|straight)?\s*([\d.]+)\s*(?:m|meter|meters)?',
            p_lower
        )
        back_match = re.search(
            r'(?:move|go|drive|step)\s+(?:the\s+robot\s+)?(?:backward|back|reverse)\s*(?:by\s+)?([\d.]+)\s*(?:m|meter|meters)?',
            p_lower
        )
        if fwd_match and not any(w in p_lower for w in ['rotate', 'turn', 'spin', 'degree']):
            val = float(fwd_match.group(1))
            parsed_rel = {'forward': val, 'left': 0.0, 'rotate_deg': 0.0}
            return self._calculate_relative_motion(parsed_rel, rx, ry, ryaw, f"{model_name} (Fast Semantic Engine)")
        if back_match:
            val = float(back_match.group(1))
            parsed_rel = {'forward': -val, 'left': 0.0, 'rotate_deg': 0.0}
            return self._calculate_relative_motion(parsed_rel, rx, ry, ryaw, f"{model_name} (Fast Semantic Engine)")

        # 3. Pure rotation (e.g. "rotate the robot by 20 degrees", "turn left 45 deg", "rotate 90 degrees")
        rot_match = re.search(
            r'(?:rotate|turn|spin)\s*(?:the\s+robot\s+)?(?:(left|right|counter-clockwise|clockwise)\s*)?(?:by\s*)?([\d.]+)\s*(?:deg|degree|degrees)?',
            p_lower
        )
        if rot_match:
            dir_str = rot_match.group(1) or 'left'
            deg_val = float(rot_match.group(2))
            if dir_str in ['right', 'clockwise']:
                deg_val = -deg_val
            parsed_rel = {'forward': 0.0, 'left': 0.0, 'rotate_deg': deg_val}
            return self._calculate_relative_motion(parsed_rel, rx, ry, ryaw, f"{model_name} (Fast Semantic Engine)")

        # --- B. Check for Multi-Room Waypoint Sequence ---
        # Look for sequences linked by 'then', 'and then', 'after', ','
        room_matches_in_order = []
        tokens = re.split(r'\b(?:then|and then|after that|afterward|,|->)\b', p_lower)
        if len(tokens) >= 2:
            for token in tokens:
                for r_id, r_info in HOUSE_ROOMS.items():
                    if any(re.search(r'\b' + re.escape(kw) + r'\b', token) for kw in r_info['keywords']):
                        if r_id not in [item['id'] for item in room_matches_in_order]:
                            room_matches_in_order.append({
                                'id': r_id,
                                'name': r_info['name'],
                                'x': r_info['x'],
                                'y': r_info['y'],
                                'yaw': r_info['yaw']
                            })
                            break
            if len(room_matches_in_order) >= 2:
                names = [it['name'] for it in room_matches_in_order]
                return {
                    'success': True,
                    'action': 'navigate_waypoints',
                    'waypoints': room_matches_in_order,
                    'thought': f"Multi-waypoint sequence detected: {' ➔ '.join(names)}",
                    'reply': f"Understood! Guiding the robot through: {' ➔ '.join(names)}!{model_status_note}",
                    'model': f"{model_name} (Fast Semantic Engine)"
                }

        # --- C. Single Room Navigation Match ---
        matched_room_id = None
        best_match_score = 0
        for room_id, r in HOUSE_ROOMS.items():
            for kw in r['keywords']:
                if re.search(r'\b' + re.escape(kw) + r'\b', p_lower):
                    score = len(kw)
                    if score > best_match_score:
                        best_match_score = score
                        matched_room_id = room_id

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

        # --- D. General Queries & Greetings ---
        if any(w in p_lower for w in ['where', 'location', 'position']):
            return {
                'success': True,
                'action': 'chat',
                'thought': "User asked for robot location.",
                'reply': f"I am currently near the {nearest_name} at coordinates ({rx:.2f}, {ry:.2f}).{model_status_note}",
                'model': f"{model_name} (Fast Semantic Engine)"
            }

        if any(w in p_lower for w in ['hello', 'hi', 'hey']):
            return {
                'success': True,
                'action': 'chat',
                'thought': "User greeting.",
                'reply': f"Hello! I am your TurtleBot3 assistant. You can tell me:\n• 'Go to living room and then kitchen'\n• 'Move forward 1 meter'\n• 'Rotate 20 degrees'\n• 'Move forward 1m and take left 1m'{model_status_note}",
                'model': f"{model_name} (Fast Semantic Engine)"
            }

        return {
            'success': True,
            'action': 'chat',
            'thought': "Could not identify navigation instruction or room.",
            'reply': f"I didn't recognize that destination or movement command. Try:\n• 'Go to Kitchen and then Dining Room'\n• 'Move forward by 1 meter'\n• 'Rotate 20 degrees'\n• 'Move forward 1m and take left 1m'{model_status_note}",
            'model': f"{model_name} (Fast Semantic Engine)"
        }
