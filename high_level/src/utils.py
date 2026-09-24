import json
import re
from typing import Any, Tuple, List, Dict

def get_last_text_line(filepath):
    """
    read the last non-empty line from a text file, ignoring comments and timestamps
    """
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = [line.strip() for line in f if line.strip()]
    # Reverse search for the last non-empty line that is not a comment
    for line in reversed(lines):
        if ']' in line:
            # Remove timestamp part
            text = line.split(']', 1)[-1].strip()
            if text:
                return text
        elif line and not line.startswith('//'):
            return line
    return ""

def get_full_text(filepath):
    """
    Read all valid non-empty lines from a text file, ignoring comments and timestamps,
    and return the combined full text.
    """
    lines = []
    with open(filepath, 'r', encoding='utf-8') as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith('//'):
                continue
            if ']' in stripped:
                # Remove timestamp like [YYYY-MM-DD HH:MM:SS]
                text = stripped.split(']', 1)[-1].strip()
            else:
                text = stripped
            if text:
                lines.append(text)
    return ' '.join(lines)

def safe_extract_json_and_response_for_llm(text: str) -> tuple[str, list[dict]]:
    """
    Extract JSON block from LLM output (which may contain mixed text and JSON).
    Returns: response, json_blocks
    """
    try:
        if not text or not text.strip():
            print(f"[safe_extract_json_and_response] Empty input text received")
            return "", []

        # Extract JSON blocks from markdown code fences
        json_pattern = r'```(?:json)?\s*([\s\S]*?)```'
        json_matches = re.findall(json_pattern, text)

        if json_matches:
            # Try to parse the first JSON block found
            json_text = json_matches[0].strip()
        else:
            # Fallback: try to parse the entire text as JSON
            json_text = text.strip()

        parsed = json.loads(json_text)
        response = parsed.get("response", "")
        actions = parsed.get("actions", [])
        return response, [parsed]  # Keep extract_json interface compatible

    except json.JSONDecodeError as e:
        print(f"[safe_extract_json_and_response] JSON parsing failed: {e}")
        print(f"[safe_extract_json_and_response] Raw LLM output (first 500 chars): {text[:500]}")
        return "", []
    except Exception as e:
        print(f"[safe_extract_json_and_response] Unexpected error: {e}")
        print(f"[safe_extract_json_and_response] Raw LLM output (first 500 chars): {text[:500]}")
        return "", []

def safe_extract_json_and_response_for_intention_llm(text: str) -> tuple[str, list[dict]]:
    """
    Extract JSON block from LLM output (which may contain mixed text and JSON).
    Returns: response, audio_response, content, json_blocks
    """
    try:
        if not text or not text.strip():
            print(f"[safe_extract_json_and_response] Empty input text received")
            return "", "", [], []

        # Extract JSON blocks from markdown code fences
        json_pattern = r'```(?:json)?\s*([\s\S]*?)```'
        json_matches = re.findall(json_pattern, text)

        if json_matches:
            # Try to parse the first JSON block found
            json_text = json_matches[0].strip()
        else:
            # Fallback: try to parse the entire text as JSON
            json_text = text.strip()

        parsed = json.loads(json_text)
        response = parsed.get("response", "")
        audio_response = parsed.get("audio response", "")
        content = parsed.get("content", [])
        return response, audio_response, content, [parsed]

    except json.JSONDecodeError as e:
        print(f"[safe_extract_json_and_response] JSON parsing failed: {e}")
        print(f"[safe_extract_json_and_response] Raw LLM output (first 500 chars): {text[:500]}")
        return "", "", [], []
    except Exception as e:
        print(f"[safe_extract_json_and_response] Unexpected error: {e}")
        print(f"[safe_extract_json_and_response] Raw LLM output (first 500 chars): {text[:500]}")
        return "", "", [], []

def safe_extract_json_and_response_for_vlm(data: Any) -> Tuple[bool, str, Dict]:
    """
    Return three values:
      1. found     : bool
      2. response  : str
      3. full_json : dict  ←  Complete JSON
    """
    try:
        # ── 1. convert to dict ─────────────────────────────
        if isinstance(data, dict):
            parsed = data
        else:
            text = re.sub(r"```json|```", "", str(data)).strip()
            text = re.sub(r"\bFalse\b", "false", text)
            text = re.sub(r"\bTrue\b",  "true",  text)
            parsed = json.loads(text)

        # ── 2. Extract keywords ───────────────────────────────
        found = bool(parsed.get("if_find", False))

        raw_resp = parsed.get("response", "")
        response = " ".join(raw_resp) if isinstance(raw_resp, list) else str(raw_resp)

        return found, response, parsed        # ← Directly return complete dict

    except Exception as e:
        print(f"[safe_extract_json_and_response] JSON parsing failed: {e}")
        return False, "", {}
