import json

# Full label set used by --combined_score; labels missing from gaze/gesture get MISSING_LABEL_SCORE.
COMBINED_LABELS = [
    "banana", "detergent bottle", "juice", "ketchup bottle",
    "pepper bottle", "salt bottle", "sponge", "tomato",
]
MISSING_LABEL_SCORE = 0.0001


def _parse_label_scores(info_str):
    """Parse gesture/gaze info JSON into {canonical_label: score}.

    Handles both flat items {"label": "x", "score": s} and the nested gaze
    format {"label": {"label": "x", "score": s}}.
    """
    if not info_str or info_str.strip() in ("", "[]", "None"):
        return {}
    try:
        items = json.loads(info_str)
    except (json.JSONDecodeError, TypeError):
        return {}
    scores = {}
    for item in items:
        if isinstance(item, dict) and isinstance(item.get("label"), dict):
            item = item["label"]
        if not isinstance(item, dict) or "score" not in item:
            continue
        name = str(item.get("label", "")).strip().lower()
        # Map e.g. "ketchup" / "ketchup bottle" to the canonical label by its first word.
        for canon in COMBINED_LABELS:
            if name == canon or name.split(" ")[0] == canon.split(" ")[0]:
                scores[canon] = float(item["score"])
                break
    return scores


def combine_label_scores(gesture_str, gaze_str):
    """Multiply gesture and gaze scores per label, then normalize by the sum.

    Returns a JSON string sorted by score, or "None" when neither input has scores.
    """
    gesture_scores = _parse_label_scores(gesture_str)
    gaze_scores = _parse_label_scores(gaze_str)
    if not gesture_scores and not gaze_scores:
        return "None"
    products = {
        label: gesture_scores.get(label, MISSING_LABEL_SCORE) * gaze_scores.get(label, MISSING_LABEL_SCORE)
        for label in COMBINED_LABELS
    }
    total = sum(products.values())
    info = [{"label": label, "score": score / total} for label, score in products.items()]
    info.sort(key=lambda item: item["score"], reverse=True)
    return json.dumps(info, ensure_ascii=False)
