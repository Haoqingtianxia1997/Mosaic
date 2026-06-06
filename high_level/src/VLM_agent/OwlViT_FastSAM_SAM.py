# Support text-driven object detection and segmentation using OwlViT + FastSAM/SAM
import re, difflib,functools
import torch, numpy as np, cv2
from PIL import Image, ImageDraw, ImageFont
from transformers import OwlViTProcessor, OwlViTForObjectDetection
from ultralytics import YOLO
import easyocr
from transformers import MarianMTModel, MarianTokenizer
import torch, gc
import numpy as np
import cv2
from PIL import Image, ImageDraw
from segment_anything import sam_model_registry, SamPredictor

# yolo_model = YOLO('yolo_model/yolo11m.pt')  # Or use your custom model path
yolo_model = YOLO('yolo_model/mixed_zed_realsense.pt')

# Pseudo color table (customizable)
COLOR_TABLE = [
    (255, 0, 0),    # Red
    (0, 255, 0),    # Green
    (0, 0, 255),    # Blue
    (255, 255, 0),  # Yellow
    (255, 0, 255),  # Magenta
    (0, 255, 255),  # Cyan
]

# ---------------- Auxiliary Functions ----------------
def apply_mask_color(mask, color, alpha=120):
    """mask: [H, W] np.uint8 0-255, color: (R,G,B), alpha: 0-255"""
    mask_img = Image.fromarray(mask)
    color_img = Image.new("RGBA", mask_img.size, color + (0,))
    # Only add transparency to the foreground
    mask_rgba = mask_img.convert("L").point(lambda x: alpha if x > 0 else 0)
    color_img.putalpha(mask_rgba)
    return color_img

def prompt_tokens(prompt: str):
    """Split the prompt into a list of English words with length ≥ 3, all lowercase"""
    return [w for w in re.findall(r"[a-zA-Z']+", prompt.lower()) if len(w) >= 3]

def fuzzy_ratio(a: str, b: str) -> float:
    return difflib.SequenceMatcher(None, a, b).ratio()

def prompt_match(tokens, ocr_tokens, fuzzy_th=0.8):
    """
    All tokens must match the OCR completely to return True:
    ① If kw is a substring of some OCR t → Match successful
    ② Or fuzzy similarity ≥ fuzzy_th → Match successful
    """
    matched_scores = []

    for kw in tokens:
        kw = kw.lower()
        matched = False
        best_score = 0.0

        for t in ocr_tokens:
            t = t.lower()
            if kw in t:
                matched = True
                best_score = 1.0
                break  # Direct hit, exit inner loop
            score = fuzzy_ratio(kw, t)
            if score >= fuzzy_th:
                matched = True
                best_score = max(best_score, score)

        if not matched:
            return False, 0  # As long as one keyword does not match → All fail
        matched_scores.append(best_score)

    # All matched, return the minimum score (the weakest link)
    return True, min(matched_scores)

# --------------------

class SAMSegmenter:
    def __init__(self, sam_ckpt="src/VLM_agent/sam_hq/sam_vit_h_4b8939.pth", device=None):
        self.device ="cpu" # device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.sam = sam_model_registry["vit_h"](checkpoint=sam_ckpt).to(self.device)
        self.sam_predictor = SamPredictor(self.sam)

    def segment_with_boxes(self, image_path, boxes_xyxy, multimask_output=False):
        """
        image_path      : str
        boxes_xyxy      : (x0,y0,x1,y1)  or  [(...), (...)]   pixel coordinates
        returns         : numpy.bool_  [N,H,W]  (multi=False)  or  [N,3,H,W]
        """
        img_bgr = cv2.imread(image_path)
        if img_bgr is None:
            raise FileNotFoundError(image_path)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        H, W = img_rgb.shape[:2]

        # --- Standardize boxes to [N,4] ---
        if isinstance(boxes_xyxy[0], (int, float)):
            boxes_arr = np.asarray([boxes_xyxy], dtype=np.float32)
        else:
            boxes_arr = np.asarray(boxes_xyxy, dtype=np.float32)

        self.sam_predictor.set_image(img_rgb)
        inp_boxes = torch.as_tensor(boxes_arr, device=self.device)           # [N,4]
        tr_boxes  = self.sam_predictor.transform.apply_boxes_torch(
                        inp_boxes, (H, W))

        masks, _, _ = self.sam_predictor.predict_torch(
            point_coords=None,
            point_labels=None,
            boxes=tr_boxes,
            multimask_output=multimask_output            # => [N,M,H,W]
        )

        masks = masks.bool().cpu().numpy()               # → numpy
        if not multimask_output:                         # [N,1,H,W] → [N,H,W]
            masks = masks[:, 0]

        return masks                   

class TextDrivenSegmenter:
    def __init__(self, fastsam_model_path='src/VLM_agent/FastSAM/FastSAM-x.pt', use_gpu=True):
        self.owl_processor = OwlViTProcessor.from_pretrained("google/owlvit-large-patch14")
        self.owl_model     = OwlViTForObjectDetection.from_pretrained("google/owlvit-large-patch14")
        
        self.fastsam       = YOLO(fastsam_model_path)
        self.sam = SAMSegmenter(sam_ckpt="src/VLM_agent/sam_hq/sam_vit_h_4b8939.pth")
        
        self.reader        = easyocr.Reader(['de'], gpu=use_gpu)
        self.device        = "cuda" if (use_gpu and torch.cuda.is_available()) else "cpu"
        self.owl_model.to(self.device)
        if self.device == "cuda":
            self.fastsam.to(self.device)
        self.fastsam_conf, self.fastsam_iou, self.fastsam_size = 0.4, 0.9, 1024
        self.color_map = {}
        trans_name = "Helsinki-NLP/opus-mt-en-de"
        self.trans_tok = MarianTokenizer.from_pretrained(trans_name)
        self.trans_mod = MarianMTModel.from_pretrained(trans_name)
        self.trans_mod.to(self.device)      # in the same device
        self.mask_labels = {"beer bottle", "mayonnaise bottle", "oil bottle", "water bottle"}
    
    @functools.lru_cache(maxsize=512)
    @torch.inference_mode()
    def _en_word2de(self, word: str) -> str:
        batch = self.trans_tok(word, return_tensors="pt").to(self.device)
        gen   = self.trans_mod.generate(**batch, max_length=16)
        return self.trans_tok.decode(gen[0], skip_special_tokens=True)

    # ---------------- Main Process ----------------
    def detect_and_segment(self, image_path, text_prompts, text_label, multi_task=False, if_sam=True, if_translate=False, method="yolo", bbox_only=True, name=None):
        image = Image.open(image_path).convert("RGB")
        if name == "realsense":
            img_arr = np.array(image)
            h, w = img_arr.shape[:2]
            ys = np.arange(h, dtype=np.float32)[:, None]
            xs = np.arange(w, dtype=np.float32)[None, :]
            img_arr[ys > 1.4 * xs + 80.0] = 0
            image = Image.fromarray(img_arr)
            
        W, H  = image.size
        draw_img   = image.copy()
        all_boxes, all_masks, all_points = [], [], []
        not_match = False

        for prompt, label in zip (text_prompts, text_label):
           
            if label == "":
                not_match = True
            else:
                if if_translate:  # Translate to German
                    tokens_en = prompt_tokens(label)
                    tokens_de = [self._en_word2de(w).lower() for w in tokens_en]
                    p_tokens = tokens_de         # Full prompt → word list
                    print(f'Prompt EN: "{label}"   →   DE: "{tokens_de}"')
                else:
                    p_tokens = prompt_tokens(label)
                    print(f'Prompt: "{label}"   →   Tokens: {p_tokens}')
                

            if method == "owlvit":
                inputs = self.owl_processor(text=[prompt], images=image,
                                            return_tensors="pt").to(self.device)
                with torch.no_grad():
                    out = self.owl_model(**inputs)
                boxes  = out.pred_boxes[0].cpu()
                scores = out.logits[0].cpu().sigmoid()[:, 0]
                keep   = torch.topk(scores, k=min(5, len(scores))).indices  # Top 5 candidates

            elif method == "yolo":
                YOLO_MASKED_CLASSES = self.mask_labels 

                yolo_results = yolo_model(image, conf=0.7)
                boxes_xyxy = yolo_results[0].boxes.xyxy.cpu().numpy()
                confs      = yolo_results[0].boxes.conf.cpu().numpy()
                labels_idx = yolo_results[0].boxes.cls.cpu().numpy()
                print(f"YOLO detected {labels_idx}")
                class_names = yolo_model.names

                for i, c in enumerate(labels_idx):
                    name = class_names[int(c)]
                    conf = confs[i]
                    bbox = boxes_xyxy[i]
                    print(f"  - Class: {name:>10s}, Confidence: {conf:.3f}, BBox: [{bbox[0]:.1f}, {bbox[1]:.1f}, {bbox[2]:.1f}, {bbox[3]:.1f}]")

                # Remove masked classes and keep only the top-scoring detection per label
                valid_mask = [
                    i for i, c in enumerate(labels_idx)
                    if class_names[int(c)].lower() not in YOLO_MASKED_CLASSES
                ]
                best_per_label = {}
                for i in valid_mask:
                    name = class_names[int(labels_idx[i])].lower()
                    if name not in best_per_label or confs[i] > confs[best_per_label[name]]:
                        best_per_label[name] = i
                kept_indices = list(best_per_label.values())
                boxes_xyxy  = boxes_xyxy[kept_indices]
                confs       = confs[kept_indices]
                labels_idx  = labels_idx[kept_indices]

                # Filter categories based on prompt
                prompt_lower = prompt.lower()
                matched_idx = [
                    i for i, c in enumerate(labels_idx)
                    if class_names[int(c)].lower() == prompt_lower
                ]
                if not matched_idx:
                    print(f"Cannot find target '{prompt}' in the detected objects. Skipping...")
                    return None, None, None

                filtered_boxes = boxes_xyxy[matched_idx]
                filtered_confs = confs[matched_idx]

                top_idx = filtered_confs.argsort()[::-1][:2]
                boxes_list = []
                for b in filtered_boxes[top_idx]:
                    x1, y1, x2, y2 = b
                    # Ensure x1 < x2 and y1 < y2
                    x1, x2 = sorted([x1, x2])
                    y1, y2 = sorted([y1, y2])
                    # Convert to cx, cy, bw, bh (normalized to 0~1)
                    cx = (x1 + x2) / 2 / W
                    cy = (y1 + y2) / 2 / H
                    bw = (x2 - x1) / W
                    bh = (y2 - y1) / H
                    boxes_list.append(np.array([cx, cy, bw, bh], dtype=np.float32))
                boxes = [torch.from_numpy(b) for b in boxes_list]
                scores = filtered_confs[top_idx]
                keep = range(len(boxes))


            matched = []   # OCR matched candidates
            for idx in keep:
                if scores[idx] < 0.1:    # Score lower limit
                    continue
                if not_match:            # No matching task, directly use the highest score box
                    box = self._box_xyxy(boxes[idx], W, H)
                    matched.append((box, scores[idx].item()))
                else:
                    box  = self._box_xyxy(boxes[idx], W, H)
                    x1,y1,x2,y2 = box
                    pad = 20                  # Expand 20 px to avoid cutting off text
                    crop = image.crop((max(0,x1-pad), max(0,y1-pad),
                                    min(W,x2+pad), min(H,y2+pad)))
                    ocr_tokens = [t.lower() for t in self.reader.readtext(np.array(crop),
                                                                        detail=0)]
                    exist , matched_score = prompt_match(p_tokens, ocr_tokens)
                    if exist:   # Hit once is enough
                        total_score = matched_score + scores[idx].item()  # Hit score + detection score
                        print(matched_score, scores[idx].item(), total_score)
                        matched.append((box, total_score))
                
            
            if len(matched)>1 and not multi_task:
                # In multi-task mode, allow multiple matches; otherwise, keep only the highest-scoring one
                matched = sorted(matched, key=lambda x: x[1], reverse=True)[:1]

            # If there are hits, use them; otherwise, fallback to the highest confidence box
            cand = matched if matched else [(self._box_xyxy(boxes[keep[0]], W, H),
                                             scores[keep[0]].item())]

            for box, conf in cand:

                if bbox_only:
                    mask = self._box_mask(box, W, H)
                elif if_sam:
                    masks_sam = self.sam.segment_with_boxes(image_path, [box],
                                                            multimask_output=False)
                    mask = masks_sam[0].astype(np.uint8)             # Choose single mask in the first box
                else:
                    mask = self._fastsam_seg(image_path, box)

                bcen, mcen = self._centers(box, mask)
                all_seg_points = self._all_seg_points(mask)  # Calculate all segmentation points
                
                all_boxes.append((box, prompt, conf))
                all_masks.append(mask)
                all_points.append({"target": prompt,
                                   "box_center_point": bcen,
                                   "seg_center_point": mcen,
                                   "seg_points": all_seg_points})

        result = self._visual(draw_img, all_boxes, all_masks)
        return result, all_boxes, all_points


    # ---------- Tools ----------
    @staticmethod
    def _box_xyxy(box, W, H):
        cx, cy, bw, bh = box.numpy()
        return (int(max(0,(cx-bw/2)*W)), int(max(0,(cy-bh/2)*H)),
                int(min(W,(cx+bw/2)*W)), int(min(H,(cy+bh/2)*H)))

    def _centers(self, box, mask):
        x1,y1,x2,y2 = box
        cx, cy = (x1+x2)//2, (y1+y2)//2
        ys, xs = np.where(mask>0)
        return [cx,cy], [None,None] if len(xs)==0 else [int(xs.mean()),int(ys.mean())]

    def _all_seg_points(self, mask):
        ys, xs = np.where(mask > 0)
        if len(xs) == 0:
            return []
        return np.array([(int(x), int(y)) for x, y in zip(xs, ys)], dtype=int)

    @staticmethod
    def _box_mask(box, W, H):
        """Create a rectangular mask from the bounding box, skipping SAM/FastSAM."""
        x1, y1, x2, y2 = box
        mask = np.zeros((H, W), dtype=np.uint8)
        mask[y1:y2, x1:x2] = 1
        return mask

    def _fastsam_seg(self, img_path, box):
        image = Image.open(img_path).convert("RGB")
        W,H   = image.size
        x1,y1,x2,y2 = map(int, box)
        crop  = np.array(image.crop((x1,y1,x2,y2)))
        res   = self.fastsam.predict(crop, conf=self.fastsam_conf, iou=self.fastsam_iou,
                                     imgsz=self.fastsam_size, device=self.device, verbose=False)
        best,marea = None,-1
        for r in res:
            if r.masks is None: continue
            for m in r.masks.data:
                arr = m.cpu().numpy().astype(np.uint8)
                area = arr.sum()
                if area>marea: best,marea = arr,area
        final = np.zeros((H,W),dtype=np.uint8)
        if best is not None:
            if best.shape!=(y2-y1,x2-x1):
                best = cv2.resize(best,(x2-x1,y2-y1),interpolation=cv2.INTER_NEAREST)
            final[y1:y2,x1:x2] = best
        return final

    # ---------- Visualization ----------
    def _visual(self, img, boxes, masks):
        draw = ImageDraw.Draw(img)
        overlay = Image.new("RGBA", img.size, (0,0,0,0))
        for (box,label,score),mask in zip(boxes,masks):
            if label not in self.color_map:
                self.color_map[label] = tuple(np.random.randint(64,256,size=3))
            col = self.color_map[label]
            draw.rectangle(box, outline=col, width=3)
            txt = f"{label} ({score:.2f})"
            try: font = ImageFont.truetype("arial.ttf", 20)
            except: font = ImageFont.load_default()
            draw.text((box[0], box[1]-25), txt, fill=col, font=font)
            if mask.any():
                rgba = np.zeros((*mask.shape,4),dtype=np.uint8)
                rgba[mask>0] = (*col,200)
                overlay = Image.alpha_composite(overlay, Image.fromarray(rgba))
        return Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB")

_SEG_INSTANCE = None

def get_segmenter():
    global _SEG_INSTANCE
    if _SEG_INSTANCE is None:
        _SEG_INSTANCE = TextDrivenSegmenter(fastsam_model_path="src/VLM_agent/FastSAM/FastSAM-x.pt")
    return _SEG_INSTANCE


    
def find_object_central_pixel(target: str, text: str, image_path, is_sam: bool = True, if_translate: bool = False, name: str = "left", segmenter=None, bbox_only: bool = True):
    if segmenter is None:
        seg = get_segmenter() #TextDrivenSegmenter(fastsam_model_path="src/VLM_agent/FastSAM/FastSAM-x.pt")
    else:
        seg = segmenter # Allow passing a segmenter instance to avoid repeated loading in multi-turn interactions
    img, boxes, points = seg.detect_and_segment(image_path, [target], [text],  multi_task = False, if_sam = is_sam, if_translate = if_translate, bbox_only = bbox_only, name= name)
    if img is None or boxes is None or points is None:
        print(f"Cannot find target '{target}' in the image.")
        return None, None, None, None, None, None
    
    if name == "left":
        img.save("images/result_l.jpg")
    elif name == "right":
        img.save("images/result_r.jpg")
    elif name == "realsense":
        img_arr = np.array(img)
        h, w = img_arr.shape[:2]
        ys = np.arange(h, dtype=np.float32)[:, None]
        xs = np.arange(w, dtype=np.float32)[None, :]
        img_arr[ys > 1.4 * xs + 80.0] = 0
        img = Image.fromarray(img_arr)
        img.save("images/realsense.jpg")

    for (b,l,s),pt in zip(boxes, points):
        print(f"{l} @ {b}  conf={s:.2f}")
        print("   centers:", pt)  

    target_prompt = points[0]["target"]
    box_center_point = points[0]["box_center_point"]
    seg_center_point = points[0]["seg_center_point"]
    all_seg_points = points[0]["seg_points"]

    bbox = tuple(boxes[0][0])  # Get the bounding box of the first target
    score = boxes[0][2]  # Get the confidence score of the first target

    # del seg, img, boxes, points
    if torch.cuda.is_available(): # clear CUDA cache
        torch.cuda.empty_cache()
    gc.collect() # 3 Optional, but recommended

    return target_prompt, box_center_point, seg_center_point, all_seg_points, bbox, score


# ---------------- demo ----------------
if __name__ == "__main__":
    target_label = "pepper bottle"  # Replace with the target you want to detect
    text = "black pepper"  # Replace with the text you want to detect
    image_path = "images/example1.jpg"  # Replace with your image path
    # text = ""
    target_prompt, box_center_point, seg_center_point, all_seg_points, bbox, score = find_object_central_pixel(target_label, text, image_path, is_sam=True, if_translate=True, bbox_only=True)  # Call the function to process object detection in the image
    print(f"🔍 Detected target: {target_label}")
    print(f"📍 Target prompt: {target_prompt}")
    print(f"📏 Bounding box: {bbox}")
    print(f"🎯 Box center point: {box_center_point}")
    print(f"🎯 Segmentation center point: {seg_center_point}")
    print(f"📊 Detection score: {score}")
