# Mosaic: LLM & VLM Integration with Whisper Transcription

## 🏗️ Structure

```shell
.
├── 1.main.py                    # main entry
├── 2.intention_predict.py       # real-time video stream: gesture + head motion detection + YOLO + speech interaction
├── 3.speech_detection.py        # speech detection
├── action_test.py               # action test
├── gesture.py                   # gesture related
├── paper.txt                    # paper/document
├── README.md
├── requirements.txt             # dependencies
├── run.sh                       # startup script
├── service_test_with_code.py    # service test
├── yolov8n.pt                   # YOLO model
├── yolo_model/                  # YOLO model folder
│   └── best.pt
├── images/                      # images and depth data
│   ├── depth.npy
│   ├── depth.png
│   ├── example1.jpg
│   ├── example2.jpg
│   ├── example3.jpg
│   ├── example4.jpg
│   ├── result_l.jpg
│   ├── result_r.jpg
│   └── ...
└── src/
    ├── utils.py
    ├── execute/
    │   ├── actions.py
    │   └── move_service_example.py
    ├── grasp/
    │   ├── bounding_box.py
    │   ├── grasp_execution.py
    │   ├── grasp_generation.py
    │   ├── mesh.py
    ├── intention/
    │   └── intention_predict.py
    ├── mistral_ai/
    │   ├── llm.py
    │   ├── mistral.py
    │   ├── vlm.py
    │   ├── prompts/
    │   │   ├── intention_prompt.py
    │   │   ├── plan_prompt.py
    │   │   └── vision_prompt.py
    │   └── scripts/
    │       ├── llm_script.json
    │       ├── llm_script.txt
    │       ├── vlm_script.json
    │       └── vlm_script.txt
    ├── pixel_world/
    │   └── pixel_and_world.py
    ├── transcribe/
    │   ├── stt.py
    │   ├── tts.py
    │   ├── speech.txt
    │   └── transcription.txt
    └── VLM_agent/
        ├── agent.py
        ├── OwlViT_FastSAM_SAM.py
        ├── FastSAM/
        │   ├── FastSAM-x.pt
        └── sam_hq/
            └── sam_vit_h_4b8939.pth
```
```

## 🔧 Installation guide


1. **Install system dependencies(for Ubuntu 22.04)**  
   ```bash
   sudo apt update
   sudo apt install portaudio19-dev python3-dev pulseaudio pulseaudio-utils
   ```

2. **Install Python dependencies**  
   ```bash
   conda create -n mosaic python=3.11
   conda activate mosaic
   pip install -r requirements.txt
   ```

3. **Set environment variables**  
   Add the following lines to `~/.bashrc`, `~/.zshrc` or `~/.config/fish/config.fish`:  
   ```bash
   export MISTRAL_API_KEY="your_mistral_api_key_here"
   ```

4. **Model source**
   ```
   src/VLM_agent/sam_hq/sam_vit_h_4b8939.pth: 
   https://huggingface.co/HCMUE-Research/SAM-vit-h/blob/main/sam_vit_h_4b8939.pth

   src/VLM_agent/FastSAM/FastSAM-x.pt:
   https://github.com/ultralytics/assets/releases/download/v8.3.0/FastSAM-x.pt
   ```

## 🎯 Run

1. **Main script**  
   ```bash
   python3 1.main.py
   ```

2. **test script**
   ```bash
   python3 action_test.py # You can test the excuation of the action sequence part with it.
   ```