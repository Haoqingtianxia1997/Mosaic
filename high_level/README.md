# Mosaic: LLM & VLM Integration with Whisper Transcription

## 🏗️ Structure

```shell
.
├── 1.main.py                    # actual main at the moment
├── 2.intention_predict.py       # real time video feed: gesture + head motion detection(moving region of interest along the head direction) + YOLO + speech interaction
├── 3.speech_detection.py        
├── gesture.py                   
├── main.py                      
├── run.sh                       # bash script to avoid warning
├── requirements.txt             # dependencies
├── README.md
├── yolo_model/                  # YOLO models
│   ├── yolo11m.pt
│   ├── yolo11n.pt
│   ├── yolo12x.pt
│   └── yolov8x-oiv7.pt
├── images/                      # rgb images and depth maps
│   ├── depth.npy
│   ├── depth.png
│   ├── rgb.png
│   ├── example1.jpg
│   └── ...
└── src/
    ├── utils.py
    ├── execute/                 
    │   └── actions.py
    ├── intention/               
    │   └── intention_predict.py
    ├── mistral_ai/
    │   ├── llm.py               
    │   ├── vlm.py               
    │   ├── mistral.py           # Mistral API
    │   ├── prompts/
    │   │   ├── plan_prompt.py
    │   │   └── vision_prompt.py
    │   └── scripts/
    │       ├── llm_script.txt
    │       ├── llm_script.json
    │       ├── vlm_script.txt
    │       └── vlm_script.json
    ├── pixel_world/
    │   └── pixel_world.py       # image coordinate and world coordinate conversion
    ├── transcribe/
    │   ├── stt.py               # speech to text
    │   ├── tts.py               # text to speech
    │   └── transcription.txt
    └── VLM_agent/
```

## 🔧 Installation guide

1. **Clone the repo**  
   ```bash
   git clone https://github.com/Haoqingtianxia1997/Mosaic.git
   cd Mosaic
   ```

2. **Install system dependencies(for Ubuntu 22.04)**  
   ```bash
   sudo apt update
   sudo apt install portaudio19-dev python3-dev pulseaudio pulseaudio-utils
   ```

3. **Install Python dependencies**  
   ```bash
   conda create -n mosaic python=3.11
   conda activate mosaic
   pip install -r requirements.txt
   ```

4. **Set environment variables**  
   Add the following lines to `~/.bashrc`, `~/.zshrc` or `~/.config/fish/config.fish`:  
   ```bash
   export MISTRAL_API_KEY="your_mistral_api_key_here"
   ```

5. **Model source**
   ```
   src/VLM_agent/sam_hq/sam_vit_h_4b8939.pth: 
   https://huggingface.co/HCMUE-Research/SAM-vit-h/blob/main/sam_vit_h_4b8939.pth

   src/VLM_agent/FastSAM/FastSAM-x.pt:
   https://github.com/ultralytics/assets/releases/download/v8.3.0/FastSAM-x.pt
   ```

## 🎯 Run

1. **Bash script**  
   ```bash
   ./run.sh
   ```

2. **Main script**  
   ```bash
   python3 1.main.py
   ```

3. **Test intention detection**  
   ```bash
   python3 2.intention_predict.py
   ```


## 📝 TODO

- [ ] Incorporate gaze detection
- [ ] ROS2 service for movement control
- [ ] Camera calibration