# Mosaic

Inspired from the paper: https://portal-cornell.github.io/MOSAIC/, the project's target is to use different interaction methods, e.g. voice command, gesture and gaze from the user, as well as the reasoning capability from LLM and VLM to achieve generalized task planning for various kinds of objects in a hierachical manner. 

<!-- ![Workspace overview](overview.png) -->

<p align="center">
  <img src="./overview.png" alt="Workspace overview" width="800"/>
</p>
<p align="center"><em>Fig. 1  Workspace overview</em></p>

Our code is divided into two parts: high level and low level(manipulation_ws). For detailed high level structure please check the README.md inside `./high_level` folder. low level part contains the motions that we need to use Franka's Panda robot arm for this task, which is developed with ROS2 humble. 

## 🔧 Installation guide


1. **Install system dependencies(for Ubuntu 22.04)**  
   ```bash
   sudo apt update
   sudo apt install portaudio19-dev python3-dev pulseaudio pulseaudio-utils
   ```

2. **Build ros2 packages**
   ```bash
   cd manipulation_ws
   source /opt/ros/humble/setup.bash
   colcon build
   ```

2. **Install Python dependencies**  
   ```bash
   conda create -n mosaic python=3.10
   conda activate mosaic
   pip install -r requirements.txt
   ```

3. **Set environment variables**  
   Add the following lines to `~/.bashrc`, `~/.zshrc` or `~/.config/fish/config.fish`:  
   ```bash
   export MISTRAL_API_KEY="your_mistral_api_key_here"
   ```
   or
   ```bash
   export OPENAI_API_KEY="your_openai_api_key_here"
   ```

4. **Model source**
   ```
   high_level/src/VLM_agent/sam_hq/sam_vit_h_4b8939.pth: 
   https://huggingface.co/HCMUE-Research/SAM-vit-h/blob/main/sam_vit_h_4b8939.pth

   high_level/src/VLM_agent/FastSAM/FastSAM-x.pt:
   https://github.com/ultralytics/assets/releases/download/v8.3.0/FastSAM-x.pt
   ```

## Run the demo
```bash
chmod +x launch.sh
./launch.sh

cd high_level
python3 1.main.py --participant P001
```
Tmux is used to manage all the necessary components. Use Ctrl + b then a number in 0~6 to switch among all the sessions.

To close the multiplexer(first detach with Ctrl+b then d) : 
```bash
tmux kill-session -t mosaic
```

## Data Recording

All scripts use `--participant` to route saved data. If the corresponding `<participant>_folder/` does not exist under `manipulation_ws/saved_intention_data/`, data falls back to `unknown_folder/`.

**ROS bag + point cloud** (`bag_record.py`):
```bash
cd manipulation_ws
python3 src/action/action/bag_record.py --participant P001
```
Sessions are saved under numbered subfolders: `<participant>_folder/NN/bag/` for bags and `<participant>_folder/NN/<label>.ply` for point clouds.

**Grasp meshes** (`1.main.py`):

During grasp execution, the best gripper and object meshes are automatically saved to `<participant>_folder/grasp/` as sequentially numbered PLY pairs (`001_object.ply` / `001_gripper.ply`, `002_object.ply` / `002_gripper.ply`, …).

To visualize saved grasp results:
```bash
# Browse all pairs one by one (close window to advance)
python3 manipulation_ws/saved_intention_data/visualize_grasp.py --participant P001

# Show only a specific pair
python3 manipulation_ws/saved_intention_data/visualize_grasp.py --participant P001 --idx 2
```

