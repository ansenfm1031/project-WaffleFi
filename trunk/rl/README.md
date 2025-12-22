### Create a symbolic link from this path to the ROS workspace

```bash
ln -s ~/intel-edge-ai-sw-8/2601_4th_proj_dahyeon/trunk/rl/proj_wifi_heatmap_rl ~/turtlebot3_ws/src/proj_wifi_heatmap_rl
```

### Build and Run

```bash
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```

```bash
cd ~/turtlebot3_ws
python3 -m proj_wifi_heatmap_rl/train_dqn.py
python3 -m proj_wifi_heatmap_rl/train_dqn.py --resume runs_dqn/dqn_ep0800.pt --more-episodes 200
python3 -m proj_wifi_heatmap_rl/train_dqn.py --resume runs_dqn/dqn_ep0800.pt --total-episodes 1000
python3 -m proj_wifi_heatmap_rl/train_dqn.py --resume runs_dqn/dqn_ep0800.pt --more-episodes 200 --reset-optim
```
