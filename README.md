# fetchit-rl-grasping
DDPG+HER with demonstration RL grasping experiments for CS6731 final project

See [writeup](https://github.com/cerrno/fetchit-rl-grasping/blob/master/Humanoid%20Robotics%20Project%20Report.pdf)

![Training](/pictures/Screenshot%20from%202019-05-07%2020-19-56.png)

## License
[MIT](https://lucasschuermann.com/license.txt)

## Usage Notes

```
pip3 install -U 'mujoco-py<2.1,>=2.0'
pip3 uninstall tensorflow
pip3 install tensorflow==1.13.1
```

```
cd gym
pip3 install -e .
pip3 install -r baselines/requirements.txt
```

Use `LD_PRELOAD` if you have nvidia gpu on Ubuntu, modify `num_cpu` to preference
```
cd lvs-exploration
export PYTHONPATH=".:..:../baselines"

LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so python3 data_generation/pick_data_generation.py

LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so python3 experiment/train.py --num_cpu=6
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so python3 experiment/train.py --num_cpu=6 --env LVSRotatedFetchPickAndPlace-v1 --demo_file data_rotated_random_100.npz 

LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so python3 experiment/play.py default_results/policy_best.pkl

python3 experiment/plot.py /tmp/openai-2019-05-07-22-02-18-894910/
mv /tmp/openai-2019-05-07-22-02-18-894910/figure[...] results/figure[...]
```
