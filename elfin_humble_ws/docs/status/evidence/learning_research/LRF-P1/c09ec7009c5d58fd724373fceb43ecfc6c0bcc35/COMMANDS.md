# LRF-P1 reproduction

```bash
source /opt/ros/humble/setup.bash  # optional; harness is ROS-free
cd /home/adamliao/work/elfin_humble_ws_eng_lrfp1
export PYTHONPATH="$PWD:$PWD/src/luggage_description:$PWD/src/luggage_perception"
python3 -m unittest discover -s research/lrf_p1/tests -p 'test_*.py'
python3 research/lrf_p1/run_spike.py --workspace "$PWD" --tests-result pass \
  --out docs/status/evidence/learning_research/LRF-P1/<revision>
```

Dependencies: numpy, scikit-learn. No downloaded weights. Six sized suitcase
STLs under `src/luggage_gazebo/models/suitcase_{loafbrr,vintage}_{small,medium,large}`.
