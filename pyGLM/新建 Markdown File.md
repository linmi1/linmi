conda activate gaussian_grouping

cd /home/vision/work/linmi2/pyGLM

Xvfb :99 -screen 0 800x600x24  & export DISPLAY=:0

Xvfb :99 -screen 0 800x600x24 & python main.py