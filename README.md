## roseus_resume

### Setup
You will need to install `euslisp` and `roseus` from source and set them to the `eus10` branch.

The following will guide you through this:

```bash
mkdir ~/roseus_resume_ws/src -p
cd ~/roseus_resume_ws/src/

# clone
wstool init .
wstool merge -t . https://gist.githubusercontent.com/Affonso-Gui/25518fef9dc7af0051147bdd2a94b116/raw/e3fcbf4027c876329801a25e32f4a4746200ddae/guiga_system.rosinstall
wstool update -t .

# To use eus10, furuschev script is required.
wget https://raw.githubusercontent.com/jsk-ros-pkg/jsk_roseus/master/setup_upstream.sh -O /tmp/setup_upstream.sh
bash /tmp/setup_upstream.sh -w ../ -p jsk-ros-pkg/geneus -p euslisp/jskeus

# install dependencies
rosdep install -y -r --from-paths . --ignore-src

# build
cd ../
catkin build roseus_resume pr2eus
```

### Usage

Commented example usage is given at [sample.l](https://github.com/Affonso-Gui/roseus_resume/blob/eus10/euslisp/sample.l).
