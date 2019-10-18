## roseus_resume

### Setup
You will need to install `euslisp` and `roseus` from source and set them to the `eus_handler` branch.

The following will guide you through this:

```bash
mkdir ~/roseus_resume_ws/src -p
cd ~/roseus_resume_ws/src/

# clone euslisp
wstool init
wget https://raw.githubusercontent.com/jsk-ros-pkg/jsk_roseus/master/setup_upstream.sh -O /tmp/setup_upstream.sh
bash /tmp/setup_upstream.sh -w ..

# clone roseus
wstool set jsk-ros-pkg/jsk_roseus --git https://github.com/jsk-ros-pkg/jsk_roseus.git -v master -u -y
cd euslisp/Euslisp/

# checkout euslisp/eus-handler
git remote add Affonso-Gui https://github.com/Affonso-Gui/EusLisp.git
git fetch Affonso-Gui 
git checkout eus-handler

# checkout roseus/eus-handler
cd ~/roseus_resume_ws/src/jsk-ros-pkg/jsk_roseus/
git remote add Affonso-Gui https://github.com/Affonso-Gui/jsk_roseus.git
git fetch Affonso-Gui 
git checkout eus-handler 
cd ~/roseus_resume_ws/src/

# clone roseus_resume
git clone https://github.com/Affonso-Gui/roseus_resume.git
cd ~/roseus_resume_ws/

# build
catkin build
```

### Usage

Commented example usage is given at [sample.l](https://github.com/Affonso-Gui/roseus_resume/blob/master/euslisp/sample.l).