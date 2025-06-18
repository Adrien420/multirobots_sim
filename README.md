# Installation &nbsp;:
	
### -&nbsp; Install & setup docker (based on this [tutorial](https://faun.pub/ros2-humble-gui-docker-container-a-step-by-step-guide-c541b73fe141)) &nbsp;:

- **Install docker and its dependencies &nbsp;:**

```
sudo apt update

sudo apt install -y apt-transport-https ca-certificates curl software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update

sudo apt install -y docker-ce
```
- **(Optional) Use Docker without needing sudo privileges &nbsp;:**

```
sudo groupadd docker
sudo usermod -aG docker $(whoami)
```

Then restart the system to apply the group changes.

### -&nbsp; Clone the project &nbsp;:

- **Clone the main repository &nbsp;:**
```
cd ~/
git clone --recursive https://github.com/Adrien420/multirobots_sim.git
cd multirobots_sim/
git switch dev-sensors # Tmp
```
- **Setup the repository &nbsp;:**

```
# switch on branches defined by .gitmodules
git submodule foreach -q --recursive 'git switch $(git config -f $toplevel/.gitmodules submodule.$name.branch || echo master)' 

# checkout to tags defined by .gitmodules
git submodule foreach -q --recursive 'git checkout tags/$(git config -f $toplevel/.gitmodules submodule.$name.tag || echo master)' 

# Init PX4-Autopilot package's submodules
cd multirobots_ws/src/PX4-Autopilot
git submodule update --init --recursive 
```

### -&nbsp; Build the docker image &nbsp;:
	
```	
docker build -f Docker/Dockerfile --tag cristal-container .
```

# Usage &nbsp;:

### -&nbsp; Run the container &nbsp;:

```
./Docker/docker_run.sh
```

### -&nbsp; Compiling the first time (or after rebuilding the docker image) &nbsp;:

- **Compile the PX4's packages *(temporary instruction, since it will be implemented in the docker file in the future)* &nbsp;:**

```
# PX4-Autopilot
cd ~/multirobots_ws/src/PX4-Autopilot
make clean
make distclean
pip3 install kconfiglib
pip3 install jsonschema
pip3 install pyros-genmsg
make px4_sitl 

# Micro-XRCE-DDS-Agent
cd ~/multirobots_ws/src/Micro-XRCE-DDS-Agent/
mkdir -p build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

- **Compile the packages &nbsp;:**

The ros_gz_bridge package needs about **8G of RAM** to compile without problems.  
You can check if you have enough with the following command line :

```
free -h
```

If you don't, create a temporary swap file (you can replace the 4G with what is needed for you) :

```
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

Then, I would recommend to compile only the ros_gz_bridge package first :

```
cd ~/multirobots_ws
colcon build --packages-select ros_gz_bridge
```

Aftewards, compile the other packages :

```
colcon build
```

### -&nbsp; Run the simulation &nbsp;:

```
ros2 launch gazebo_sim multirobots_simu.launch.py rviz:=true
```