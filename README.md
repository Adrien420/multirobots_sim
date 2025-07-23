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
git clone --recursive https://github.com/Adrien420/multirobots_sim.git

# Install Git LFS to pull large files
cd multirobots_sim/
sudo apt-get install git-lfs
git lfs pull
```
- **Setup the repository &nbsp;:**

```
# switch on branches defined by .gitmodules
git submodule foreach -q --recursive 'git switch $(git config -f $toplevel/.gitmodules submodule.$name.branch || echo master)' 

# Init PX4-Autopilot package's submodules
cd multirobots_ws/src/PX4-Autopilot
git submodule update --init --recursive 
```

### -&nbsp; Build the docker image &nbsp;:
	
```	
docker build -f Docker/Dockerfile --tag cristal-container .
```

### -&nbsp; (Optional) Configuration to use GPU acceleration in Docker with NVIDIA GPUs (cf [nvidia_doc](https://docs.nvidia.com/ai-enterprise/deployment/vmware/latest/docker.html)) &nbsp;:

- **Installation &nbsp;:**

```
# Configure the production repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update the packages list from the repository
sudo apt-get update

# Install the NVIDIA Container Toolkit packages
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.17.8-1
sudo apt-get install -y \
    nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
    nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
    libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
    libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
```

- **Configuration &nbsp;:**

```
# Configure the container runtime by using the nvidia-ctk command
sudo nvidia-ctk runtime configure --runtime=docker

# Restart the Docker daemon
sudo systemctl restart docker
```

# Usage &nbsp;:

### -&nbsp; Run the container &nbsp;:

```
./Docker/docker_run_intel.sh
```

If you are using NVIDIA GPU acceleration, run this script instead :

```
./Docker/docker_run_nvidia.sh
```

### -&nbsp; Compiling the first time (or after rebuilding the docker image) &nbsp;:

- **Compile the PX4-Autopilot package &nbsp;:**

```
cd ~/multirobots_ws/src/PX4-Autopilot
make clean
make distclean
pip3 install kconfiglib
pip3 install jsonschema
pip3 install pyros-genmsg
make px4_sitl 
```

- **Compile the Micro-XRCE-DDS-Agent package &nbsp;:**

```
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
source install/setup.bash
```

### -&nbsp; Run launch files & nodes &nbsp;:

- **Launch the simulation &nbsp;:**

Launch arguments &nbsp;:

| Argument | Type | Default Value | Usage |
| :---: | :---: | :---: | :---: |
| `rviz` | bool | false | Choose whether you launch Rviz or not |
| `rosbag` | bool | false | Choose whether you register data with rosbag or not |
| `nb_summits` | int | 1 | Choose the number of Summit XL to spawn |
| `nb_drones` | int | 1 | Choose the number of drones to spawn |

Command (example) &nbsp;:

```
ros2 launch gazebo_sim multirobots_simu.launch.py rviz:=true nb_drones:=3
```

# Troubleshooting &nbsp;:

### -&nbsp; Launch file not working after running a new intsance of Docker container &nbsp;:

Normally, once everything has been compiled the first time, everything should work out of the box each time.
However, in a recent installation, I found it wouldn't work when running a new container, without compiling the Micro-XRCE-DDS-Agent package again.

In that case, the easiest fix for now is just to recompile it each time, with :

```
cd ~/multirobots_ws/src/Micro-XRCE-DDS-Agent/
mkdir -p build
cd build
cmake ..
make
sudo make install
sudo ldconfig

cd ~/multirobots_ws
colcon build
```

### -&nbsp; Launch file not working all of a sudden in the same Docker container &nbsp;:

If nothing has been changed in the project, it is usually caused by a gz instance still running.

To check that, use :

```
ps aux | grep gz
```

And kill any instance remaining with :

```
kill -9 <instance_id>
```

### -&nbsp; Drone not taking off despite rotors spinning &nbsp;:

If you changed something in the models or in the drone's spawn position, there is a chance its legs are inside the terrain collider.
In this case, either the drone is spawned too low, or the max_step_size (defined in gazebo_sim/worlds/forest.sdf) is too high and the collision isn't detected soon enough. Of course, don't lower the max_step_size too much, since it will make the simulation much slower.