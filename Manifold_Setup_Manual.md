#DJI Manifold Setup Manual 

###1. Install ROS Indigo (Ubuntu armhf)

- First follow the instructions [here](http://wiki.ros.org/indigo/Installation/UbuntuARM)

- Then install several 3rd-party ROS Pack??????????/ages:
```
sudo apt-get install ros-indigo-cv-bridge
sudo apt-get install ros-indigo-aruco
sudo apt-get install ros-indigo-camera-info-manager
sudo apt-get install ros-indigo-v4l-utils
```

###2. Install CUDA 6.5: 

- Download the source file from [here](http://developer.download.nvidia.com/embedded/L4T/r21_Release_v3.0/cuda-repo-l4t-r21.3-6-5-prod_6.5-42_armhf.deb)

- Installation: 
```
sudo dpkg -i cuda-repo-l4t-r21.3-6-5-prod_6.5-42_armhf.deb
sudo apt-get update
sudo apt-get install cuda-toolkit-6-5
```

- Set GPU to be accessible by current user:

```
sudo usermod -a -G video $USER
```


- Set the environment variable:

```
gedit ~/.bashrc
```

- Then add the following lines to ~/.bashrc:

```
export PATH=/usr/local/cuda-6.5/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-6.5/lib:$LD_LIBRARY_PATH
```


- Then source the .bashrc file again:

```
source ~/.bashrc
```

###3. Install OpenCV4Tegra:

- Download the source file from [here](http://developer.download.nvidia.com/embedded/OpenCV/L4T_21.2/libopencv4tegra-repo_l4t-r21_2.4.10.1_armhf.deb)

- Install the dependencies:
```
sudo dpkg -i libopencv4tegra-repo_l4t-r21_2.4.10.1_armhf.deb
sudo apt-get update
sudo apt-get install libopencv4tegra libopencv4tegra-dev libopencv4tegra-python
sudo apt-get install libgtk2.0-dev pkg-config
```

###4. Install OpenCV 2.4.10

- Download the source file from [here](https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.10/opencv-2.4.10.zip/download)

- Unzip:

```
unzip opencv-2.4.10.zip
```

- Compile OpenCV:

Under the parent directory of "opencv-2.4.10", make another directory named "build":
```
mkdir build
cd build
cmake -DWITH_CUDA=ON -DCUDA_ARCH_BIN="3.2" -DCUDA_ARCH_PTX="" -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF ../opencv-2.4.10/
```
Take a look at the message after cmake command is completed. If `Use CUDA` is `Yes`, then CUDA can be used.

PS: If "cmake" cannot be find after typing the above commands, please install the essential applications first:
```
sudo apt-get install build-essential make cmake g++
```

- Install OpenCV:
```
sudo make -j4 install
```

- Modify the environment variables:
```
echo "# Use OpenCV and other custom-build libraries" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> ~/.bashrc
source ~/.bashrc
```