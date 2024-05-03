# Autonomous Vehicle

Our project is about implementing a level-three autonomous functionality on an electric vehicle. For convenience's sake, we call our vehicle "historia".

# Setup

### ROS Binary Packages
TBA

### ZED 2 Camera
The Project relies heavily on ZED 2 for positional tracking, mapping, and obstacle detection. For that, we need to setup the camera first.

#### Install CUDA Toolkit (Rrquires Nvidia GPU)
- Check your supported CUDA version by typing `nvidia-smi` in terminal.
- Find the matching version from [archive](https://developer.nvidia.com/cuda-toolkit-archive) and install.
- We use Ubuntu 20.04, hence the choice would be <br/> `Linux > x86_64 > Ubuntu > 20.04 > runfile (local)`
- During installation, you will be presented with a list of items to install, deselect driver and kernel (furthest top and bottom).
- Once instation is completed, there will be a warning text about adding `export ...` to your `.bashrc` file. To do so, add the following to `~/.bashrc`: <br/>
  ```bash
  export PATH="/usr/local/cuda-{version}/bin:$PATH"
  export LD_LIBRARY_PATH="/usr/local/cuda-{version}/lib64:$LD_LIBRARY_PATH"
  ```
- Be sure to replace `{version}` with your installed CUDA toolkit version!
- To verify installation, run the command `nvcc --version`.

#### Install ZED SDK
- Download **ZED 4.1** SDK for Ubuntu 20 from [here](https://www.stereolabs.com/developers/release).
- Convert file to be executable then run.
- Accept license and proceed to type `y` for all dialogues besides the one that requires optimization of AI models (last dialogue).

