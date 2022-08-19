# Turtlebot tracker

A turtle bot tracker is capable of following a person by tracking different Aruco markers and controlling its speed with Proportional Integral feedback. Its software relies on ROS Melodic and runs on a Turtlebot2. Code is developed for the final project of the course ROB514 "Introduction to Robotics" at OSU.

## Setup

To set up the Python version this code relies on, you need to do the following.

```bash
cd /home # Go to home directory
sudo mkdir python-envs # Create folder for virtual python environments
cd python_envs # Navigate into folder containing virtual python environments
sudo python3 -m venv ros-melodic # Create python virtual environment
sudo chown -R <your-linux-username> ros-melodic # Makes you the owner of the folder
source ros-melodic/bin/activate # Source the python version in the virtual environment
```

To install the required python dependencies, run the following.

```bash
pip install --upgrade pip  # Upgrade pip
pip install opencv-python  # Install opencv
pip install rospkg         # Install ros packages
```
For the repo to work properly, you'll need to set up the Turtlebot2 packages for ROS Melodic. Follow the instructions [here](https://github.com/gaunthan/Turtlebot2-On-Melodic) to set them up.

Now, set up and build our package with the following commands, assuming you have built a catkin workspace as ```~/catkin_ws```.

```bash
cd ~/catkin_ws/src # Navigate to catkin workspace source directory
git clone git@github.com:EverardoG/hot_chocolate.git # Clone the repository
cd .. # Navigate back to catkin_ws
catkin_make # Build all packages
source devel/setup.sh # Source you workspace
```
## Run

To run the package, simply run a ```roslaunch``` command of the ```main.launch``` file, like so.

```bash
roslaunch hot_chocolate main.launch
```
This will launch the minimal turtlebot launch files as well as run this package's specific scripts.

## Contributing
Other contributors are: [Everardo Gonzalez](https://github.com/EverardoG)

Mentor: [Prof Cindy Grimm](https://github.com/cindygr)

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)
