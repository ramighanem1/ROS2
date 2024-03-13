# Setting Up ROS2 Workspace

This guide walks you through setting up a ROS2 workspace on your system. Follow the steps below:

## Step 1: Open Terminal
Open your terminal application.

## Step 2: Install Gedit
Run the following command to install Gedit :

```
pip install gedit
```

## Step 3: Add Source Command to ~/.bashrc
To avoid running the source command each time you open the terminal, follow these steps:

**Open ~/.bashrc with Gedit :**

```
gedit ~/.bashrc
```

**Add the following line to the end of the file :**

```
source /opt/ros/humble/setup.bash
```

Save and close the file.

## Step 4: Install ROS2 Build Tool - Colcon

Execute the following command to install the ROS2 build tool - Colcon:

```
sudo apt install python3-colcon-common-extensions
```

## Step 5: Add Command for Auto Colon Completion

To enable auto colon completion, do the following:

**Open ~/.bashrc with Gedit :**

```
gedit ~/.bashrc
```

**Add the following line to the end of the file :**

```
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
Save and close the file.

## Step 6: Create Workspace

Create a ROS2 workspace by executing the following commands:

```
cd ~
mkdir ros2_ws
```

## Navigate to Workspace and Create Source Folder :

**Move into the newly created workspace directory :**

```
cd ros2_ws
```

**Create a src folder where your ROS2 packages will reside :**

```
mkdir src
```
## Initialize Workspace and Build:

**Initialize the workspace using colcon for building packages :**

```
colcon build
```

## Configure Bash Environment:

**Open your ~/.bashrc file for editing :**

```
gedit ~/.bashrc
```

**Add the following line at the end of the file to automatically source your workspace setup script each time you open a terminal :**

```
source ~/ros2_ws/install/setup.bash
```

Save and close the file.






