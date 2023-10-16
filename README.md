# Cogniteam Component library for ROS humble
This library contains open dockerized components for ROS2
If you wish to use ROS check out our [ROS library](https://github.com/cogniteam/Library.Components.ROS/tree/master)

# Cogniteam’s Components Table
Image | Link
--- | ---
<img src="./components/generic-webcam/generic-webcam/generic-webcam-driver.jpg" alt="generic-webcam" width="40"/> | [generic-webcam](components/generic-webcam)
<img src="./components/richbeam-lakibeam/Lakibeam1-ROS2/lakibeam.png" alt="Lakibeam1-ROS2" width="40"/> | [richbeam-lakibeam](components/richbeam-lakibeam)
<img src="./components/ros-tutorials-talker/ros2-tutorials-talker/Cogniteam.jpg" alt="ros2-tutorials-talker" width="40"/> | [ros-tutorials-talker](components/ros-tutorials-talker)
<img src="./components/ros2-gateway/ros2-gateway/nimbusc.jpg" alt="ros2-gateway" width="40"/> | [ros2-gateway](components/ros2-gateway)
<img src="./components/rover-mini-driver/rover-driver/rover-mini-driver.jpg" alt="rover-driver" width="40"/> | [rover-mini-driver](components/rover-mini-driver)
<img src="./components/slamtec-rplidar-driver/ros2-slamtec-rplidar-a2/slamtec-rplidar-a2-driver.jpg" alt="ros2-slamtec-rplidar-a2" width="40"/> | [slamtec-rplidar-driver](components/slamtec-rplidar-driver)
<img src="./components/slamtec-rplidar-s2-driver/ros2-slamtec-rplidar-driver-s2/slamtec-rplidar--s2.jpg" alt="ros2-slamtec-rplidar-driver-s2" width="40"/> | [slamtec-rplidar-s2-driver](components/slamtec-rplidar-s2-driver)
<img src="./components/ydlidar-driver/ydlidar-driver/ydlidar-tia.png" alt="ydlidar-driver" width="40"/> | [ydlidar-driver](components/ydlidar-driver)

# Contirbuted Components Table 
Image | Link
--- | ---
<img src="https://github.com/AcutronicRobotics/gym-gazebo2/raw/dashing/imgs/alr_logo.png" alt="ros2learn" width="40"/> | [ros2learn](https://github.com/AcutronicRobotics/ros2learn/tree/dashing)
# Contribution
 If you wish to contribute by adding a new component to our library as part of our ongoing competition, please follow the instructions below:

 ## Prerequisites

Before you begin, ensure you have met the following requirements:

- You have a [GitHub](https://github.com) account.
- You have installed [Git](https://git-scm.com/).
- You have installed [Docker](https://www.docker.com/get-started).

## Forking and Cloning the Repository

1. **Fork the Repository**: Click on the 'Fork' button on the upper right-hand side of the page. A copy of the repository will be created on your personal GitHub account.
2. **Clone the Repository**: Clone the forked repository to your local machine by running:
   ```bash
   git clone https://github.com/cognimbus/Nimbus.Library.Components.ROS2.git
   ```

## Adding a New Component

### Option 1: Add a folder with your component's Dockerfile

#### 1. **Prepare Your Component Structure**
   - `comp_name`: Directory for your component
      - `docker`: Contains code and the Docker file
      - `img_file`: image represents the component

#### 2. **Create and Test Your Dockerfile**
   - Navigate to the cloned repository on your local machine.
   - Create a new Dockerfile with the required configurations for the ROS/ROS2 application you wish to containerize.
   - Test your Dockerfile locally with:
     ```bash
     docker build -t ros_app:<tag> .
     docker run --rm -it ros_app:<tag>
     ```

#### 3. **Place Your Files in the Directory Structure**
   - Place the Dockerfile and code into the appropriate directory structure within `comp_name/docker`.

#### 4. **Commit Your Changes**
   - After testing, commit your changes:
     ```bash
     git add .
     git commit -m "Your detailed commit message"
     ```
### Option 2: Add your git repository to ContributedComponents.MD

If you already have a git repository with a Dockerfile, simply add a link in ContributedComponents.MD and request to merge it. Make sure to add a relevant image from your git and make sure your git includes a valid Dockerfile that uses this version of ROS. 

#### Step 1: Prepare Your Image and Repository URL

Before adding a new row to the table, make sure you have the following:

1. **Image URL**: The URL of the image that represents your component. This should be hosted inside your Git repository. You can obtain the URL by navigating to the image file in your Git repository (e.g., on GitHub) and copying the URL.
   
2. **Repository URL**: The URL of your Git repository where the component is hosted.

#### Step 2: Add a New Row to the Table

To add a new component to the table, follow these steps:

1. **Open the Markdown File**: Open the markdown file where the table is located.

2. **Add a New Row**: Add a new row to the table with the following format:

   ```markdown
   Image | Link
   --- | ---
   <img src="IMAGE_URL" alt="COMPONENT_NAME" width="40"/> | [COMPONENT_NAME](REPOSITORY_URL)
   ```

3. **Replace Placeholders**: Replace `IMAGE_URL`, `COMPONENT_NAME`, and `REPOSITORY_URL` with the actual values:
   
   - `IMAGE_URL`: The URL of the image you prepared in step 1.
   - `COMPONENT_NAME`: The name of your component.
   - `REPOSITORY_URL`: The URL of your Git repository.

## Submitting a Merge Request

1. **Push Your Changes**: Push to your forked repository:
   ```bash
   git push origin <ros-distro>
   ```
2. **Create a Pull Request**: Navigate to your forked repository's GitHub page, click 'Pull request', and write a detailed comment.
3. **Submit**: Click 'Submit pull request'.

## Support and Contact

If you have questions or encounter issues, open an issue in the repository, and one of our maintainers will get back to you as soon as possible. Thank you for your contribution!

---

- [Nimbus Library Components for ROS](https://github.com/cognimbus/Nimbus.Library.Components.ROS)
- [Nimbus Library Components for ROS2](https://github.com/cognimbus/Nimbus.Library.Components.ROS2)
