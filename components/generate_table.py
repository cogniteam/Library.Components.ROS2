import os

CONTAINING_DIR = 'components'

def get_repo_url(repo_name):
    return f"[{repo_name}]({CONTAINING_DIR}/{repo_name})"

# This method generates a link to the repo image
def generate_img_src(dir_path, repo_name, dir_name):
    try:
        files = os.listdir(os.path.join(dir_path, repo_name, dir_name))                                 # list the files at the repo nimbus directory
        files.remove('nimbusc.json')                                                                    # Remove the nimbus json file from the list (folder contains a json and an image)
        img_name = files[0]                                                                             # Extract the image's name
        return f"<img src=\"./{CONTAINING_DIR}/{repo_name}/{dir_name}/{img_name}\" alt=\"{dir_name}\" width=\"40\"/>"    # Return the link to the image
    except Exception as e:
        print('Error while trying to generate image link')
        print(str(e))

def generate_table():
    components_dir_path = os.path.dirname(__file__)
    repos = os.listdir(components_dir_path)

    non_components = ['generate_readme.py', 'generate_table.py', 'docker_retag.py', '.git', '.gitignore', '.gitlab-ci.yml', '.filter_only_updated_items.py',
                      'isaac-skeleton-viewer', 'README.md', 'json_retag.py']
    
    for element in non_components:
        try:
            repos.remove(element)
        except Exception as e:
            print(str(e).replace('x', element))

    library_dir_path = os.path.dirname(components_dir_path)
    with open(os.path.join(library_dir_path, 'README.md'), 'w') as f:
        table = "Image | Link\n--- | ---\n"
        for repo in sorted(repos):
            print(repo)
            repo_dir_path = os.path.join(components_dir_path, repo)
            dirs = os.listdir(repo_dir_path)
            files_to_remove = ['README.md', 'docker', '.gitignore', '.catkin_workspace']
    
            for file_name in files_to_remove:
                try:
                    dirs.remove(file_name)
                except ValueError:
                    pass
                
            for dir in dirs:
                table += f'{generate_img_src(components_dir_path, repo, dir)} | {get_repo_url(dir)}\n'
            
        f.write(table)
        f.write('\n')
        f.write(CONTENT)

CONTENT = """
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
   git clone https://github.com/cognimbus/Nimbus.Library.Components.ROS.git
   ```

## Adding a New Component

### 1. **Prepare Your Component Structure**
   - `comp_name`: Directory for your component
      - `docker`: Contains code and the Docker file
      - `nimbus`: Contains the Nimbus component JSON file and image with the name nimbusc (nimbusc.json, nimbusc.jpg, etc)

### 2. **Create and Test Your Dockerfile**
   - Navigate to the cloned repository on your local machine.
   - Create a new Dockerfile with the required configurations for the ROS/ROS2 application you wish to containerize.
   - Test your Dockerfile locally with:
     ```bash
     docker build -t ros_app:<tag> .
     docker run --rm -it ros_app:<tag>
     ```

### 3. **Place Your Files in the Directory Structure**
   - Place the Dockerfile and code into the appropriate directory structure within `comp_name/docker`.

### 4. **Commit Your Changes**
   - After testing, commit your changes:
     ```bash
     git add .
     git commit -m "Your detailed commit message"
     ```

## Submitting a Merge Request

1. **Push Your Changes**: Push to your forked repository:
   ```bash
   git push origin master
   ```
2. **Create a Pull Request**: Navigate to your forked repository's GitHub page, click 'Pull request', and write a detailed comment.
3. **Submit**: Click 'Submit pull request'.

## Support and Contact

If you have questions or encounter issues, open an issue in the repository, and one of our maintainers will get back to you as soon as possible. Thank you for your contribution!

---

- [Nimbus Library Components for ROS](https://github.com/cognimbus/Nimbus.Library.Components.ROS)
- [Nimbus Library Components for ROS2](https://github.com/cognimbus/Nimbus.Library.Components.ROS2)

**Options**:
- Option to push manually
- Option to push from CI master
- Option to build Dockers and push with tags

---

## ROS Component Configuration Guide

### 1. Component Information

#### `name` (Required)
- **Type**: String
- **Description**: The unique identifier for the component within the system, often represented by a combination of a namespace and the actual name, like `nimbus/hector`. This name is mandatory for identifying the component.

#### `type` (Required)
- **Type**: String
- **Description**: This field defines whether the component is a driver or a standard component.
- **Allowed Values**: 
  - `"COMPONENT_TYPE_COMPONENT"`: Denotes a standard component.
  - `"COMPONENT_TYPE_DRIVER"`: Denotes a driver component.

#### `className` (Required)
- **Type**: String
- **Description**: This field denotes the class to which the component belongs.
- **Allowed Values**: 
  - `"Ros1Component"`: Component is based on ROS1.
  - `"Ros2Component"`: Component is based on ROS2.

#### `instanceName` (Optional)
- **Type**: String
- **Description**: A custom name that is assigned to an instance of the component. If left empty, it will be set by default to the value in the `name` field.

#### `description` (Optional)
- **Type**: String
- **Description**: A user-friendly description of what the component does. For example, `"2D laser scan mapping using Hector mapping algorithm"` explains the component's function. This field can be helpful in understanding the purpose and functionality of the component.

#### `start` (Optional)
- **Type**: String
- **Description**: This field defines how the component will start. It's an optional field that can take two values:
  - `"START_MODE_AUTO"`: The component will load automatically upon system initialization.
  - `"START_MODE_MANUAL"`: The component must be manually started by the user.

Complete JSON section for reference:

```json
{
  "name": "nimbus/hector", 
  "type": "COMPONENT_TYPE_COMPONENT", 
  "className": "Ros1Component", 
  "instanceName": "", 
  "description": "2D laser scan mapping using Hector mapping algorithm", 
  "start": "START_MODE_AUTO"
}
```

### 2. Environment Configuration

#### `environment`

The `environment` field defines settings related to the Docker environment in which the component operates.

---

#### `dockerInfo`

This subsection provides detailed information about the Docker container's settings for the component.

- **`image` (Required)**
  - **Type**: String
  - **Description**: The name of the Docker image to be used for the component. Example: `"cognimbus/usb-cam:latest"` specifies the image named `cognimbus/usb-cam` with the tag `latest`.

- **`commands` (Optional)**
  - **Type**: Array of Strings
  - **Description**: The list of commands or arguments to be passed to the Docker container upon startup. For ROS-based components, this often includes commands like `roslaunch` and relevant parameters.

- **`privileged` (Optional)**
  - **Type**: Boolean
  - **Description**: Specifies whether the Docker container runs in privileged mode. This mode allows the container broader access to the host system's resources. Default is `false`.

- **`gpu` (Optional)**
  - **Type**: Boolean
  - **Description**: Indicates if the Docker container should have access to the host's GPU resources. Default is `false`.

- **`networkHost` (Optional)**
  - **Type**: Boolean
  - **Description**: Determines if the Docker container uses the host's network stack. Default is `false`.

- **`binds` (Optional)**
  - **Type**: Array of Objects
  - **Description**: This field defines any directory or device binds from the host system to the Docker container. Each bind object must have a `source` (path on the host system) and a `target` (path inside the container).

- **`user` (Optional)**
  - **Type**: String
  - **Description**: The user or UID to run commands inside the Docker container. If left empty, it defaults to the user specified in the Docker image.

- **`onExit` (Optional)**
    - **Type**: String
    - **Description**: This field determines the action to take when the Docker container exits. For instance, `"IGNORE"` implies that the exit will not trigger any additional action.

Complete JSON section for reference:

```json
"environment": {
  "dockerInfo": {
    "image": "hector1:latest",
    "commands": [
      "roslaunch",
      "hector_mapping",
      "hector_mapping.launch",
      "map_resolution:=${map_resolution}",
      "map_size:=${map_size}",
      "base_frame:=${base_frame}"
    ],
    "privileged": true,
    "gpu": false,
    "networkHost": false,
    "binds": [
      {
        "source": "/dev/video0",
        "target": "/dev/video0"
      }
    ],
    "user": ""
  },
  "onExit": "IGNORE"
}
```

### 3. Component Parameters
This section defines parameters passed to the component's launch file, including the parameter name, data type, and value.

#### `parameters`
A list of key-value pairs containing the information related to the parameters.

- **`name`**: The name of the parameter.
  - **Type**: String
  - **Example**: `"map_resolution"`
  - **Description**: Specifies the name of the parameter as it is used within the component.

- **`doubleValue`, `stringValue`, `etc.`**: The value of the parameter, specified in the appropriate data type.
  - **Type**: Depending on the parameter, this could be a double, string, integer, boolean, etc.
  - **Example**: For a parameter `"map_resolution"`, the double value might be `"doubleValue": 0.1`.
  - **Description**: This represents the actual value of the parameter that will be used by the component. Different parameters may require different data types.

Complete JSON section for reference:

```json
"parameters": {
  "parameters": [
    {
      "name": "map_resolution",
      "doubleValue": 0.1
    },
    {
      "name": "map_string",
      "stringValue": "string-value"
    }
  ]
}
```

This configuration enables you to specify various parameters needed for the launch file with different data types, providing flexibility in component configuration.

### 4. Streams Configuration
This section defines the input and output streams of the component, including their names, types, and topics.

#### `inputStreams`
A list of objects that define the input streams that the component will receive.

- **`name`** (required): The name of the input stream.
  - *Type*: String
  - *Example*: `"scan"`
  - *Description*: The identifier for the input stream.

- **`ros_topic`** (required): The ROS topic information for the input stream.
  - **`topic`** (required): The name of the ROS topic.
    - *Type*: String
    - *Example*: `"/scan"`
    - *Description*: Specifies the ROS topic to subscribe to for receiving data.
  - **`type`** (required): The data type of the messages on the ROS topic.
    - *Type*: String
    - *Example*: `"Messages.sensor_msgs.LaserScan"`
    - *Description*: Specifies the message type for the input stream.

- **`latched`** (optional): Whether the input stream is latched.
  - **Type**: Boolean
  - **Example**: `false`
  - **Description**: If set to true, the component will receive the last published message on the topic even if it was published before the component subscribed.

#### `outputStreams`
A list of objects that define the output streams that the component will send.

- **`name`** (required): The name of the output stream.
  - **Type**: String
  - **Example**: `"map"`
  - **Description**: The identifier for the output stream.

- **`ros_topic`** (required): The ROS topic information for the output stream.
  - **`topic`** (required): The name of the ROS topic.
    - **Type**: String
    - **Example**: `"/map"`
    - **Description**: Specifies the ROS topic to publish data to.
  - **`type`** (required): The data type of the messages on the ROS topic.
    - **Type**: String
    - **Example**: `"Messages.nav_msgs.OccupancyGrid"`
    - **Description**: Specifies the message type for the output stream.

Complete JSON section for reference:

```json
"streams": {
  "inputStreams": [
    {
      "name": "scan",
      "ros_topic": {
        "topic": "/scan",
        "type": "Messages.sensor_msgs.LaserScan"
      },
      "latched": false
    }
  ],
  "outputStreams": [
    {
      "name": "map",
      "ros_topic": {
        "topic": "/map",
        "type": "Messages.nav_msgs.OccupancyGrid"
      }
    }
  ]
}
```

This configuration provides a clear definition of what data streams the component interacts with, including details on which parameters are required or optional, allowing for precise control and understanding of the component's communication.

### 5. ROS Configuration
This section allows the component to view all devices using other components. Using the ROS tf package, Nimbus maintains the relationship between the coordinate frames of connected devices and the platform's base frame (e.g., `base_link`).

- **`base_frame`** (optional): Platform's base frame.
  - **Type**: String
  - **Example**: `"base_link"`
  - **Description**: Specifies the base frame for the ROS node, used for transforming coordinates between different frames.

- **`rate`** (required): Rate of TF messages publish.
  - **Type**: Float
  - **Example**: `10.0`
  - **Description**: Specifies the frequency at which the ROS node will publish TF messages.

- **`publishTfDevices`** (optional): Determines if the component should publish TF information for devices.
  - **Type**: Boolean
  - **Example**: `true`
  - **Description**: If set to true, the component will publish the TF information, maintaining the relationship between the coordinate frames of connected devices.

- **`rosMasterUri`** (optional)
  - **Type**: String
  - **Example**: `""`
  - **Description**: Specifies the URI for connecting to the ROS Master server. If left empty, the system default will be used.

- **`rosIp`** (optional):
  - **Type**: String
  - **Example**: `""`
  - **Description**: Specifies the IP address for the ROS node. If left empty, the system will use the default network interface.

Complete JSON section for reference:


```json
"ros": {
  "base_frame": "base_link",
  "rate": 10.0,
  "publishTfDevices": true,
  "rosMasterUri": "",
  "rosIp": "",
  "autoDetectIp": false
}
```

These configurations enable control over various ROS-related parameters, including coordinate frame settings, communication rate, and network configuration. They allow for precise control over the ROS environment in which the component operates, including the handling of devices through the `publishTfDevices` option.

These configurations enable control over various ROS-related parameters, including coordinate frame settings, communication rate, and network configuration. They allow for precise control over the ROS environment in which the component operates.

### 6. Required Devices
This section lists the required devices for the component, including their names, types, and vendor information. Each device entry consists of the following attributes:

- **`name`** (required): A unique identifier for the device.
  - **Type**: String
  - **Example**: `"laser"`
  - **Description**: A human-readable name to identify the device within the system.

- **`info`** (required): Contains detailed information about the device.
  - **`type`** (required): Specifies the port type for the device.
    - **Type**: String
    - **Example**: `"USB_PORT_TYPE_SERIAL"`
    - **Description**: Defines the interface type through which the device connects.
  - **`productId`** (required): The product ID associated with the device.
    - **Type**: String
    - **Example**: `"ea60"`
    - **Description**: A unique identifier for the specific model of the device.
  - **`vendorId`** (required): The vendor ID associated with the device.
    - **Type**: String
    - **Example**: `"10c4"`
    - **Description**: A unique identifier for the manufacturer of the device.

- **`mountAs`** (required): Specifies where the device should be mounted within the system.
  - **Type**: String
  - **Example**: `"/dev/ttyUSB0"`
  - **Description**: Provides the system path to which the device will be mounted, enabling software components to access it.

Complete JSON section for reference:


```json
"requiredDevices": [
  {
    "name": "laser",
    "info": {
      "type": "USB_PORT_TYPE_SERIAL",
      "productId": "ea60",
      "vendorId": "10c4"
    },
    "mountAs": "/dev/ttyUSB0"
  }
]
```

This section ensures that the necessary devices are properly defined and accessible for the component to function. It provides essential information for connecting and interacting with hardware peripherals, aligning with the system's specifications and requirements.

Certainly! Here's the detailed breakdown for the "Optional Configuration for Running ROS Components Without Docker" section:

### Optional Configuration for Running ROS Components Without Docker
These configurations allow the component to run as a native ROS process without Docker, providing an alternative way to execute the component.

#### `environment` 
This section provides details about the ROS environment to set up the component.

- **`rosLocalInfo`** : Contains information specific to the ROS local environment.
  - **`PackageName`** (required): The name of the ROS package.
    - **Type**: String
    - **Example**: `"hector_mapping"`
    - **Description**: Specifies the ROS package containing the launch file.
  - **`launchFile`** (required): The specific launch file to run.
    - **Type**: String
    - **Example**: `"mapping_default.launch"`
    - **Description**: The ROS launch file that initializes the component.
  - **`rosVersion`** (optional): Specifies the ROS version.
    - **Type**: String
    - **Example**: `""`
    - **Description**: If provided, indicates the specific version of ROS to use.
  - **`workspaceSetup`** (required): Path to the ROS workspace setup file.
    - **Type**: String
    - **Example**: `"/some_ws/devel/setup.bash"`
    - **Description**: Specifies the shell script to source the ROS workspace.
  - **`required`** (optional): Indicates if the local environment setup is mandatory.
    - **Type**: Boolean
    - **Example**: `true`
    - **Description**: If true, this local environment setup is necessary.

#### `parameters` (optional)
This section allows for the specification of additional parameters for the ROS environment.

- **`parameters`** (optional): A list of key-value pairs for additional configuration.
  - **`name`** (required): The parameter name.
    - **Type**: String
    - **Examples**: `"ros_ip"`, `"ros_master_uri"`
    - **Description**: Specifies the parameter's identifier.
  - **`stringValue`** (required): The parameter value.
    - **Type**: String
    - **Examples**: `"192.168.1.32"`, `"http://192.168.1.32:11311"`
    - **Description**: Specifies the value for the corresponding parameter.

```json
"environment": {
  "rosLocalInfo": {
    "PackageName": "hector_mapping",
    "launchFile": "mapping_default.launch",
    "rosVersion": "",
    "workspaceSetup": "/opt/ros/melodic/setup.bash",
    "required": true
  }
},
"parameters": {
  "parameters": [
    {
      "name": "ros_ip",
      "stringValue": "192.168.1.32"
    },
    {
      "name": "ros_master_uri",
      "stringValue": "http://192.168.1.32:11311"
    }
  ]
}
```

This section provides flexibility in running the component as a native ROS process, bypassing the need for Docker. It offers specific configurations tailored to ROS, ensuring compatibility and proper functioning within a given environment.
"""

generate_table()