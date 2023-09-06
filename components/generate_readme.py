import os
import json
import requests
import re


def generate_headline(repo_name:str):
    return f"# {repo_name.title()}"


# This method generates a link to the repo image
def generate_img_src(dir_path, repo_name):
    try:
        files = os.listdir(os.path.join(dir_path, repo_name))          
        print(files)                      # list the files at the repo nimbus directory
        files.remove('nimbusc.json')                                                         # Remove the nimbus json file from the list (folder contains a json and an image)
        img_name = files[0]                                                                  # Extract the image's name
        return f"<img src=\"./{repo_name}/{img_name}\" alt=\"{repo_name}\" width=\"400\"/>"  # Return the link to the image
    except Exception as e:
        print('Error while trying to generate image link')
        print(str(e))


# This method extract the docker image name from the nimbus json file
def extract_image_from_json(dir_path, repo_name):
    try:
        json_file = open(os.path.join(dir_path, repo_name, 'nimbusc.json'), 'r')        # Open json file
        json_content = json.load(json_file)                                             # Load the json content
        docker_image = json_content['environment']['dockerInfo']['image'].split(':')[0] # Extract the docker image name from the json file   
        return docker_image                                                             # Return the docker image name
    except Exception as e:
        print("Error while trying to extract docker image from json file")
        print(str(e))


# This method returns a link to the docker hub image url
def generate_dockerhub_image(dir_path, repo_name):
        docker_image = extract_image_from_json(dir_path, repo_name)
        return f"* Dockerhub image https://hub.docker.com/r/{docker_image}"


# This method extract the supported architecture of the docker image from the docker hub api
def generate_supported_arch(dir_path, repo_name):
    try:
        image_name = extract_image_from_json(dir_path, repo_name)                                           # Extract the docker image name from json file
        registry_url = "https://hub.docker.com/v2/repositories"                                             # Docker hub api registry
        tags_endpoint = f"{registry_url}/{image_name}/tags"                                                 # Docker hub api endpoint for the relevant image
        response = requests.get(tags_endpoint)                                                              # Send request to docker hub api
        
        if response.status_code != 200:
            raise Exception(f"Failed to retrieve tags for image '{image_name}'. Error: {response.text}")
        
        images = response.json()['results'][0]['images']                                                    # Extract image's list
        architectures = [image['architecture'] for image in images]                                         # Extract image's architectures
        return f"* Supported architectures <b>{'/'.join(architectures)}</b>"                                # Return supported architecture string with relevant architectures
    except Exception as e:
        print('Error while trying to extract architectures from docker hub')
        print(str(e))


# This method extracts the ros version of the component from the docker file
def generate_ros_version(dir_path, repo_name):
    try:
        docker_file = open(os.path.join(dir_path, 'docker', 'Dockerfile'), 'r')
        ros_version = docker_file.readline().split(':')[-1]
        return  f"* ROS version <b>{ros_version}</b>"
    except Exception as e:
        print('Error while trying to extract ros version from Dockerfile')
        print(str(e))


# This method extract the description from the json file and return description string for readme
def generate_description(dir_path, repo_name):
    try:
        json_file = open(os.path.join(dir_path, repo_name, 'nimbusc.json'), 'r')
        json_content = json.load(json_file)
        description = json_content['description']
        return f"# Short description\n* {description}"
    except Exception as e:
        print('Error while trying to extract description from json file')
        print(str(e))

# This method extract the roslaunch arguments from the json file parameters section
def extract_launch_arguments(params):
    args = {}
    for param in params:
        val_key = [key for key in param.keys() if key.endswith('Value')][0]
        args[param['name']] = param[val_key]
    return args

def generate_roslaunch(commands, params):
    launch_args = extract_launch_arguments(params)
    pattern = r"\{(.*?)\}"                              # Regular expression pattern to match content inside curly braces
    for i in range(len(commands)):
        match = re.search(pattern, commands[i])         # Search for string with a matching pattern
        if match:
            content = match.group(1)
            arg_name = commands[i].split(':')[0]
            if '.' in content:
                commands[i] = f"{arg_name}:={content.split('.')[0]}"
            else:
                commands[i] = f"{arg_name}:={launch_args[content]}"  # Replace the arg from json file with the parameter's default value
    return commands

# This method extract docker's -v flag for mounts based on the binds section in the nimbus josn file
def extract_binds(binds):
    if len(binds) == 0:
        return ''
    
    string = ""
    for bind in binds:
        string += f"-v {bind['source']}:{bind['target']}"
        string += ' '
    return string

# This method generate's example usage of how to run the docker image
def generate_example_usage(dir_path, repo_name):
    # TODO: gpu, runtime
    flags = {'privileged':'--privileged', 'runtime':'--runtime'}
    try:
        json_file = open(os.path.join(dir_path, repo_name, 'nimbusc.json'), 'r')        # Open json file
        json_content = json.load(json_file)                                             # Load the file's content
        commands = json_content['environment']['dockerInfo']['commands']                # Extract the commands from the json file
        params = json_content['parameters']['parameters']                               # Extract the parameters from the json file
        commands = generate_roslaunch(commands, params)                                 # generate roslaunch command
        example = f"# Example usage\n```\ndocker run -it --network=host "               # Head of the example usage string
        docker_info = json_content['environment']['dockerInfo']                         # Extract the docker info from the json file
        docker_image = json_content['environment']['dockerInfo']['image']
        
        # Iterate over the docker flags
        for flag in flags.keys():
            try:
                # if the flag is set to true at json file, add it to the example usage
                if docker_info[flag]:
                    example += flags[flag] + ' '
            except:
                pass

        example += extract_binds(docker_info['binds'])                                  # Add mounts to docker run coomand (if necessary)

        return f"{example}{docker_image} {' '.join(commands)}\n```"              # Return example usage string
    except Exception as e:
        print("Error while trying to extract commands from json file")
        print(str(e))

def check_topic(streams):
    for stream in streams:
        if 'rosTopic' in stream:
            return True
    return False


def genreate_sub_table(dir_path, repo_name):
    try:
        json_file = open(os.path.join(dir_path, repo_name, 'nimbusc.json'), 'r')        # Open json file
        json_content = json.load(json_file)
        subs = json_content['streams']['inputStreams']
        sub_table = '# Subscribers\n'
        if not check_topic(subs):
            return sub_table + "This node has no subscribers\n"
        sub_table += "ROS topic | type\n--- | ---\n"
        for sub in subs:
            try:
                topic = sub['rosTopic']['topic']
                type = sub['rosTopic']['type'].split('.')
                type = '/'.join(type[1:])
                sub_table += f'{topic} | {type}\n'
            except:
                pass
        return sub_table
    except Exception as e:
        print('Error while trying to extract subscribers table from json')

def genreate_pub_table(dir_path, repo_name):
    try:
        json_file = open(os.path.join(dir_path, repo_name, 'nimbusc.json'), 'r')        # Open json file
        json_content = json.load(json_file)
        pubs = json_content['streams']['outputStreams']
        pub_table = '# Publishers\n'
        if not check_topic(pubs):
            return pub_table + "This node has no publishers\n"
        pub_table += "ROS topic | type\n--- | ---\n"
        for pub in pubs:
            try:
                topic = pub['rosTopic']['topic']
                type = pub['rosTopic']['type'].split('.')
                type = '/'.join(type[1:])
                pub_table += f'{topic} | {type}\n'
            except:
                pass
        return pub_table
    except Exception as e:
        print('Error while trying to extract subscribers table from json')
        print(str(e))

def check_tf(streams):
    for stream in streams:
        if 'rosTf' in stream:
            return True
    return False

def generate_req_tf(dir_path, repo_name):
    try:
        json_file = open(os.path.join(dir_path, repo_name, 'nimbusc.json'), 'r')        # Open json file
        json_content = json.load(json_file)
        streams = json_content['streams']['inputStreams']
        req_tf = '# Required tf\n'
        if not check_tf(streams):
            return req_tf + "This node does not require tf\n"
        for stream in streams:
            try:
                base_frame = stream['rosTf']['baseFrame']
                child_frame = stream['rosTf']['childFrame']
                req_tf += f'{base_frame}--->{child_frame}\n'
            except:
                pass
        return req_tf
    except Exception as e:
        print('Error while trying to extract provided tf')
        print(str(e))

def generate_provided_tf(dir_path, repo_name):
    try:
        json_file = open(os.path.join(dir_path, repo_name, 'nimbusc.json'), 'r')        # Open json file
        json_content = json.load(json_file)
        streams = json_content['streams']['outputStreams']
        prov_tf = '# Provided tf\n'
        if not check_tf(streams):
            return prov_tf + "This node does not provide tf\n"
        for stream in streams:
            try:
                base_frame = stream['rosTf']['baseFrame']
                child_frame = stream['rosTf']['childFrame']
                prov_tf += f'{base_frame}--->{child_frame}\n'
            except:
                pass
        return prov_tf
    except Exception as e:
        print('Error while trying to extract provided tf')
        print(str(e))



def generate_repo_readme(dir_path, repo_name):
    readme = ""
    readme += generate_headline(repo_name)
    readme += '\n\n'
    readme += generate_img_src(dir_path, repo_name)
    readme += '\n\n'
    # TODO: ROS project page
    readme += generate_dockerhub_image(dir_path, repo_name)
    readme += '\n'
    readme += generate_supported_arch(dir_path, repo_name)
    readme += '\n'
    readme += generate_ros_version(dir_path, repo_name)
    readme += '\n\n'
    readme += generate_description(dir_path, repo_name)
    readme += '\n\n'
    readme += generate_example_usage(dir_path, repo_name)
    readme += '\n\n'
    readme += genreate_sub_table(dir_path, repo_name)
    readme += '\n\n'
    readme += genreate_pub_table(dir_path, repo_name)
    readme += '\n\n'
    readme += generate_req_tf(dir_path, repo_name)
    readme += '\n\n'
    readme += generate_provided_tf(dir_path, repo_name)
    return readme


def generate_library_reademe():
    library_dir_path = os.path.dirname(__file__)
    repos = os.listdir(library_dir_path)
    non_components = ['generate_readme.py','.gitlab-ci.yml','generate_table.py','docker_retag.py','json_retag.py',
                      '.filter_only_updated_items.py','isaac-skeleton-viewer', 'ros2-gateway']

    for element in non_components:
        try:
            repos.remove(element)
        except ValueError:
            pass

    for repo in sorted(repos):
        print(repo)
        repo_dir_path = os.path.join(library_dir_path, repo)
        dirs = os.listdir(repo_dir_path)
        files_to_remove = ['README.md', 'docker', '.gitignore', '.catkin_workspace']

        for file_name in files_to_remove:
            try:
                dirs.remove(file_name)
            except ValueError:
                pass

        with open(os.path.join(repo_dir_path, 'README.md'), 'w') as f:
            readme = ""
            for dir in dirs:
                readme += generate_repo_readme(repo_dir_path, dir)
                readme += '\n\n'
            f.write(readme)

generate_library_reademe()

# library_dir_path = os.path.dirname(__file__)
# repo_dir_path = os.path.join(library_dir_path, 'amcl')
# readme = generate_repo_readme(repo_dir_path, 'amcl')
# print(readme)