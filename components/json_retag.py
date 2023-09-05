import os
import json 
import subprocess

def get_git_branch():
    try:
        branch_name = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"], 
            universal_newlines=True
        ).strip()
        return branch_name
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while trying to get the branch name: {e}")
        return None
    except FileNotFoundError as e:
        print(f"Git is not installed or the script is not in a git repository: {e}")
        return None

def json_retag():
    workdir = os.path.dirname(os.path.abspath(__file__))
    repos = os.listdir(workdir)
    non_components = ['docker_retag.py', 'generate_readme.py','generate_table.py','json_retag.py','README.md','.filter_only_updated_items.py',
                      '.gitignore','.gitlab-ci.yml','.git', 'ros2-gateway']
    ros_version = get_git_branch()

    for element in non_components:
        try:
            repos.remove(element)
        except ValueError:
            pass

    for repo in repos:
        print(repo)
        repo_path = os.path.join(workdir, repo)
        nimbus_dir = os.listdir(repo_path)
        
        files_to_remove = ['docker', 'README.md', '.catkin_workspace', '.gitignore']
        
        for file_name in files_to_remove:
            try:
                nimbus_dir.remove(file_name)
            except ValueError:
                pass

        for dir in nimbus_dir:
            json_file_path = os.path.join(repo_path, dir, 'nimbusc.json')

            with open(json_file_path, 'r') as json_file:
                json_content = json.load(json_file)

            docker_image = json_content['environment']['dockerInfo']['image'].split(':')[0] # Extract the docker image name from the json file   
            json_content['environment']['dockerInfo']['image'] = f"{docker_image}:{ros_version}"

            with open(f'{json_file_path}', 'w') as json_file:
                json.dump(json_content, json_file, indent=4)


json_retag()