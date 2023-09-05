import os
import subprocess

CONTAINING_DIR = 'components'

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
        table = f"# Components Table For ROS {get_git_branch()}\n"
        table += "Image | Link\n--- | ---\n"
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
        with open(os.path.join(library_dir_path, 'instructions.md')) as instructions:
            f.write(instructions.read())

generate_table()