import os

def get_repo_url(repo_name):
    return f"[{repo_name}]({repo_name})"

# This method generates a link to the repo image
def generate_img_src(dir_path, repo_name, dir_name):
    try:
        files = os.listdir(os.path.join(dir_path, repo_name, dir_name))                                      # list the files at the repo nimbus directory
        files.remove('nimbusc.json')                                                                    # Remove the nimbus json file from the list (folder contains a json and an image)
        img_name = files[0]                                                                             # Extract the image's name
        return f"<img src=\"./{repo_name}/{dir_name}/{img_name}\" alt=\"{dir_name}\" width=\"40\"/>"   # Return the link to the image
    except Exception as e:
        print('Error while trying to generate image link')
        print(str(e))

def generate_table():
    library_dir_path = os.path.dirname(__file__)
    repos = os.listdir(library_dir_path)

    repos.remove('generate_readme.py')
    repos.remove('generate_table.py')
    # repos.remove('docker_retag.py')
    repos.remove('README.md')
    repos.remove('.git')
    repos.remove('.gitignore')
    repos.remove('.gitlab-ci.yml')
    repos.remove('.filter_only_updated_items.py')  # Unknown

    f = open(os.path.join(library_dir_path, 'README.md'), 'w')
    table = "Image | Link\n--- | ---\n"
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
        
        for dir in dirs:
            table += f'{generate_img_src(library_dir_path, repo, dir)} | {get_repo_url(repo)}\n'
        
    f.write(table)
    
generate_table()
