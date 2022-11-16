from optparse import OptionParser
import sys
import os
import shutil
import re
import subprocess
from distutils.dir_util import copy_tree



def main():
    parser = OptionParser()
    parser.add_option('--dir-name', dest='dir_name', action='store', default='/tmp/nimbus-library')

    (options, args) = parser.parse_args()
    if args:
        parser.error("No arguments are permitted")
    proc = subprocess.Popen(['git', 'diff', '--name-only', 'HEAD', 'HEAD~1'], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    output, error = proc.communicate()
    output, error = output.decode('ascii'), error.decode('ascii')
    print('getting list of all modified files:')
    print(output)
    changed_files = output.split(os.linesep)
    changed_files = list(
        map(
            lambda f: f.strip(),
            changed_files
        )
    )
    absolute_paths = list(
        map(
            lambda path: os.path.abspath(path),
            changed_files
        )
    )
    ignore_paths = []
    for i in range(len(absolute_paths)):
        file_path = absolute_paths[i]
        match = re.search(f'{os.getcwd()}/[a-zA-Z0-9\-]+/', file_path)
        if match is not None:
            file_path = file_path[:match.end()]
        else:
            ignore_paths.append(file_path)
            continue
        absolute_paths[i] = file_path
    # remove files that should be ignored
    for f in ignore_paths:
        try:
            absolute_paths.remove(f)
        except:
            pass
    # remove duplicates
    absolute_paths = list(dict.fromkeys(absolute_paths))
    print('all items to update:')
    # remove non-existing paths
    absolute_paths = list(
        filter(
            lambda path: os.path.isfile(os.path.join(path, 'nimbusd.json')),
            absolute_paths
        )
    )

    for p in absolute_paths:
        print(p)
    print()
    print(f'-------------saving items in {options.dir_name} directory----------------')
    # create tmp directory
    directory_name = f'{options.dir_name}'
    if os.path.exists(directory_name):
        shutil.rmtree(directory_name)

    os.mkdir(directory_name)

    for p in absolute_paths:
        device_name = [x for x in p.split(os.path.sep) if x][-1]
        print(f'saving device {device_name} in {directory_name}/{device_name}')
        copy_tree(p, f'{directory_name}/{device_name}')


if __name__ == '__main__':
    main()
