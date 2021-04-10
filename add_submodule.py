import os
import re
a = os.listdir()
pattern = "https[^\s]*"
print(a)
def add_submodule(current_dir):
    dirs = os.listdir(current_dir)
    if any(".git" in dirname for dirname in dirs):
        print("found a git submodule in ",current_dir)
        owd = os.getcwd()
        os.chdir(current_dir)        
        remote = os.popen("git remote -v").readline()
        url = re.search(pattern, remote).group()
        os.chdir(owd)
        a = "git submodule add "+url+" "+current_dir
        # print(a)
        os.system(a)
        return
    dirs = filter(lambda name: not name.startswith("."),dirs)
    dirs = map(lambda dirname: os.path.join(current_dir,dirname),dirs)
    dirs = filter(lambda path: os.path.isdir(path),dirs)
    for dirname in dirs:
        print(dirname)
        add_submodule(dirname)
add_submodule("src")