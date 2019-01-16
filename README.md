# MIE443
Use the following cmds to get started

```
cd /home/<user or any other desired directory>
git clone https://github.com/ahsueh1996/MIE443.git
git checkout <branch_name>
```

Branches are just parallel code versions that may be developed with different features.
I think we should create a branch to work on everytime we want to try something then use the `master` branch as the 'good copy' of our code.

To create a branch, `cd` to your local copy of the MIE443 folder, then use the following cmds:

```
git checkout master
git pull
git branch <new branch name>
git branch #check that your new branch is highlighted
```

After some changes to the code

```
git status #to  see the files you've made changes to (in red)
git add <file1> <file2> <or entire folders>
git status #to see the files you just added now listed in green
git commit -m "<type some descriptive comments about the changes you've made>"
git push -u origin <your branch name>
```
