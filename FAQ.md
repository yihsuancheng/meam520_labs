# MEAM5200 Frequently Asked Questions
---
# Lab 0

### Error with optimizing VMWare Player 17.0

- Solution 1: Some students were experiencing this issue with VMWare Player 17.0. You may want to install VMWare Player 16.0 instead.
- Solution 2: You can modify the configuration file directly. You need to first find where the virtual machine is installed (build a new virtual machine and find the default path). 
Then find the vmx (configuration) file. Open the file with a text editor or IDE. Once you open it, you can modify the configuration file easily and corresponding field is obvious by name. 
If your problem is solved, you don't need to modify the configuration file anymore. Otherwise, you can edit number of cores and storage size directly in the vmx file: [ref](https://kb.vmware.com/s/article/205n).

### Atom Editor does not Save Python code

- Solution 1: 
1. Inside your virtual machine, open up the terminal (make sure you close Atom)
2. Enter this command `cd ~/.local/share`
3. Then enter the following command `rm -rf keyrings`
4. Recreate `keyrings` folder by typing this command `mkdir keyrings`
5. Open up Atom and now you'll see a prompt asking for password, simply type any password and press enter
6. Now you should be able to edit and save files.
   
- Solution 2: Try using some other editor like vscode. You can always use 'nano' in the terminal if nothing else works.
