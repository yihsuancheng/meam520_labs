# MEAM5200 Frequently Asked Questions
---
# Lab 0

### Error with optimizing VMWare Player 17.0

It's come to our attention that some of you are having issues of accessing VM settings on VMware Workstation Player 17. You might follow the instructions below to fix this issue.

- Solution 1: Modifying the configuration file directly.

1. Make sure you close the VMware app, then open the folder where the MEAM5200 virtual machine is located.
2. In that folder, For Linux, locate one file with the extension .vmx; For Windows, locate one file with type "VMware virtual machine configuration"
3. Open it with any text editor or IDE, for example, NotePad on Windows.
4. If the value for virtualhw.version is not "19" , please change it to "19" and save the file.
5. Reopen VMware software, try to edit settings to see if it works.
If your problem is solved, you don't need to modify the configuration file anymore. Otherwise, you can edit number of cores and storage size directly in the vmx file: [ref](https://kb.vmware.com/s/article/205n).

- Solution 2: You may want to install VMWare Player 16.0 instead.

### Atom Editor does not Save Python code

- Solution 1: 
1. Inside your virtual machine, open up the terminal (make sure you close Atom)
2. Enter this command `cd ~/.local/share`
3. Then enter the following command `rm -rf keyrings`
4. Recreate `keyrings` folder by typing this command `mkdir keyrings`
5. Open up Atom and now you'll see a prompt asking for password, simply type any password and press enter
6. Now you should be able to edit and save files.
   
- Solution 2: Try using some other editor like vscode. You can always use 'nano' in the terminal if nothing else works.


### No module named 'core'

Please make sure the folder name is **meam520_labs** exactly, not "meam5200_labs" or "meam5200-labs"
