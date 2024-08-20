# VSCode support

The repository includes json configuration files for VSCode and Docker cross-builds.

This will allow you to deploy to a Raspberry Pi target easily, as well as debug the app remotely from VSCode

## Tasks

The configuration files provide custom tasks to handle the following:
- make clean
- make all
- Create and Deploy SSH Public Key ID for accessing Target RPi
- Deploy (copy) compiled executable to Target RPi
- Launch GDB Server on Target RPi for the executable

In addition, the VSCode Debug Configuration is set up for remote debugging of the executable on the Target RPi via .vscode/launch.json

## Customization

There are two sample files found in their respective directories:
.devcontainer
.vscode

You will need to duplicate the *.sample files to new files without the *.sample extension, and modify the values marked as 'xxxx' to match your configuration.


