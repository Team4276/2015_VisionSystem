# 2015_VisionSystem

How to Build:

If you have not done so already install the latest JDK and NetBeans 7.3.1 or later on a Windows computer

The NetBeans IDE runs on your Windows computer, but the compiling and debugging will be done on the BeagleBone.

The source code resides on the BeagleBone, but is shared by Samba so you can use it as another drive letter from a Windows computer.

To use the checked in project files as-is, map a drive "Q:\" on your Windows machine to the topmost folder ("/") on the BeagleBone
(For example, if you are using the same Ubuntu 12.04 image I'm using, this will map as "\\192.168.1.19\viking")

Start NetBeans, and tell it to use the compilers on the BeagleBone remotely:
--> Tools --> Options
    Select "C/C++" Tab
    Click "Edit..."
    Click "Add..."
    Enter IP address of the BeagleBone, (e.g. 192.168.1.19)
    Click "Next"
    For user enter admin username 
    Click "Next"
    When prompted enter password for admin user
    Click "OK"
    (NetBeans will establish contact with the BeagleBone, after which click "Finish")
    Make sure the combo box next to the "Edit..." button is set to the BeagleBone host
    (Compiler paths like /usr/bin/g++ should now be black (not red) to indicate tools can now be used remotely)
    Click "OK"
    
--> File --> Open Project
    Browse to where Subversion has placed the source code (e.g. "Q:\home\ubuntu\trunk\viking_cv")
    Right-click top of project tree and select "Clean and Build"
    
    

Prepare to run:
    Connect power to Axis Camera, and network cable from Axis camera to same switch as the BbeagleBone and the Windows PC
    (as of this doc update the camera is set to static IP address 192.168.1.20, and the BeagleBone is using DHCP, currently at 192.168.1.19)
    
    

To run:
    In NetBeans, Ctrl-F5 starts the program using gdb as a remote debugger.
    On the Windows machine open a Chrome browser (IE won't work) and go to port 7777 on the BeagleBone (e.g. 192.168.1.19:7777)
    (Should see the annotated reduced-size frames from the camera)
    
    
