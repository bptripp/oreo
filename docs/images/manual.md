# OREO Manual 
This is a work-in-progress manual that covers setup and basic use of the OREO robot. Unfortunately, it is based on notes that were taken after the robot was complete, during the process of setting up the robot on a new computer. So it is missing some procedures that were performed during the initial robot setup. We will expand it the next time the robot is configured from scratch. 

## System Overview
The head has seven degrees of freedom: yaw, pitch, and roll in the neck, and yaw and pitch in each eye. Position in each degree of freedom is sensed by a rotary encoder. The neck is moved by rotary motors. The controllers for these motors are in the base. The eyes are moved by linear actuators, and their controller is on top of the head. Calibration is needed to map the linear actuator positions to the eye angles. 

The main hardware components of the system are: 
* 7-DOF gimbal (Consists of motors, encoders, controllers, frame, linkages, and cabling. Manufacture and assembly are not covered here.)
* Cameras (Point Grey Flea3 FL3-U3-13S2C-CS)
* Fixed lenses (Edmund Optics 58-000 - 8.5mm Compact Fixed Focal Length Lens)
* Variable-focus lenses (-600 to 160FL, VIS, Optotune Industrial Focus-Tunable Lens) 
* Ethernet switch
* A host computer with a Windows OS. 

The following software is needed on the host computer: 
* FLIR Spinnaker (the camera SDK)
* Matlab
* Custom Matlab scripts (in the matlab repository folder)
* Technosoft EasyMotion Studio
* MinGW (includes gcc)
* Optotune software

### Motor Controllers 
Review the manuals (available online) for EasyMotion studio, language, and setup. Searching with google site:technosoft.com is more effective than help.

Note: 
* EasyMotion Studio has a command interpreter that you can use to send commands over Ethernet to controllers. 
* EasyMotion Studio has a Binary Code Viewer that shows binary versions of commands (these are the . This is what must be send from Matlab etc. to get values and set controller to run code at a certain address. 

The linear actuators have Hall encoders. These are incremental (relative to position at startup). Feedback control of eye positions runs on the controller cards and is based on the linear Hall encoders only (not the rotary encoders). 

The neck uses rotary encoders for PID feedback. 

## Setup
### Software Installation
Install Technosoft EasyMotion Studio by running ESM_installer.exe. The download at technosoftmotion.com is a demo that doesn’t support Ethernet communication. You must register to get a functional copy. 

Copy and unzip “EMS Projects/Eye Drive May 16 2017” and “EMS Projects/Eye Drive May 16 2017” to Program Files  (x86) / Technosoft  /ESM / Projects

Install Matlab. 

Try to run calibration.m. This may prompt installation of a Matlab-MinGW adapter, which you can do from the AddOns menu. If this process does not result in MinGW being installed properly, install MinGW manually (see below) and try again. 

If MinGW didn’t install from Matlab, install 64-bit MinGW (includes gcc; make sure gcc stuff included) on host computer. There are multiple versions of MinGW on the internet and not all of them will work. This is the one: https://sourceforge.net/projects/tdm-gcc/files/TDM-GCC%20Installer/Previous/1.1309.0/tdm64-gcc-4.9.2.exe/download Add C:\MinGW\bin to Path environment variable (in Advanced System Settings from right-click-Start->System). 

Build DLL using command in README file if Matlab hasn’t done it. 

Install FLIR Spinnaker for the cameras. This should be independent of the above steps, but doing it first seems to prevent EasyMotion studio from installing properly. This is a download from the Point Grey web site. Select “Application Development” during the installation and install the default components. Launch SpinView, plug in the cameras, and see if they work. 

### Hardware
Clamp robot to a stable base. 
Connect both controllers (top of head and within base) to Ethernet switch, connect computer Ethernet to switch, set subnet of that Ethernet card to 192.168.2, IP to 192.168.2.2, default gateway 192.168.2.1.
Open EasyMotion studio, choose axis, communication->setup, make the port right. It should say “online” in green at bottom of EasyMotion window. If not, try pinging control boards (eye controller is 192.168.2.14 and neck controller is 192.168.2.15). 
![Alt text](docs/images/em-comm-setup.png?raw=true "EasyMotion Studio communication setup window")

Check communication with command interpreter, e.g. “?apos2” should return encoder position for axis 2. 

## Procedures 
### Startup 
Move robot to neutral position. 
Make sure wires are not hung up on anything. 
Run calibration.m. 
You can now run other code. 

### Calibration
Calibration maps desired eye angles to corresponding linear encoder positions. This is used to give feedforward commands. 

Run calibration.m every time robot starts up. This moves the eye actuators through their range of motion while reading encoder values, and then performs a least-squares fit of the motor-angle model parameters. This must be done every time power is cycled, because there is no absolute reference for the motor positions; they are relative to their positions at startup. First make sure eyes are roughly centred to begin (move and hold the eyes manually by holding the cameras’ USB3 cables). Note this procedure will stop if current is unexpectedly high (which is conservative, in case a wire gets hung up). 

### Test Encoders 
In EMS, drive setup. Select test connections. 

### Shutdown
Hold robot by the eyebrows so it doesn’t fall when neck pitch torque turns off. Turn the power off. 

### Setting Max Speed
TODO 

### Controller Tuning on Drives
TODO 
Note: this procedure can reset the drives and move the robot quickly, so have hand on power and be ready to catch chin. 

### Network
Everything is on 2 subnet. Eye controller is 192.168.2.14 and neck is 192.168.2.15. Eye host ID is 120, and neck is 121 (these are Ethernet to canbus adapters; called Axis ID in UI). These are used in communication setup in EasyMotion studio. Each axis has an axis ID additionally. In the neck, 1, 2, 3 are yaw, pitch, roll. Ordered from right to left motor for eyes. One eye encoder is plugged into each drive, in order yaw, pitch, pitch, yaw. So if you want to read from right eye yaw encoder, you must talk to axis one. 

### Updating Parameters on Control Cards
The three neck axes have very different controller gains. Particularly the pitch gain is much higher. 
To reset, open neck project in EasyMotion Studio. Select an axis, select large Edit icon in setup screen, check parameters (use Motor Setup and Drive Setup buttons to get to all the parameters), click OK. 
To program controller, click “Download to drive/motor” for each axis to get control params on controller, also go into “motion” and click run to get code on controller. Axis number is under “application general information”. Axis numbers can be set programmatically but are currently set to “use hardware”. This means they are configured with jumpers on the card (see card manual). These are currently set so the axes are in order starting at the Ethernet-CAN adapter side. 
