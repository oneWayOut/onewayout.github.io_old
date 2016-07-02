---
layout: post
title:  "How to customize Pixhawk in your own project"
date:   2016-05-24 20:52:16 +0200
categories: pixhawk
---

> 本文是在ISAE Supaero实习时，应老师要求所写。本来老师要求我对Pixhawk的理解写成文档或PPT,　因刚好在折腾博客，就放在这里了。
> Pixhawk is an open-source autopilot platform. In this article, I explained the basic architecture of Pixhawk source code.
> And how to customize it in your own project.

* [0. Prerequisite](#0)

* [1. Understand Pixhawk source code](#1)

  + [1.1 Install the toolchain and build the code](#1.1)

  + [1.2 How are the source code directories organized](#1.2)

  + [1.3 The boot process](#1.3)

  + [1.4 The startup scripts](#1.4)

  + [1.5 The architecture](#1.5)

* [2. How to costumize](#2)

  + [2.1 A small tutorial](#2.1)

  + [2.2 Add you own controller](#2.2)

  + [2.3 Change the mixer](#2.3)

  + [2.4 Change the makefile and the startup script](#2.4)

* [3. Try this example](#3)

  + [3.1 How to Try it](#3.1)

  + [3.2 What I have changed](#3.2)

<h2 id="0">0. Prerequisite</h2>

It's recommended to use **Ubuntu 14.04 LTS**, otherwise you may have strange issues.

Please get familiar with GIT, it's a very powerfull software version control tool.　You can install the GUI tool `git cola` (In terminal: apt-get install git-cola) if you are not comfortable with the git commands in terminal.

SublimeText 3 is a convenient editor to navigate the numerous source files. There is already [a project file](https://github.com/PX4/Firmware/blob/master/Firmware.sublime-project) in the source folder that you can import to SublimeText. One feature that I used every day is that: Press "Ctrl + p" and type in the filename, and you can find the file you want instantly. 

<h2 id="1">1. Understand Pixhawk source code</h2>

<h3 id="1.1">1.1 Install the toolchain and build the code </h3>

First you have to install the toolchain by following the steps in [this webpage](http://dev.px4.io/starting-installing-linux.html). And then you can build the code and flash it to your PX4 board as [this page](http://dev.px4.io/starting-building.html).

<h3 id="1.2">1.2 How are the source code directories organised </h3>

After you have cloned the source code repository, you might be scared by so many directories and files. Actually you don't need to know all of them. I list the directory tree below(only the very import directories),　and explain what are in the folders.

{% highlight sh %}
Firmware 
├── cmake  
├── msg 
│   └── templates 
│       ├── px4 
│       └── uorb 
├── ROMFS 
│   ├── px4fmu_common 
│   │   ├── init.d 
│   │   ├── logging 
│   │   └── mixers 
│   └── px4fmu_test 
│       ├── init.d 
│       ├── mixers 
│       └── unit_test_data 
└── src 
    ├── drivers 
    ├── examples 
    ├── modules 
    │   ├── attitude_estimator_ekf 
    │   ├── attitude_estimator_q 
    │   ├── bottle_drop 
    │   ├── commander 
    │   ├── controllib_test 
    │   ├── dataman 
    │   ├── ekf2 
    │   ├── ekf2_replay 
    │   ├── ekf_att_pos_estimator 
    │   ├── fw_att_control 
    │   ├── fw_pos_control_l1 
    │   ├── gpio_led 
    │   ├── land_detector 
    │   ├── local_position_estimator 
    │   ├── mavlink 
    │   ├── mc_att_control 
    │   ├── mc_att_control_multiplatform 
    │   ├── mc_pos_control 
    │   ├── mc_pos_control_multiplatform 
    │   ├── muorb 
    │   ├── navigator 
    │   ├── param 
    │   ├── position_estimator_inav 
    │   ├── px4iofirmware 
    │   ├── sdlog2 
    │   ├── segway 
    │   ├── sensors 
    │   ├── simulator 
    │   ├── systemlib 
    │   ├── uavcan 
    │   ├── unit_test 
    │   ├── uORB 
    │   └── vtol_att_control 
    └── systemcmds 
        ├── bl_update 
        ├── config 
        ├── dumpfile 
        ├── esc_calib 
        ├── i2c 
        ├── mixer 
        ├── motor_test 
        ├── mtd 
        ├── nshterm 
        ├── param 
        ├── perf 
        ├── pwm 
        ├── reboot 
        ├── reflect 
        ├── tests 
        ├── top 
        ├── topic_listener 
        ├── usb_connected 
        └── ver 
{% endhighlight %}


    
|---
| Folder | Description 
|-|:-
| cmake | make files
| msg | uORB msg template, the uORB msg headers are generated from this folder 
| ROMFS | startup scripts and mixer files 
|---
| src | drivers, examples, flight control tasks
| src/drivers | all the drivers: gps, gyro, pwm...
| src/examples | some simple examples help you understand the code
| src/modules | estimators, controllers ....
| src/systemcmds | some handy commands can be used in Nuttx shell

As you can see, the source code files are well organised. Though you still need time to get familiar with them.

<h3 id="1.3">1.3 The boot process</h3>
If you power on your Pixhawk board through USB cable or BEC, the LED will flash and the buzzer will play a special tune(you can check the tune meaning [here](http://ardupilot.org/copter/docs/common-sounds-pixhawkpx4.html)). But you may wander what exactly happen during this process. 
<center><img src="/assets/img/pixhawk/pixhawk_board.png" alt="pixhawk_board" width="219" height="337" /></center>

When powering on the board, the bootloader will run first. Bootloader is like BIOS in your PC. And it's already in the board when you buy it. So you may never need to bother it. The bootloader will launch the Nuttx Operating System. After some initialization of the hardware, memory... the Nuttx will execute a script file called **"init.d/rcS"** in function `nsh_initscript()` of file `nsh_script.c`. You can check the script file folder in [section 1.2](#1.2)). This is a **very important** step. By executing this script file, some parameters in the EEPROM will be read, and the corresponding tasks related to these parameters will be launched. I will explain this in the next section.

<h3 id="1.4">1.4 The startup scripts</h3>

Nuttx is like a simplified Linux, moreover it's real time. So the script file is quite the same as Bash script in Linux. You can easily understand it if you know linux well.

Let's read some lines in `ROMFS/px4fmu_common/init.d/rcS` first.

{% highlight sh %}
......
#
# Try to mount the microSD card.
#
# REBOOTWORK this needs to start after the flight control loop
if mount -t vfat /dev/mmcsd0 /fs/microsd
then
	echo "[i] microSD mounted: /fs/microsd"
	# Start playing the startup tune
	tone_alarm start
else
	tone_alarm MBAGP
	if mkfatfs /dev/mmcsd0
	then
		if mount -t vfat /dev/mmcsd0 /fs/microsd
		then
			echo "[i] microSD card formatted"
		else
			echo "[i] format failed"
			tone_alarm MNBG
			set LOG_FILE /dev/null
		fi
	else
		set LOG_FILE /dev/null
	fi
fi
......
{% endhighlight %}

At first it will start serial driver and set some parameters, which is not interested by us and not listed here. Then you will see `if mount -t vfat /dev/mmcsd0 /fs/microsd`. `mount` is a built-in command supported by Nuttx (Linux has the same command). It will try to mount the microSD card. If the return value is true, which means microSD card is mounted successfully,  the `echo` will print the result in shell window. And you should hear the buzzer alarm by `tone_alarm start`.

There are many similar statements like `tone_alarm start`. If you understand this, you will almost know how the script file works and how to modify it to satisfy your own needs. Well, the grammar is simple: `command -arguments`, just like the commands in Linux Terminal. `tone_alrm` is a command compiled from file `tone_alarm.cpp` by some tricks in makefile. If you scrutinise the function `tone_alarm_main()` in this file, you will find this command has arguments `start` and `stop`.

Let's read another piece of code in this file to see if you have any clue.

{% highlight sh %}
......
if [ $MODE == autostart ]
then
	#
	# Start the ORB (first app to start)
	#
	uorb start

	#
	# Load parameters
	#
	set PARAM_FILE /fs/microsd/params
	if mtd start
	then
		set PARAM_FILE /fs/mtd_params
	fi

	param select $PARAM_FILE
	if param load
	then
		echo "[param] Loaded: $PARAM_FILE"
	else
		echo "[param] FAILED loading $PARAM_FILE"
		if param reset
		then
		fi
	fi
......
{% endhighlight %}

You can find the commands `uorb, mtd, param` in files: `uORBMain.cpp, mtd.c, param.c`. You don't need to go deep inside these files right now, anyway we may use this method to scrutinise other files later (For instance: flight control files). This piece of code will just start uORB to provide communication service, and load parameter file `mtd_params` which contains airframe configuration, PID parameters, etc. 

I assume you have read through this file. So to summarize, the startup scripts are very important in the boot process. I list the things happened in this process to our concern:

  * Read the parameter file
  * Start the sensor drivers (script `rc.sensors`)
  * Set and load the mixer corresponding to the airframe parameter `SYS_AUTOSTART`, set the pwm channel (script `rc.autostart`, this file is generated after you build the code) 
  * Start the flight tasks corresponding to the airframe parameter `SYS_AUTOSTART` (script `rc.fw_apps`, `rc.mc_apps`, etc.)

<h3 id="1.5">1.5 The architecture</h3>

All the flight control tasks run in Nuttx system. They communicate with each other through uORB. uORB is a implementation of publish-subscribe pattern.

<center><img src="/assets/img/pixhawk/pixhawk_arch.png" alt="pixhawk_arch" /></center>

<br>
To control a vehicle, you need to navigate to waypoints, estimate the position and attitude, and control the position and attitude by using feedback control theory. That's the idea in pixhawk flight control architecture. 
<center><img src="/assets/img/pixhawk/pixhawk_feedbackcontrol.png" alt="pixhawk_arch" /></center>  

These flight control modules are in folder `Firmware/src/modules`. I list the  modules used by different airframes below. Actually you can find where they are launched in the startup scripts.
 
|---
|   | Fixed Wing | Multi Copter | VTOL 
|-|:-|:-|:-
| Navigator | navigator | navigator | navigator
| Estimator | ekf_att_pos_estimator | attitude_estimator_q <br> position_estimator_inav | attitude_estimator_q <br> position_estimator_inav
| Controller | fw_att_control <br> fw_pow_control_l1 | mc_att_control <br> mc_pos_control | vtol_att_control <br> mc_att_control <br> mc_pos_control <br> fw_att_control <br> fw_pow_control_l1 

<br>
The architectural overview could be seen [here](http://dev.px4.io/concept-architecture.html). The tasks communicate by publishing and subscribing uORB messages. For instance, the messages related to the module `attitude_estimator_q` are as follows:

* Published messages:
    - vehicle_attitude
    - control_state
* Subscribed messages:
    - sensor_combined 
    - vision_position_estimate 
    - att_pos_mocap 
    - airspeed 
    - parameter_update 
    - vehicle_global_position

You can check the meaning of these messages in folder `Firmware/msg/templates`.

<h2 id="2">2. How to costumize</h2>

<h3 id="2.1"> 2.1 A small tutorial </h3>

We can do a small exercise to understand the code better, and then go even further. Please follow [this tutorial](http://dev.px4.io/tutorial-hello-sky.html#step-2-minimal-application). The FTDI 3.3v cable is **a necessary hardware** for developpers to interact with Nuttx through **Nuttx Shell(NSH)**. 

<h3 id="2.2"> 2.2 Add you own controller </h3>

Following the same concept, you can add a simple control law in [`Firmware/src/examples/fixedwing_control/main.c`](https://github.com/PX4/Firmware/tree/master/src/examples/fixedwing_control). The program subscribes the estimated position and attitude, manual control input. The only thing you need to do is to implement the PID control law, and calculate the **normalized control value**. Then publish it in the `actuator_controls_0` message to mixer to control the servos or motors.

<h3 id="2.3"> 2.3 Change the mixer </h3>

The key concept of the mixer is to translate the normalized control output from the controller to pwm to actuators, Which greatly improves reusability of code. If you have a special airframe, you may need to have you own mixer to control the acuators. Please refer to [this webpage](http://dev.px4.io/concept-mixing.html) and the source file folder `Firmware/ROMFS/mixers`.

<h3 id="2.4"> 2.4 Change the makefile and the startup script</h3>

You can add you own program in `Firmware/cmake/configs/nuttx_px4fmu-v2_default.cmake` as you did before in [section 2.1](#2.1), so as to compile it. But you still need to launch it in Nuttx shell. An alternative and simple way is to start it in startup script like below (Take fixed wing airframe for instance):
{% highlight sh %}
......
#
# Start attitude controller
#
# fw_att_control start
# fw_pos_control_l1 start

ex_fixedwing_control start
......
{% endhighlight %}
This piece of code is from `rc.fw_apps`. As you can see, two lines are commented, one line is inserted. So your own controller `ex_fixedwing_control` from [section 2.2](#2.2) is started instead of the original one: `fw_att_control` and `fw_pos_control_l1`.

<h2 id="3">3. Try this example </h2>

I have modified some Pixhawk code for my internship project. So you can try my code and use the same method to customize pixhawk in your own project too.

In my example, the `MAIN OUT` channel 1 and 2 can be controlled by the roll angle. And the `MAIN OUT` channel 3, 4, 5 and 6 are controlled by vertical speed.

<h3 id="3.1">3.1 How to Try it</h3>

In terminal, go to the folder where you want to store the source code, type in the following commands:
{% highlight sh %}
# clone the repository
git clone https://github.com/oneWayOut/Firmware.git
cd Firmware
git checkout caidev
git submodule update --init --recursive

# please connect pixhawk to your computer through USB cable
make px4fmu-v2_default upload
{% endhighlight %}

After you have execute the commands above, connect the servo and motor to the `MAIN OUT` channels. Connect a FTDI 3.3v cable to pixhawk as explained [here](http://dev.px4.io/advanced-system-console.html#connecting-via-ftdi-33v-cable). I assume you have installed `screen`. So you can connect to Nuttx shell by the command below(change `/dev/ttyXXX` to your own device name, something like `/dev/ttyUSB0`).
{% highlight sh %}
screen /dev/ttyXXX 57600 8N1
{% endhighlight %}

And start my customized task in Nuttx shell:
{% highlight sh %}
nsh> ex_visionair_control start
{% endhighlight %}

Press the safety button to arm the board, now you can observe how the servo and motor react when you change the roll angle and the vertical speed of Pixhawk board. 

<h3 id="3.2">3.2 What I have changed </h3>

As I said before, I changed the startup scripts, makefile. And I renamed the folder `fixedwing_control` to `visionair_control`. In the file `main.c` from this folder, I connect the sensor value  directly to pwm channel, but not using mixer.  You can see what I have changed [here](https://github.com/PX4/Firmware/compare/master...oneWayOut:caidev).
