---
layout: post
title:  "How to customize Pixhawk in your own project"
date:   2016-05-24 20:52:16 +0200
categories: myblog
---

>
> In this article, I will explain the basic architecture of Pixhawk source code.
> And then how to customize it in your own project.

* [0. Prequisite](#0)

* [1. Understand Pixhawk source code](#1)

  + [1.1 Install the toolchain and build the code](#1.1)

  + [1.2 How are the source code directoris organized](#1.2)

  + [1.3 The boot process](#1.3)

  + [1.4 The startup scripts](#1.4)

  + [1.5 The architecture](#1.5)

* [2. How to costumize](#2)

  + [2.1 Add you own controller](#2.1)

  + [2.2 Change the makefile](#2.2)

  + [2.3 Change the mixer(To be continued...)](#2.3)

* [3. Try this example](#3)


<h2 id="0">0. Prequisite</h2>

It's recommended to use **Ubuntu 14.04 LTS**, otherwise you may have strange issues.

Please get familiar with GIT, it's a very powerfull software version control tool.　You can install git cola (In terminal: apt-get install git-cola) if you are not comfortable with the git commands.

SublimeText3 is a convinient editor to navigate the numourous source files. There is already [a project file](https://github.com/PX4/Firmware/blob/master/Firmware.sublime-project) in the source files that you can import to SublimeText. One feature that I used every day is that: Press "Ctrl + p" and type in the filename, and you can find the file you want quickly. 

<h2 id="1">1. Understand Pixhawk source code</h2>

<h3 id="1.1">1.1 Install the toolchain and build the code </h3>

First you have to install the toolchain by following the steps in [this webpage](http://dev.px4.io/starting-installing-linux.html). And then you can build the code and flash it to your PX4 board as [this page](http://dev.px4.io/starting-building.html).

<h3 id="1.2">1.2 How are the source code directoris organized </h3>

After you have cloned the source code repository, you might be scared by so many directories and files. Actually you don't need to know all of them. I list the directory tree below(only the very import directories),　and explain what's in the folders.

{% highlight sh %}
Firmware 
├── cmake  
│   ├── cmake_hexagon 
│   │   └── toolchain 
│   ├── common 
│   ├── configs 
│   ├── nuttx 
│   ├── posix 
│   ├── qurt 
│   ├── scripts 
│   ├── templates 
│   ├── test 
│   └── toolchains 
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
| cmake | make file. we will change **nuttx_px4fmu-v2_default.cmake** later
| msg | uORB msg template, the uORB msg headers are generated from this foler 
| ROMFS | startup scrips and mixer files 
|---
| src | drivers, examples, flight control tasks
| src/drivers | all the drivers: gps, gyro, pwm...
| src/examples | some simple examples help you understand the code
| src/modules | estimators, controllers ....
| src/systemcmds | some handy commands can be used in Nuttx shell

As you can see, the source code files are well organized. Nevertheless, you still need time to get familiar with them.

<h3 id="1.3">1.3 The boot process</h3>
If you power on your Pixhawk board through USB calbe or BEC, the LED will flash and the buzzer will play a special tune(you can check the tune meaning [here](http://ardupilot.org/copter/docs/common-sounds-pixhawkpx4.html)). But you may wander what exactly happen during this process. 
<center><img src="/assets/img/pixhawk/pixhawk_board.png" alt="pixhawk_board" width="219" height="337" /></center>

When powering on the board, the bootloader will run first. Bootloader is like BIOS in your PC. And it's already in the board when you buy it. So you may never need to bother it. The bootloader will launch the Nuttx Operating System. After some initialization of the hardware, memory... the Nuttx will execute a script file called **"init.d/rcS"** in fuction `nsh_initscript()` of file `nsh_script.c`. You can check the script file folder in [section 1.2](#1.2)). This is a **very import** step. By executing this script file, some parameters in the EEPROM will be read, and the corresponding tasks related to these paraameters will be lanuched. I will explain this in the next section.

<h3 id="1.4">1.4 The startup scripts</h3>

Nuttx is like a simplified Linux, but it's real time. So the script file is quite the same as Bash script in Linux. You can easily understand it if you know linux well.

Let's read some lines in `ROMFS/px4fmu_common/init.d/rcS` first.

{% highlight sh %}
...........
..........
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
.....
.....
{% endhighlight %}

At first it will start serial driver and set some parameters, which is not interested by us and not listed here. Then you will see `if mount -t vfat /dev/mmcsd0 /fs/microsd`. `mount` is a built in command suportted by Nuttx (Linux has the same command). It will try to mount the microSD card. If the return value is true, which means microSD card is mounted successfully,  the `echo` will prompt the result. And you should heard the buzzer alarm by `tone_alarm start`.

There are many similar statments like `tone_alarm start`. If you understand this, you will almost know how the script file works and how to modify it to satisfy your own needs. Well, the grammar is simple: `command -arguments`, just like the commands in Linux Terminal. `tone_alrm` is a command compiled from file `tone_alarm.cpp` by some tricks in makefile. If you scrutinise the function `tone_alarm_main()` in this file, you will find this command has arguments `start` and `stop`.

Let's read another piece of code in this file.

{% highlight sh %}
...........
..........
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
.....
.....
{% endhighlight %}

You can find the commands `uorb, mtd, param` in files: `uORBMain.cpp, mtd.c, param.c`. You don't need to go deep inside these files. This piece of code will just start UORB to provide communication service, and load parameter file `mtd_params` which contains airframe configuration, PID parameters, etc. 





<h3 id="1.1">1.3强调</h3>

这是第一段第三节

这是第一段第三节

这是第一段第三节

这是第一段第三节

这是第一段第三节




