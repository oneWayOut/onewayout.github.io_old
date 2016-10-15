---
layout: post
title:  "Pixhawk多旋翼姿态控制算法解析"
date:   2016-07-02 10:43:16 +0000
categories: essay
---

   <script type="text/javascript"
  src="http://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML">
  </script>

  <script type="text/x-mathjax-config">
  MathJax.Hub.Config({
    TeX: { equationNumbers: { autoNumber: "AMS" } }
  });
  </script>



网上有些关于Pixhawk姿态控制的文章, 写的都不是很透彻清晰. 因为其姿态控制算法中计算各姿态轴误差的部分代码看起来很是晦涩，很难在教科书或网上找到相关资料，纯粹是开发者自己的经验想法，我所能找到的中文资料基本上没有讲清这一点. 最近在研究Pixhawk多旋翼飞控，现已基本明白其姿态控制原理, 在本文中力图简明地阐述我的理解. 


我在之前的[这篇文章](http://onewayout.github.io/pixhawk/2016/05/24/pixhawk-principle-and-customize.html#1.5)已对Pixhawk总体架构做了一个概括. 理解了代码架构、控制率，想要利用pixhawk强大的软硬件平台来扩展做自己想要的东西就相当简单了. 大部分个性化的定制应该在控制率、仿真上面. 比如我们新设计了一个异型翼面，控制通道与现有飞机不同的飞机，仅需要在原有控制率上修改或新实现自己的控制率，并更改相关启动脚本及mixer文件就可以了. 而姿态估计、位置估计等模块可不做任何更改完全复用，这也是Pixhawk中通过消息发布/订阅模式来实现模块间通信，从而达到高度模块化的软件架构带来的好处. 


* [1. 前置知识](#1)

  + [1.1 旋转矩阵的含义](#1.1)

  + [1.2 飞机姿态控制中用到的几种旋转表示方法](#1.2)

    * [1.2.1 坐标系约定](#1.2.1)

    * [1.2.2 欧拉角表示](#1.2.2)

    * [1.2.3 四元数表示](#1.2.3)

    * [1.2.4 罗德里格旋转公式](#1.2.4)

  + [1.3 姿态误差的求取](#1.3)

* [2. Pixhawk姿态控制解析](#2)

  + [2.1 Pixhawk飞控架构](#2.1)

  + [2.2 Pixhawk姿态控制代码解析](#2.2)

    * [2.2.1 角度误差控制](#2.2.1)

    * [2.2.2 角速率误差控制](#2.2.2)



<h2 id="1">1. 前置知识</h2>

姿态控制需要用到空间旋转，矩阵等知识，网上教程很多，鱼龙混杂难以取舍，我在这里尽量以最简明的语言阐释所有需要用到的知识（KISS原则, 你懂的^_^）. 

<h3 id="1.1">1.1 旋转矩阵的含义</h3>

矩阵可表示两个向量间关系，有两种惯例来表示旋转矩阵的意义：

  1. 在固定坐标系下对向量进行旋转，旋转前坐标向量和旋转后坐标向量的关系；
  2. 对坐标系进行旋转，同一个向量在两个坐标系下坐标向量的关系；
  
我们将先以简单的二维旋转为例对上述两种意义逐一阐述，三维空间旋转矩阵不过多了一个维度，其意义相同. 

* 第一种表示意义：旋转向量
<center><img src="/assets/img/pixhawk_control_law/01RotationMatrix_1000.gif" alt = "RotationMatrix" /></center>

平面上的每一点 $${\displaystyle P}$$都有一个坐标 $${\displaystyle P(x,y)}$$，并对应着一个向量$${\displaystyle (x,y)}$$. 所有普通意义上的平面向量组成了一个空间，记作ℝ²，因为每个向量都可以表示为两个实数构成的有序数组$${\displaystyle (x,y)}$$. 在向量空间ℝ²，将固定坐标系内的给定向量$$\mathbf{v_0}$$(本例中为$$(1,0)^T$$)绕逆时针旋转$$\theta$$至$$\mathbf{v’}$$(本例中为$$(\cos\theta, \sin\theta)^T$$), 可表示如下：

$$\mathbf{v’} = \mathbf{R_\theta}\mathbf{v_0}$$

上式中$$\mathbf{R_\theta}$$为：

$$\mathbf{R_\theta} =
{\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}}
$$

<!-- $$\mathbf{v’} = \mathbf{R'_\theta}\mathbf{v_0}$$

$$\mathbf{R'_\theta} =
{\begin{bmatrix}
\cos\theta & \sin\theta \\
-\sin\theta & \cos\theta
\end{bmatrix}}
$$-->

* **第二种表示意义：旋转坐标系**

本文着重讨论此种惯例表示，因为在飞机姿态控制中会用到多个坐标系:大地坐标系(惯性系)，机体坐标系(非惯性系)...等, 我们需要知道同一向量(速度, 加速度,角速度等)在这些坐标系之间的转换关系, 理解此种表示法的意义非常重要. 

仍从最简单的二维旋转开始，如下图所示：
<center><img src="/assets/img/pixhawk_control_law/02RotationMatrixAxes_1000.gif" alt = "RotationMatrixAxes" /></center>

旋转矩阵可以表示不同坐标系下同一向量的坐标之间的关系. 以上图为例: $$\mathbf{v_0}=(1,0)^T$$及$$\mathbf{v’}=(\cos\theta, -\sin\theta)^T$$分别是向量**V**在坐标系OX<sub>0</sub>Y<sub>0</sub>及OX'Y'中的向量坐标，则有:

$$\mathbf{v_0} = \mathbf{R_\theta}\mathbf{v'}$$

$$\mathbf{R_\theta} =
{\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}}$$


矩阵中每个元素可视为两个坐标系内坐标轴向量夹角的余弦值，这也是称旋转矩阵为方向余弦矩阵(Direction Cosine Matrix)的原因. 


三维旋转较二维旋转复杂得多. 在三维空间内，以大地坐标系和飞机机体坐标系间的旋转关系为例，设大地坐标系为Oxyz, 机体坐标系为OXYZ（坐标系方向约定见[1.2.1节](#1.2.1)）, 设**i, j, k**为大地坐标系内x,y,z轴的单位向量. **I, J, K**为机体坐标系内X, Y, Z轴的单位向量，向量V在大地坐标系和机体坐标系内的坐标分别为$$\mathbf{v^G}$$,$$\mathbf{v^B}$$, 则此两向量的关系如下：

$$\begin{equation}\mathbf{v^G} = \begin{bmatrix}
\mathbf{v_x^G}  \\
\mathbf{v_y^G}  \\
\mathbf{v_z^G}
\end{bmatrix} = \begin{bmatrix}
\mathbf{i}\cdot\mathbf{I} & \mathbf{i}\cdot\mathbf{J} & \mathbf{i}\cdot\mathbf{K} \\
\mathbf{j}\cdot\mathbf{I} & \mathbf{j}\cdot\mathbf{J} & \mathbf{j}\cdot\mathbf{K} \\
\mathbf{k}\cdot\mathbf{I} & \mathbf{k}\cdot\mathbf{J} & \mathbf{k}\cdot\mathbf{K} 
\end{bmatrix}\begin{bmatrix}
\mathbf{v_x^B}  \\
\mathbf{v_y^B}  \\
\mathbf{v_z^B}
\end{bmatrix} = 
\mathbf{R_B^G}
\mathbf{v^B}\label{1eq}
\end{equation}$$

**旋转矩阵$$\mathbf{R_B^G}$$的每个元素即为两坐标系内坐标轴的夹角余弦值(即[点乘](https://zh.wikipedia.org/wiki/数量积)). 其三个列向量分别为机体坐标系的X, Y, Z轴在大地坐标系Oxyz内的坐标向量；其三个行向量分别为大地坐标系x, y, z轴在机体坐标系内坐标向量.** 即：

$$\begin{equation}\mathbf{R_B^G} = 
\begin{bmatrix}
\mathbf{I^G}  & \mathbf{J^G}  &\mathbf{K^G}
\end{bmatrix} = 
\begin{bmatrix}
\mathbf{i^B}  \\
\mathbf{j^B}  \\
\mathbf{k^B}
\end{bmatrix}\label{2eq}
\end{equation}$$

旋转矩阵为正交单位阵，其逆矩阵为其转置矩阵:　$$\mathbf{R^{T}}=\mathbf{R^{-1}}$$. 利用这一性质可在计算机内快速求其逆.    
请务必记清旋转矩阵的含义及其性质, 且不要将两坐标系的转换顺序弄反. 

下节将阐述如何根据各种旋转表示方法计算此矩阵. 

本节参考资料：  
[http://mathworld.wolfram.com/RotationMatrix.html](http://mathworld.wolfram.com/RotationMatrix.html)  
[https://gentlenav.googlecode.com/files/DCMDraft2.pdf](https://gentlenav.googlecode.com/files/DCMDraft2.pdf)  
[http://www.starlino.com/dcm_tutorial.html](http://www.starlino.com/dcm_tutorial.html)  
Randal W. Beard & Timothy W. McLain _Small Unmanned Aircraft: Theory and Practice_   

<h3 id="1.2">1.2 飞机姿态控制中用到的几种旋转表示方法</h3>

在飞控中我们更多地用到三维旋转，这较二维旋转复杂很多，因为二维旋转仅有一个旋转轴(垂直于OXY平面，即Z轴). 三维旋转可绕空间任意轴旋转，二维旋转只是三维旋转的特例. 除了矩阵，我们还可以用四元数、欧拉角等来表示旋转，他们具有一定的等价关系. 

<h4 id="1.2.1">1.2.1 坐标系约定</h4>

大地坐标系一般采用[NED(North east down)](https://en.wikipedia.org/wiki/North_east_down).   
机体坐标系一般定义如下图所示, roll, pitch, yaw轴分别为X, Y, Z轴:
<center><img src="/assets/img/pixhawk_control_law/04AirplaneAxes.png" alt = "AirplaneAxes" /></center>
坐标系方向和旋转方向遵从右手法则. 

<h4 id="1.2.2">1.2.2 欧拉角表示</h4>

以欧拉角表示旋转有多种约定，因为对于同一旋转**欧拉角与旋转次序密切相关**, [参考此处](https://zh.wikipedia.org/zh-cn/%E6%AC%A7%E6%8B%89%E8%A7%92#.E5.88.A5.E7.A8.AE.E9.A0.86.E5.BA.8F). 
航空航天工程中常用[z-y′-x″顺规](https://en.wikipedia.org/wiki/Euler_angles#Tait.E2.80.93Bryan_angles)，如下图所示：

<center><img src="/assets/img/pixhawk_control_law/03Taitbrianzyx.png" alt = "Taitbrianzyx" /></center>
从大地坐标系oxyz旋转到机体坐标系oXYZ的顺序为：

* 绕oz旋转$$\psi$$;
* 绕oy′旋转$$\theta$$;
* 绕ox″(即oX)旋转$$\phi$$.

$$\psi$$，$$\theta$$，$$\phi$$即分别为偏航，俯仰，滚转角(yaw, pitch, roll). 

若从机体旋转到大地坐标系，则为相反的顺序和负的旋转方向. 

通过此三个角度可得出旋转矩阵：

$$\begin{equation}\mathbf{R_B^G} = \begin{bmatrix} c(\psi)c(\theta) & c(\psi)s(\phi)s(\theta) - c(\phi)s(\psi) & s(\phi)s(\psi) + c(\phi)c(\psi)s(\theta) \\ c(\theta)s(\psi) & c(\phi)c(\psi) + s(\phi)s(\psi)s(\theta) & c(\phi)s(\psi)s(\theta) - c(\psi)s(\phi)\\ -s(\theta) & c(\theta)s(\phi) & c(\phi)c(\theta) \end{bmatrix}\label{3eqEulerAngles}
\end{equation}$$

式中以$$c(\alpha)$$表示$$\cos(\alpha)$$, $$s(\alpha)$$表示$$\sin(\alpha)$$. 


<!--
http://www.chrobotics.com/library/understanding-euler-angles
此网址中将此公式的矩阵为$$R_I^B$$, 实应为$$R_B^I$$-->

<h4 id="1.2.3">1.2.3 四元数表示</h4>
上节使用欧拉角表示两个坐标系间旋转需要旋转三次，而且存在[万向锁定问题(Gimbal Lock)](https://en.wikipedia.org/wiki/Gimbal_lock). 而实际任意两个坐标系间可仅通过一次旋转完成，用四元数表示旋转可直观的得出此旋转的旋转轴和旋转角. 

四元数通常可表示为$$\mathbb{R}^4$$内的一个向量：
$$\mathbf{e} = \begin{pmatrix}
e_0 \\ e_1 \\ e_2 \\ e_3 
\end{pmatrix}
$$


现以从大地坐标系到机体坐标系的旋转为例说明四元数的定义：设该旋转的旋转轴在大地坐标系内的单位向量为：$$(u_x, u_y, u_z)^\mathrm{T}$$, 旋转角度为$$\Theta$$, 可得出四元数的四个元素为：

$$e_0 = \cos\frac\Theta2$$

$$\begin{pmatrix}
e_1 \\ e_2 \\ e_3 
\end{pmatrix} = \begin{pmatrix}
u_x \\ u_y \\ u_z
\end{pmatrix}\sin\frac\Theta2
$$

[维基百科中对四元数](https://zh.wikipedia.org/wiki/四元数与空间旋转)表示旋转的推导比较简明，在此不再赘述. 

<!--$$q=e^{\frac\Theta2(u_xi+u_yj+u_zk)}=\cos\frac\Theta2+(u_xi+u_yj+u_zk)\sin\frac\Theta2$$-->

在1.2.2节中我们已经知道通过欧拉角可直接计算出旋转矩阵，实际上通过四元数也可计算出欧拉角，因此四元数、欧拉角、旋转矩阵间可相互转换. 

本节参考资料：  
[https://zh.wikipedia.org/wiki/四元数与空间旋转](https://zh.wikipedia.org/wiki/四元数与空间旋转)  
[https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)  
Randal W. Beard & Timothy W. McLain _Small Unmanned Aircraft: Theory and Practice_   

<h4 id="1.2.4">1.2.4 罗德里格旋转公式</h4>
若已知旋转轴$$\mathbf{u}$$(为单位向量)和旋转角度$$\Theta$$，我们可以通过计算四元数构造出旋转矩阵，但最简便的方法莫过于直接使用[罗德里格旋转公式](https://zh.wikipedia.org/wiki/罗德里格旋转公式)，当然这两者完全等价. 

两个坐标系间的旋转矩阵$$\mathbf {R}$$为：

$$\begin{equation}{\mathbf {R}}={\mathbf {I}}+(\sin \Theta ){\mathbf {U}}+(1-\cos \Theta ){\mathbf {U}}^{2}\label{4eqrodrig}
\end{equation}$$


也可写作：

$$\begin{equation}\mathbf {R} = (\cos \Theta)\mathbf {I} +(\sin \Theta){\mathbf {U}} +(1-\cos \Theta )\mathbf {u} \otimes \mathbf {u}
\label{5eqrodrig}
\end{equation}$$

仍以大地坐标系到机体坐标系的旋转说明上式: 若$$\mathbf{u}$$为旋转轴在大地坐标系内的坐标向量, 旋转角度为$$\Theta$$, 则上式计算结果即为$$\mathbf{R_B^G}$$.

上两式中矩阵$${\mathbf {U}}$$为向量$$\mathbf{u}$$的[叉积矩阵(cross product matrix)](https://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication):

$${\mathbf {U}}={\begin{bmatrix}0&-u_{z}&u_{y}\\
u_{z}&0&-u_{x}\\
-u_{y}&u_{x}&0
\end{bmatrix}}$$


$$\mathbf {u} \otimes \mathbf {u}$$为向量$$\mathbf {u}$$自身的张量积(也可认为是u的行向量与列向量之积):

$$\mathbf {u} \otimes \mathbf {u} ={\begin{bmatrix}u_{x}^{2}&u_{x}u_{y}&u_{x}u_{z}\\
u_{x}u_{y}&u_{y}^{2}&u_{y}u_{z}\\
u_{x}u_{z}&u_{y}u_{z}&u_{z}^{2}\end{bmatrix}}$$


由于$$\mathbf{u}$$为单位向量，可以看出矩阵$${\mathbf {U}}^{2}$$与矩阵$$\mathbf {u} \otimes \mathbf {u}$$之差为单位阵，因此$$\eqref{4eqrodrig}$$与$$\eqref{5eqrodrig}$$等价. 

<h3 id="1.3">1.3 姿态误差的求取</h3>


各姿态轴的误差可通过旋转轴**单位**向量与旋转角度的乘积给出:

$$\begin{equation}\mathbf{error} = \begin{pmatrix}
u_x \\ u_y \\ u_z
\end{pmatrix}\Theta
\label{6eqerror}
\end{equation}$$

<!--**TODO, where do I find the reference矩阵计算形式**-->


<h2 id="2">2. Pixhawk姿态控制解析</h2>

有了前面旋转矩阵等相关知识的铺垫，本章节所讲述Pixhawk多旋翼姿态控制就不是特别难了. 

<h3 id="2.1">2.1 Pixhawk飞控系统框图</h3>

Pixhawk飞控系统框图可表示如下：
<center><img src="/assets/img/pixhawk_control_law/2.1control_Arch.png" alt = "control_Arch" /></center>
最内层为姿态控制环节，外层依次为位置和航路控制环节. 上图对于多旋翼，固定翼及垂直起降(VTOL)飞机均适用，不同构型飞机飞控的差别在于姿态(Attitude)和位置(Position)的估计(Estimator)和控制(Controller)模块会使用不同的算法，因此会使用不同的模块. 不同构型飞机使用姿态位置估计控制模块可[参考此处的表格](http://onewayout.github.io/pixhawk/2016/05/24/pixhawk-principle-and-customize.html#1.5). 

<h3 id="2.2">2.2 Pixhawk姿态控制解析</h3>

Pixhawk中多旋翼姿态控制模块对应的源文件为[Firmware/src/modules/mc_att_control/mc_att_control_main.cpp](https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp). 其关键部分是角度误差控制及角速率误差控制，可表示如下图：
<center><img src="/assets/img/pixhawk_control_law/2.2attitude_controller.png" alt = "attitude_controller" /></center>
角度误差控制和角速率误差控制分别对应于源文件中的`control_attitude(float dt)`和`control_attitude_rates(float dt)`函数.   
此模块中油门(Thrust或Throttle)基本没作控制，以下不讨论. 

<h4 id="2.2.1">2.2.1 角度误差控制</h4>
<!--ad.hoc-->
此环节的功能由函数`control_attitude(float dt)`完成. 其输入为飞机当前姿态和目标姿态，通过此两姿态的旋转矩阵计算出三个姿态轴的角度误差，进行比例控制，并对航向角度误差(yaw error)另作前馈控制，输出为目标角速率.   
此控制环节最难的部分是**如何根据当前姿态和目标姿态两个旋转矩阵计算角度误差**，此部分完全是[开发者Anton Babushkin自己的想法](https://groups.google.com/forum/#!topic/px4users/ln3yzqF6Vz0), 无现成参考论文或资料. 其基本指导思想是：

1. 由于多旋翼roll和pitch的较yaw的响应速度快，因此首先控制roll和pitch，使飞机的Z轴与目标姿态Z轴一致，最后旋转z轴，消除yaw偏差, 根据此方法计算出各姿态误差；
2. 在目标姿态Z轴水平时，由于欧拉角的奇异点(singularity)问题，忽略yaw误差;
3. 若目标姿态与当前姿态Z轴夹角过大(90~270°), 意味着roll和pitch需要更大的控制量. 此时直接根据四元素计算出各姿态误差; 并将此误差与前面第1种方法计算出的误差取各自权重相加,得到最终各姿态误差.


<!--**TODO 几个坐标系的图示**  或表格;-->

{% highlight c++ %}
void
MulticopterAttitudeControl::control_attitude(float dt)
{
	vehicle_attitude_setpoint_poll();

	_thrust_sp = _v_att_sp.thrust;

	/* construct attitude setpoint rotation matrix */
	math::Matrix<3, 3> R_sp;
	R_sp.set(_v_att_sp.R_body);

	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Matrix<3, 3> R = q_att.to_dcm();

	/* all input data is ready, run controller itself */
{% endhighlight %}

此段代码主要是为得到: 

* 当前姿态机体坐标系到大地坐标系的旋转矩阵$${\mathbf {R_B^G}}$$(代码中为**`R`**);
* 目标姿态机体坐标系到大地坐标系的旋转矩阵$${\mathbf {R_{sp}^G}}$$(代码中为**`R_sp`**). 

至此所有输入已准备就绪，接下来逐步计算各姿态轴角度误差. 

我们之前提到过，旋转当前机体，以使当前机体的Z轴和目标姿态的Z轴重合，此姿态与目标姿态仅存在yaw误差，姑且称此姿态为**rp姿态**, 以下代码将计算将从当前姿态旋转到**rp姿态**的旋转轴. 
{% highlight c++ %}	
	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length();
	float e_R_z_cos = R_z * R_sp_z;

	
	/* calculate weight for yaw control */
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);
{% endhighlight %}
注意代码中**`%`**, **`*`**运算符已重载, 对向量而言分别是叉乘和点乘, 下同.  
根据前面[1.1节](#1.1)，**`R_z`**和**`R_sp_z`**分别为当前机体Z轴和目标机体姿态Z轴在大地坐标系中坐标向量. 其叉乘与此两向量垂直，即为使当前机体的Z轴和目标姿态的Z轴重合的旋转的旋转轴在大地坐标系中的向量，与**`R.transposed()`**相乘后，所得**`e_R`**即为此旋转轴在当前机体坐标系内的坐标向量. 根据向量叉乘及点乘定义，容易理解`e_R_z_sin`, `e_R_z_cos`分别为当前机体Z轴与目标机体Z轴夹角的正弦值和余弦值.

**`R_sp(2, 2)`**为目标机体姿态Z轴与大地z轴夹角的余弦值. **`yaw_w`**将在后面作为一个权重值使用, 稍后解释.

{% highlight c++ linenos %}
	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}
{% endhighlight %}
`e_R_z_sin`为非负值(因叉乘定义中两向量夹角范围为0~180°), 因此若Z轴存在误差, 第6~9行代码计算从当前姿态旋转到rp姿态的旋转轴向量(单位化), 以及旋转角, `e_R`即为从当前姿态到rp姿态的各姿态轴误差. 注意这几行代码的向量均在当前姿态坐标系内.

第12~22行代码即根据此旋转轴和旋转角度, 利用罗德里格旋转公式计算旋转矩阵. 第22行等式右侧最外层括号中即为rp姿态到当前姿态的旋转矩阵, 乘以R后, **`R_rp`**即为rp姿态坐标系至大地坐标系的旋转矩阵. 

{% highlight c++ linenos %}
	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q_error;
		q_error.from_dcm(R.transposed() * R_sp);
		math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f: -q_error.imag() * 2.0f;

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}
{% endhighlight %}



此段代码中第2~4行首先计算目标姿态Z轴与当前姿态Z轴误差角度. 前面提到过`yaw_w`为目标姿态Z轴与大地坐标系z轴夹角余弦值的平方, 为一权重值. 当两轴夹角为0时, `yaw_w`为1; 两轴夹角为90°, 由于欧拉角的奇异点, yaw与pitch重合, 无需控制yaw, 此时`yaw_w`为0. 明白这一点, 这三行代码也就不难理解了.

若目标姿态Z轴与当前姿态Z轴相差角度较小(±90°), 根据旋转两次的方法计算出的e_R向量即为最终的个姿态轴误差向量.  
若目标姿态Z轴与当前姿态Z轴相差角度较大(90~270°), 可仅考虑一次旋转(不需要中间的rp姿态), 利用四元数直接计算三轴误差角度.

第9~11行代码通过四元数计算出从当前姿态到目标姿态的旋转轴和旋转角度, 从而直接得出三个姿态轴的误差角度向量**`e_R_d`**. 

第14~15行代码: 将通过两种不同方法计算出的误差角度向量**`e_R_d, e_R`**以各自权重组合, 计算得到最终的误差角度向量(此处不是特别明白).


至此, 已完全计算得出姿态轴角度误差, 接下来根据此误差计算出目标角速率.

{% highlight c++ %}
	/* calculate angular rates setpoint */
	_rates_sp = _params.att_p.emult(e_R);

	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if (_v_control_mode.flag_control_velocity_enabled && !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));
		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
		}
	}

	/* weather-vane mode, dampen yaw rate */
	if (_v_att_sp.disable_mc_yaw_control == true && _v_control_mode.flag_control_velocity_enabled && !_v_control_mode.flag_control_manual_enabled) {
		float wv_yaw_rate_max = _params.auto_rate_max(2) * _params.vtol_wv_yaw_rate_scale;
		_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max);
		// prevent integrator winding up in weathervane mode
		_rates_int(2) = 0.0f;
	}

	/* feed forward yaw setpoint rate */
	_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;

	/* weather-vane mode, scale down yaw rate */
	if (_v_att_sp.disable_mc_yaw_control == true && _v_control_mode.flag_control_velocity_enabled && !_v_control_mode.flag_control_manual_enabled) {
		float wv_yaw_rate_max = _params.auto_rate_max(2) * _params.vtol_wv_yaw_rate_scale;
		_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max);
		// prevent integrator winding up in weathervane mode
		_rates_int(2) = 0.0f;
	}

}
{% endhighlight %}
以上代码相对比较简单, 对角度误差进行比例控制得到目标角速率, 进行饱和抑制, 并对yaw另单独进行前馈控制. 


<h4 id="2.2.2">2.2.2 角速率误差控制</h4>
上节中我们已经得到目标角速率, 当前角速率可由姿态估计模块给出, 角速率误差控制环节的输入即为此两角速率. 输出为各姿态轴控制命令. 对照代码容易看出其控制框图如下, 此处不再列出源代码:
<center><img src="/assets/img/pixhawk_control_law/2.3control_rates.png" alt = "control_rates" /></center>
此处需要注意除了<span style="color: Green;">前馈环节</span>以外, 图中<span style="color: Red;">PI控制器</span>以及<span style="color: Blue;">微分环节</span>并不是典型的PID控制器, 因为<span style="color: Blue;">此微分环节</span>的输入为当前角速率, 而非角速率误差. 可如此理解<span style="color: Blue;">此微分环节</span>的作用, 若当前角加速度过大, 应加以控制使其减小, 即对当前角速率进行微分负反馈. 
