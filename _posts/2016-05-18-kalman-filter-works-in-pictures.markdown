---
layout: post
title:  "图示卡尔曼滤波器原理(译文)"
date:   2016-05-18 22:38:16 +0200
categories: jekyll update
---

>
> 最近需要用到卡尔曼滤波器，看了一些文章，发现这篇[ How a Kalman filter works, in pictures](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures)最为深入浅出，通俗易懂，翻译出来供朋友们参考。
> 若你发现有任何不当之处请Email至：[cia120@163.com](mailto:cia120@163.com)
>

我将在本文介绍[卡尔曼滤波器](https://zh.wikipedia.org/wiki/卡尔曼滤波)，因为它在工程中有着广泛应用. 

令人吃惊的是貌似很少有软件工程师和科学家知道它，这让我有点伤感，因为这是一个如此通用和强大的**信息融合**工具，特别是出现不确定因素时. 其提取精确信息的能力曾一度看起来有些不可思议. 如果说我有些夸大其词，请看[我之前发的一个视频](http://www.bzarg.com/p/improving-imu-attitude-estimates-with-velocity-data)，这个视频中我演示了卡尔曼滤波器根据一个自由浮动的物体的*速度*得出其*方位*. 非常巧妙. 

# 卡尔曼滤波器是什么?

卡尔曼滤波器对于含**不确定信息**的动态系统都可适用，你可以对系统下一步的行为做一个**有根据的猜测**. 哪怕实际情况与你的猜测相去甚远，卡尔曼滤波器通常也能计算出实际发生了什么. 并且它能有效利用你可能不会想到的各种极端现象之间的联系. 

对于连续变化的系统，卡尔曼滤波器是十分理想的. 其优势是占用内存少（除了上一个状态，不需要存储其他历史数据），而且速度快，这些优势使其非常适用于实时问题和嵌入式系统. 

Google一下卡尔曼滤波器，搜索到的大部分文章用到的数学知识都看起来晦涩难懂，让人望而却步. 事实上如果你从正确的角度看待它，就会发现卡尔曼滤波器其实超级简单易懂,这使它成为一个很好的文章主题.我将以许多清晰、漂亮的图片以及各色符号来阐述之. 理解本文你仅需的前置知识是懂一些概率论和矩阵. 


我们先从一个不太严谨的例子开始. 但如果你想直奔主题到漂亮的图片和数学公式，请[跳到这里继续阅读](#jump). 

# 卡尔曼滤波器能干什么？

我们假定这样一个例子：你组装了一个能在树林中漫步的小机器人，它必须知道自身的确切位置，从而能自主行驶. 

<div style="text-align:center">
<img src="/assets/img/robot_forest-300x160.png" alt="Your little robot" />
</div>

我们可以说这个机器人有一个状态向量$$\vec{x_k}$$，包含位置和速度：

$$\vec{x_k} = (\vec{p}, \vec{v})$$

注意状态向量其实就是列出了关于系统组态的一些数字. 在我们的例子中就是位置和速度，根据你所研究的问题不同，它也可能是油箱中液体的体积，汽车引擎的温度，手指在触摸板上的位置等你所关注的任何东西. 

我们的机器人有GPS传感器，其精度约10m，还不错. 但它还需更精确地知道自身位置，树林中遍地沟壑，就算有几十厘米的失误，机器人也可能跌入其中，所以仅靠GPS定位还是不够的. 
<center><img src="/assets/img/robot_ohnoes-300x283.png" alt="Oh no." /></center> 

我们可能还能知道关于机器人移动的一些其他信息：发送给驱动轮电机的指令，若其行进方向无障碍，其下一个时刻可能就在这个方向稍远的位置. 当然关于其运动也有些未知因素：机器人可能被强风吹歪，其轮子可能打滑，或在崎岖的地形上翻滚. 因此轮子转动的距离不一定就代表了机器人行进的距离，预测也可能不准. 

GPS能得到关于状态一些东西，但并不直接，并带有不确定和不精确性. **预测**能得到机器人如何将如何移动，但有同样的局限性. 



如果用上所有这些手头上的信息，能比单独仅用GPS或仅凭预测得到更好的结果吗？答案是肯定的，这就是卡尔曼滤波器的用武之处. 


<span id="jump">  </span>

# 卡尔曼滤波器如何看你的问题

（`译注：下文很多地方将出现估计（estimate）和预测(predict)两词，请注意其区别,前者指根据测量值等已知量估计出所关心的状态变量的值，后者指根据当前状态预测下一时刻的状态.`）

看一下我想阐释的问题，我们继续从这个仅有位置和速度的简单状态向量开始：

$$\vec{x} = \begin{bmatrix}
p\\
v
\end{bmatrix}$$

我们并不知道确切的位置和速度，只知道一些位置和速度的可能组合，但其中一些组合的可能性比其他的更大:
<center><img src="/assets/img/gauss_0.png" width="310" height="325"/></center> 

 
 
卡尔曼滤波器假定这两个变量（在我们的例子中是位置和速度）是随机且[高斯分布](https://zh.wikipedia.org/wiki/正态分布)（即正态分布）. 每个变量有一个**平均值**$$\mu$$（也就是最可能的状态），以及[**方差(Variance)**](https://zh.wikipedia.org/wiki/方差)*$$\sigma^2$$*，方差表示不确定性:（`译注：下图中有一小错误，颜色标记的应为标准差，而非方差，否则单位不一致`）
<center><img src="/assets/img/gauss_1.png" width="310" height="276"/></center> 


在上图中，位置和速度是**不相关**的，这意味着你无法从一个变量的状态知道另一个变量的任何信息. 

下图显示了一个稍有趣的例子：位置和速度是**相关**的，观测到一个特定的位置的可能性取决于此时的速度. 
<center><img src="/assets/img/gauss_3.png" width="310" height="286"/></center> 

当我们以当前位置来预测下一个位置时，这种相关的情况就可能发生. 如果速度较快，下一位置就会更远，反之亦然. 

跟踪这种相关性非常重要，因为这给了我们**更多的信息**：对一个变量的测量值告诉我们其他变量的可能值. 这就是卡尔曼滤波器的目的，从一些不太确定的测量中尽量挤出更多有用的信息!

这种相关性可以用[协方差矩阵(Covariance matrix)](https://zh.wikipedia.org/wiki/协方差矩阵)来量度. 简单来说，该矩阵中每个元素$$\Sigma_{ij}$$表示了第i个状态变量和第j个状态变量的相关性的程度. （你可能猜得到协方差矩阵是[对称阵](https://zh.wikipedia.org/wiki/對稱矩陣)，因为交换i,j的次序, 结果相同）. 协方差矩阵通常以符号“$$\mathbf{\Sigma}$$”表示，因此其每个元素可以“*$$\Sigma_{ij}$$”*表示.
<center><img src="/assets/img/gauss_2.png" width="310" height="286"/></center>

# 用矩阵描述我们的问题

若对这个问题中的状态向量建模，我们需要知道在*k*时刻的两个信息：最佳估计$$\mathbf{\hat{x}_k}$$（均值）,
**以及协方差矩阵**$$\mathbf{P_k}$$. 

$$\begin{equation} \label{eq:statevars}
\begin{aligned}
\mathbf{\hat{x}}_k &= \begin{bmatrix}
\text{position}\\
\text{velocity}
\end{bmatrix}\\
\mathbf{P}_k &=
\begin{bmatrix}
\Sigma_{pp} & \Sigma_{pv} \\
\Sigma_{vp} & \Sigma_{vv} \\
\end{bmatrix}
\end{aligned}
\end{equation}$$

(此处我们仅使用了位置和速度两个变量，在实际应用中状态向量可包含任意个变量，用来表示你所关心的任何信息). 

接下来，我们很关心如何得到<span style="color: RoyalBlue;">当前状态</span>(<span style="color: RoyalBlue;">**k-1**</span>)和预测<span style="color: DeepPink;">下一状态</span>(<span style="color: DeepPink;">**k**</span>)，注意我们并不知道状态值是否真实，但是预测函数对此并不关心，它仅在所有状态可能性的基础上给出一个新的预测分布：
<center><img src="/assets/img/gauss_7.jpg" width="310" height="286"/></center>
当前状态向量到预测下一步的状态向量的转换可用矩阵$$\mathbf{F_k}$$表示：
<center><img src="/assets/img/gauss_8.jpg" width="310" height="286"/></center>
从上图可看出，每一个可能的向量$$\color{royalblue}{\mathbf{X_{k-1}}}$$（原始估计）都可通过矩阵$$\mathbf{F_k}$$转换到$$\color{deeppink}{\mathbf{X_k}}$$(预测值), 若原始估计值正确，则系统下一个时刻的状态很可能为预测值. 

在我们这个机器人的小例子中，如何使用矩阵来预测下一时刻的位置和速度呢？使用基本的运动学公式：（这里认为两个时刻的速度近似不变）

$$\begin{split}
\color{deeppink}{p_k} &= \color{royalblue}{p_{k-1}} + \Delta t
&\color{royalblue}{v_{k-1}} \\
\color{deeppink}{v_k} &= &\color{royalblue}{v_{k-1}}
\end{split}$$

即：

$$\begin{align}
\color{deeppink}{\mathbf{\hat{x}}_k} &= \begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix} \color{royalblue}{\mathbf{\hat{x}}_{k-1}} \\
&= \mathbf{F}_k \color{royalblue}{\mathbf{\hat{x}}_{k-1}}\label{statevars}
\end{align}$$

现在我们得出了**预测矩阵**$$\mathbf{F_k}$$，能告诉下一个状态；但如何更新协方差矩阵仍未知. 

这里我们需要使用另外一个公式，如果对每一个具有概率分布的点乘以矩阵$$\color{firebrick}{\mathbf{A}}$$，其协方差矩阵$$\Sigma$$会如何？

其实这也不难，如下：

$$\begin{equation}
\begin{split}
Cov(x) &= \Sigma\\
Cov(\color{firebrick}{\mathbf{A}}x) &= \color{firebrick}{\mathbf{A}}\Sigma \color{firebrick}{\mathbf{A}}^T
\end{split} \label{covident}
\end{equation}$$

综合$$\eqref{covident}$$, $$\eqref{statevars}$$两式：

$$\begin{equation}
\begin{split}
\color{deeppink}{\mathbf{\hat{x}}_k} &= \mathbf{F}_k\color{royalblue}{\mathbf{\hat{x}}_{k-1}} \\
\color{deeppink}{\mathbf{P}_k} &= \mathbf{F_k}\color{royalblue}{\mathbf{P}_{k-1}} \mathbf{F}_k^T
\end{split}　\label{eq5}
\end{equation}$$

## 外部影响

然而我们并未考虑完全，外部环境的某些变化可能**与状态向量不相关**，但仍能影响系统. 

例如：若上节中的状态向量表示了列车的运动，列车驾驶员可能推动油门，使列车加速. 类似的，在我们的机器人中，导航软件可能发出使轮子转动或停止的指令. 如果知道实际情况中这些额外的信息，我们可以将其放在向量$$\color{darkorange}{\vec{\mathbf{u}_k}}$$中，利用这个向量对我们的预测做进一步纠正. 

比方说根据油门或控制指令，我们知道预期的加速度$$\color{darkorange}{a}$$，可以得到：

$$\begin{split}
\color{deeppink}{p_k} &= \color{royalblue}{p_{k-1}} + {\Delta t}
&\color{royalblue}{v_{k-1}} + &\frac{1}{2} \color{darkorange}{a}
{\Delta t}^2 \\
\color{deeppink}{v_k} &= &\color{royalblue}{v_{k-1}} + &\color{darkorange}{a} {\Delta t}
\end{split}$$

矩阵的形式如下：

$$\begin{equation}
\begin{split}
\color{deeppink}{\mathbf{\hat{x}}_k} &= \mathbf{F}_k\color{royalblue}{\mathbf{\hat{x}}_{k-1}} + \begin{bmatrix}
\frac{\Delta t^2}{2} \\
\Delta t
\end{bmatrix} \color{darkorange}{a} \\
&= \mathbf{F}_k \color{royalblue}{\mathbf{\hat{x}}_{k-1}} +\mathbf{B}_k \color{darkorange}{\vec{\mathbf{u}_k}}
\end{split} \label{eq6}
\end{equation}$$

$$\mathbf{B}_k$$称之为**控制矩阵**，$$\color{darkorange}{\vec{\mathbf{u}_k}}$$为**控制向量**. （对于简单无外部干扰的系统，可忽略这两者）

我们可以加入更多细节，但若预测模型并非100%准确将会发生什么？

## 外部不确定性

如果状态向量按照其固有属性发展，一切好说（可利用公式$$\eqref{eq5}$$）. 如果系统在有外界力的作用下发展，只要我们知道这些力，仍没问题(利用式$$\eqref{eq6}$$). 

但如果我们不知道这些力，怎么办？例如，强风的影响，轮子打滑，路面颠簸. 我们无法对这些全部考虑，而这些事情会导致我们的预测失灵. 

通过给每一步预测添加表示不确定的量，我们可以给环境的这种不确定性建模：
<center><img src="/assets/img/gauss_9.jpg" width="310" height="286"/></center>
如上图所示，k-1时刻的估计值$$\color{royalblue}{\mathbf{\hat{x}}_{k-1}}$$经过预测步骤后，可能以协方差$$\color{mediumaquamarine}{\mathbf{Q}_k}$$移动至<span style="color: Purple;">紫色</span>高斯斑点（Gaussian blob）内某个位置，也可以说将未知的环境影响视为协方差为$$\color{mediumaquamarine}{\mathbf{Q}_k}$$的**噪音**. 
<center><img src="/assets/img/gauss_10a.jpg" width="310" height="310"/></center>
这将产生一个新的高斯斑点，但有不同的协方差（相同的平均值）：
<center><img src="/assets/img/gauss_10b.jpg" width="310" height="310"/></center>
简单地**加上**$$\color{mediumaquamarine}{\mathbf{Q}_k}$$，能得到**预测步骤**的完整表达式：

$$\begin{equation}
\begin{split}
\color{deeppink}{\mathbf{\hat{x}}_k} &= \mathbf{F}_k\color{royalblue}{\mathbf{\hat{x}}_{k-1}} + \mathbf{B}_k\color{darkorange}{\vec{\mathbf{u}_k}} \\
\color{deeppink}{\mathbf{P}_k} &= \mathbf{F_k}\color{royalblue}{\mathbf{P}_{k-1}} \mathbf{F}_k^T +\color{mediumaquamarine}{\mathbf{Q}_k}
\end{split}
\label{kalpredictfull}
\end{equation}$$

也就是说，<span style="color: DeepPink;">新的最佳估计</span>是对<span style="color: RoyalBlue;">上一步最佳估计</span>的**预测**，并加上考虑<span style="color: DarkOrange;">已知的外界影响</span>后的**修正量**. 

<span style="color: DeepPink;">新的不确定性</span>是从<span style="color: RoyalBlue;">上一步的不确定性</span>**预测**得来，并加上<span style="color: MediumAquamarine;">环境的额外不确定性</span>. 

如此一来就很简单了，通过$$\color{deeppink}{\mathbf{\hat{x}}_k}$$，$$\color{deeppink}{\mathbf{P}_k}$$，我们有对系统的模糊估计. 下一步我们将看看当从传感器读数据时，实际上发生了什么？

# 利用测量改善估计

我们可能有一些能给出系统状态信息的传感器. 目前它们能测什么没什么关系，可能有的传感器能测位置，有的能测速度. 每个传感器能**间接地（indirect）**给出状态的某些信息，也就是说，传感器对系统状态进行某种操作，从而给出**读数**. 
<center><img src="/assets/img/gauss_12.jpg" width="621" height="286"/></center>

注意到传感器读数的单位和比例与我们所关心的系统状态的单位比例可能不同，你可能猜得到，我们以矩阵$$\mathbf{H}_k$$给传感器这种行为建模. 
<center><img src="/assets/img/gauss_13.jpg" width="621" height="286"/></center>
我们可以弄明白传感器读数的分布：

$$\begin{equation}
\begin{aligned}
\vec{\mu}_{\text{expected}} &= \mathbf{H}_k\color{deeppink}{\mathbf{\hat{x}}_k} \\
\mathbf{\Sigma}_{\text{expected}} &= \mathbf{H}_k\color{deeppink}{\mathbf{P}_k} \mathbf{H}_k^T
\end{aligned}
\end{equation}$$

卡尔曼滤波器很棒的一点就是能处理传感器噪音. 噪音是指传感器总是有一些不可靠之处，原始估计中的每个状态会产生一系列可能的读数. 
<center><img src="/assets/img/gauss_14.jpg" width="621" height="286"/></center>

根据观测到的每个读数，我们可以猜测系统处于某个特定的状态. 但由于有一些不确定性，一些状态比其他状态更具可能性，这些更具可能性的状态就产生了我们看到的读数:
<center><img src="/assets/img/gauss_11.jpg" width="310" height="286"/></center>
我们将此不确定性(例如传感器噪音)的**协方差**称为$$\color{mediumaquamarine}{\mathbf{R}_k}$$，此分布的**均值**就是我们观测到的读数$$\color{yellowgreen}{\vec{\mathbf{z}_k}}$$. 

现在我们有两个高斯斑点，一个的中心是预测的转换（$$\mathbf{H}_k\color{deeppink}{\mathbf{\hat{x}}_k}$$）后的均值，另一个的中心是传感器读数($$\color{yellowgreen}{\vec{\mathbf{z}_k}}$$). 
<center><img src="/assets/img/gauss_4.jpg" width="310" height="286"/></center>	

对于系统状态现在可以从两个途径猜测，一个是基于从<span style="color: DeepPink;">**上一状态的预测**</span>（式$$\eqref{kalpredictfull}$$），另一个是根据**传感器读数**，我们必须兼顾调和这两个猜测. 


那么最有可能的新系统状态是什么？对于我们看到的读数$$(z_1,z_2)$$，实际上有两种相关的可能性：<span style="color: YellowGreen;">（1）</span>传感器读数$$\color{yellowgreen}{\vec{\mathbf{z}_k}}$$测出了$$(z_1,z_2)$$的值，<span style="color: DeepPink;">（2）</span>从上一步估计推测出此时系统的状态，此估计产生了我们看到的读数. 

如果有这两种可能性，而我们想知道这两种都正确的几率，只需要将其相乘：
<center><img src="/assets/img/gauss_6a.png" width="310" height="286"/></center>

**重叠的区域**最具可能性. 这比任何单独一种的预测精确得多. 此分布的均值就是上述两种估计都最具可能性的值，因而也就是在所有给定信息基础上的**最佳猜测**. 这看起来很像另外一个高斯分布. 
<center><img src="/assets/img/gauss_6.png" width="310" height="286"/></center>
事实证明，若将两个具有不同均值和协方差矩阵的高斯分布相乘，会得到一个具有新均值及新协方差矩阵的高斯分布. 下文将讲述此转换过程中各高斯分布参数间关系. 

# 组合高斯分布

从最简单的一维高斯分布开始，均值为$$\mu$$， 方差为*$$\sigma^2$$*的**一维高斯分布**可定义如下：

$$\begin{equation} \label{gaussformula}
\mathcal{N}(x, \mu,\sigma) = \frac{1}{ \sigma \sqrt{ 2\pi } }
e^{ -\frac{ (x – \mu)^2 }{ 2\sigma^2 } }
\end{equation}$$

$$\begin{equation} \label{gaussequiv}
\mathcal{N}(x, \color{fuchsia}{\mu_0},\color{deeppink}{\sigma_0}) \cdot \mathcal{N}(x,\color{yellowgreen}{\mu_1}, \color{mediumaquamarine}{\sigma_1})\stackrel{?}{=} \mathcal{N}(x, \color{royalblue}{\mu’},\color{mediumblue}{\sigma’})
\end{equation}$$

我们想知道将两个高斯分布相乘会得到什么？
<center><img src="/assets/img/gauss_joint.png" width="589" height="381"/></center>
将$$\eqref{gaussformula}$$式代入$$\eqref{gaussequiv}$$式并进行一些代数运算（注意需对新的分布作归一化处理，以使其总概率为1），可以得到：

$$\begin{equation} \label{fusionformula}
\begin{aligned}
\color{royalblue}{\mu’} &= \mu_0 + \frac{\sigma_0^2 (\mu_1 –\mu_0)} {\sigma_0^2 + \sigma_1^2}\\
\color{mediumblue}{\sigma’}^2 &= \sigma_0^2 –\frac{\sigma_0^4} {\sigma_0^2 + \sigma_1^2}
\end{aligned}
\end{equation}$$

通过引入系数$$\color{purple}{\mathbf{k}}$$，可将上式简化:

$$\begin{equation} \label{gainformula}
\color{purple}{\mathbf{k}} = \frac{\sigma_0^2}{\sigma_0^2 +\sigma_1^2}
\end{equation}$$

$$\begin{equation}
\begin{split}
\color{royalblue}{\mu’} &= \mu_0 + &\color{purple}{\mathbf{k}}
(\mu_1 – \mu_0)\\
\color{mediumblue}{\sigma’}^2 &= \sigma_0^2 –
&\color{purple}{\mathbf{k}} \sigma_0^2
\end{split} \label{update}
\end{equation}$$


看这公式多简洁！我们能在之前的估计基础上加上一些东西，从而得到新的估计. 

这几个公式的矩阵版本是怎样的呢？只需将$$\eqref{gainformula}$$和$$\eqref{update}$$以矩阵的形式重写就可以了. 
若$$\Sigma$$为高斯分布的协方差矩阵，$$\vec{\mu}$$为其均值，则有：

$$\begin{equation} \label{matrixgain}
\color{purple}{\mathbf{K}} = \Sigma_0 (\Sigma_0 +\Sigma_1)^{-1}
\end{equation}$$

$$\begin{equation}
\begin{split}
\color{royalblue}{\vec{\mu}’} &= \vec{\mu_0} +
&\color{purple}{\mathbf{K}} (\vec{\mu_1} – \vec{\mu_0})\\
\color{mediumblue}{\Sigma’} &= \Sigma_0 –
&\color{purple}{\mathbf{K}} \Sigma_0
\end{split} \label{matrixupdate}
\end{equation}$$

将$$\color{purple}{\mathbf{K}}$$矩阵称为**卡尔曼增益**，后面将会用到它. 

简单吧，我们还有一点点就学完了！

# 综上所述

我们有两个分布：预测值的测量$$(\color{fuchsia}{\mu_0},\color{deeppink}{\Sigma_0}) = (\color{fuchsia}{\mathbf{H}_k\mathbf{\hat{x}}_k}, \color{deeppink}{\mathbf{H}_k \mathbf{P}_k\mathbf{H}_k^T})$$；
观测到的测量$$(\color{yellowgreen}{\mu_1},\color{mediumaquamarine}{\Sigma_1}) =
(\color{yellowgreen}{\vec{\mathbf{z}_k}},\color{mediumaquamarine}{\mathbf{R}_k})$$. 将他们代入$$\eqref{matrixupdate}$$中：

$$\begin{equation}
\begin{aligned}
\mathbf{H}_k \color{royalblue}{\mathbf{\hat{x}}_k’} &=\color{fuchsia}{\mathbf{H}_k \mathbf{\hat{x}}_k} & + &\color{purple}{\mathbf{K}} (\color{yellowgreen}{\vec{\mathbf{z}_k}} –\color{fuchsia}{\mathbf{H}_k \mathbf{\hat{x}}_k} ) \\
\mathbf{H}_k \color{royalblue}{\mathbf{P}_k’} \mathbf{H}_k^T &=\color{deeppink}{\mathbf{H}_k \mathbf{P}_k \mathbf{H}_k^T} & – &\color{purple}{\mathbf{K}} \color{deeppink}{\mathbf{H}_k\mathbf{P}_k \mathbf{H}_k^T}
\end{aligned} \label {kalunsimplified}
\end{equation}$$

从$$\eqref{matrixgain}$$式，得到卡尔曼增益是：

$$\begin{equation} \label{eq:kalgainunsimplified}
\color{purple}{\mathbf{K}} = \color{deeppink}{\mathbf{H}_k\mathbf{P}_k \mathbf{H}_k^T} ( \color{deeppink}{\mathbf{H}_k\mathbf{P}_k \mathbf{H}_k^T} +\color{mediumaquamarine}{\mathbf{R}_k})^{-1}
\end{equation}$$

可将$$\eqref{kalunsimplified}$$中等式左侧的$$\mathbf{H}_k$$，$$\mathbf{H}_k^T$$都消掉，从而简化为：

$$\begin{equation}
\begin{split}
\color{royalblue}{\mathbf{\hat{x}}_k’} &=\color{fuchsia}{\mathbf{\hat{x}}_k} & + &\color{purple}{\mathbf{K}’} (\color{yellowgreen}{\vec{\mathbf{z}_k}} –\color{fuchsia}{\mathbf{H}_k \mathbf{\hat{x}}_k} ) \\
\color{royalblue}{\mathbf{P}_k’} &= \color{deeppink}{\mathbf{P}_k}
& – & \color{purple}{\mathbf{K}’} \color{deeppink}{\mathbf{H}_k\mathbf{P}_k}
\end{split}
\label{kalupdatefull}
\end{equation}$$

$$\begin{equation}
\color{purple}{\mathbf{K}’} = \color{deeppink}{\mathbf{P}_k\mathbf{H}_k^T} ( \color{deeppink}{\mathbf{H}_k \mathbf{P}_k\mathbf{H}_k^T} + \color{mediumaquamarine}{\mathbf{R}_k})^{-1}
\label{kalgainfull}
\end{equation}$$

这就是**更新步骤**的完整方程.
$$\color{royalblue}{\mathbf{\hat{x}}_k’}$$是新的最佳估计，我们可以持续将此值以及$$\color{royalblue}{\mathbf{P}_k’}$$反馈给新的一轮又一轮的**更新**与**预测**. 

<center><img src="/assets/img/kalflow.png" width="850" height="1100"/></center>

# 小结

以上所有数学公式中，你仅需实现式$$\eqref{kalpredictfull}, \eqref{kalupdatefull}$$, 及 $$\eqref{kalgainfull}$$.（或者直接从$$\eqref{covident}$$, $$\eqref{matrixupdate}$$式推导）

通过这些公式，你准确地可对任何线性系统建模，对于非线性系统，通过对预测及测量的平均值作简单的线性化处理，可使用**扩展卡尔曼滤波器(Extended　Kalman Filter)**. 

本文参考了[这篇好文](http://www.cl.cam.ac.uk/~rmf25/papers/Understanding%20the%20Basis%20of%20the%20Kalman%20Filter.pdf)，有探索精神的朋友可在此文中看到更深入的推导. 
