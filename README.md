编写foc控制代码

参考：

[如何从零开始写一套自己的FOC矢量控制程序 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/103758450)

[从零开始自己编写FOC 算法篇：FOC和SVPWM_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1wP4y167ao/?spm_id_from=333.788&vd_source=c13bbf05d4b570439590d88626eb3093)

[最小FOC矢量控制系统所需的基本模块和功能配置 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/667327043)

[GitHub - chenzt2020/foc_learning](https://github.com/chenzt2020/foc_learning/tree/main)

[FOC?看这篇文章就够了 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/364247816)

[正点原子电机笔记——FOC入门 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/654633911#:~:text=实际PID控制只使用了PI控制，没有引入微分控制，因为 电流的采样率非常高，不需要加入微分项,。 FOC目的 Iq是需要的，代表了期望的力矩输出；Id是不需要的，尽可能控制为零！)

[ svpwm - 知乎 (zhihu.com)](https://www.zhihu.com/search?type=content&q=svpwm)

[FOC项目知识点总结三 | 完全搞懂 Clarke 与 Park 变换（附动图，仿真文件以及详细讲解数学推导过程）_clarke park-CSDN博客](https://blog.csdn.net/weixin_43229030/article/details/115190315)

[STM32 进阶教程 19 - IQmath数学库的使用-CSDN博客](https://blog.csdn.net/zhanglifu3601881/article/details/97617417)

[数字电路设计（30）之电流检测(含计算)_stm32 10a电流检测-CSDN博客](https://blog.csdn.net/qq_37120496/article/details/133471283?ops_request_misc=%7B%22request%5Fid%22%3A%22170616773116800186553556%22%2C%22scm%22%3A%2220140713.130102334..%22%7D&request_id=170616773116800186553556&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-133471283-null-null.142^v99^control&utm_term=INA240A1&spm=1018.2226.3001.4187)

[foc学习笔记3——电流环_foc电流环-CSDN博客](https://blog.csdn.net/jdhfusk/article/details/120646346)

[【电机控制】PMSM无感FOC控制（一）FOC入门_foc有感、无感无刷电机控制-CSDN博客](https://blog.csdn.net/Liu_eight_nine/article/details/132448058)



![img](https://pic1.zhimg.com/80/v2-ffd192a2ed4cfb2a52df559f3e5d7a6c_720w.webp)

### **FOC目的** 

![img](https://pic4.zhimg.com/80/v2-6de1497d5b3eb21c543d8ae52ec452f7_720w.webp)

**Iq是需要的，代表了期望的力矩输出；Id是不需要的，尽可能控制为零！！**

 



仔细观察我们可以发现，当三相绕组产生的磁场方向始终与转子磁铁相切的时候最为理想，这样相同电流下产生的旋转力量最大（图中IQ），当我们三相绕组产生的磁场方向与转子磁场方向反向平行时，这时候电机会被吸在原地不动(图中ID)，电流都用来产生热量。因此我们以转子磁铁为参考，建立DQ坐标系，DQ坐标系随转子转动。(备注：为了便于理解我对磁铁进行了放大)

如图当我们让ID为0, IQ为1则转子就会逆时针旋转，且转速随着IQ的变大而变大。 如图当我们让ID为1, IQ为0则转子就会定在原地，且发热量随着ID的变大而变大。 因此可以总结一下，要想让转子平稳的转动下去就需要让三相绕组产生的磁场方向一直和转子磁铁的磁场方向垂直，即图中的IQ, 同时使ID尽可能为0以减小发热。那么怎么让绕组产生的磁场方向等效为IQ ID呢由此变引入Park反变换和Clark反变换。

**坐标变换可以采用全浮点实现，也可以采用标幺化处理，以 Q15 格式进行运算，三角函数可以采用查表实现，减少运算复杂度，节省运算时间。**

SVPWM 的实现一般涉及到硬件 PWM 模块的配置，矢量合成扇区的判断以及作用时间的计算等，常采用 STM32 的两个高级定时器 TIM1 和 TIM8 实现。高级定时器相对于通用定时器添加了可编程死区、重复计数器以及刹车等功能。

因为要产生互补对称的方波， PWM 定时器一般设置为中央对齐模式，且需要加入一定时间的死区，防止同一桥臂上的上下两个 MOS 管同时导通，造成系统短路。

[M法、T法以及M/T法测速原理概述 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/105165951)



最后实现 FOC 矢量控制两个大的任务模块，一个是 FOC 核心算法，这个核心算法需要在每个 FOC 执行周期中执行一次，一般放到 ADC 中断中执行；另外一个任务模块负责状态机的轮询和系统保护，这个模块通常放在系统滴答定时器中断中。

完成以上模块基本上可以实现一个 FOC 的最小控制系统，当然为了调试方便，可以配置 DAC 功能、USART 功能、CAN 功能等等。

**Iq是需要的，代表了期望的力矩输出；Id是不需要的，尽可能控制为零！！**

### **速度+电流环** 

![img](https://pic3.zhimg.com/80/v2-73d06cf463c2534899a5a5c6015abc22_720w.webp)

Speed_Ref代表期望转速，而ω是电机转速；
将**速度环的输出，作为电流环的输入**，实现速度+电流的双闭环控制！



所以为了避免速度环节带来的误差，在做位置控制的时候可以只使用位置和电流组成的双环进行控制，不过此时需要对位置环做一定的变化，控制框图如下：

![img](https://pic3.zhimg.com/80/v2-af17ac8465092f198e2f4f2ea8dad752_720w.webp)

位置-电流双闭环控制

由于去掉了速度环，这里的位置环我们使用完整的**PID控制**，即把微分项加上（因为位置的微分就是速度，这样可以减小位置控制的震荡加快收敛；积分项的作用是为了消除静态误差）。

​	







## 应用

[【自动驾驶】运动规划丨速度规划丨T型/S型速度曲线 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/453670719)

S型曲线加速

[SimpleFOC（八）—— 理论+实践 深度分析SVPWM_svpwm的原理及法则推导和控制算法详解-CSDN博客](https://blog.csdn.net/loop222/article/details/117468766)

[手把手教你为DC/DC转换器选择电感和电容 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/145217274)