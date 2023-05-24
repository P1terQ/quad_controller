# 4-20
stuck in the swing trajectory

# 4-21
start to store this project in github.(test)

# 4-22
把positionerrorgain调小,上楼梯就可以了

footplacement constraint的权重还需要调整
加入了friction cone的发现了法向量获取不知道效果如何
明天还要把elevation mapping那边再搞搞

# 4-23
footplacement constraint的权重调好了.现在是把b归一化了，不会出现某一个方向不稳定的现象了
elevation mapping调好了
friction cone暂时没问题

上楼梯的时候不稳定，需要解决

# 4-24
上楼梯不稳定的原因是之前做预测的时候Z方向没有进行赋值,导致有时候高度高了容易投影到错误的平面上去.
现在基本可以实现上下楼梯了

elevationmapping在跨沟的时候识别不到?
加入navigation planner?
之后上实物state estimator还是需要的

# 4-25
看了两篇CBF论文，想把CBF尝试加上去。貌似有很多2D的navigation算法，3D的很少看到

# 5-3
尝试sim2real，主机无法直接获取到摄像头话题，采用ros多机通信，elevationmapping报错

# 5-7
盲走版本能用，尝试解决elevation_mapping报错，尝试外置NUC

# 5-9
加入optiTrack辅助定位,not finished

# 5-15
optiTrack加上了后效果并没有取得改善，暂时先不在现实环境中调试了
之前在现实中实验的时候改了一点代码，
考虑将WBC tracker换成RL tracker

# 5-22 
找到了几篇参考的文章。具体的代码可以参照deep_mimic或者GenLoco的，这是开源代码里最接近我的想法的了。继续学习isaac_sim和orbit

# 5-23
之后需要加一个dummy_navigator.直接在控制模块给目标是不可行的，因为落足点需要实时根据期望速度更新
在load_controller.launch文件中加入了if_perceptive标志位，默认不使用elevation_mapping为false

# 5-24
已经加上perceptive_ptp模式了，通过发布‘/dummy_navigator_PTP/rel_goal’（这个有时候好像还有问题）和‘/dummy_navigator_PTP/abs_goal’（这个应该是稳定的）这两个话题控制，而且这里用的是欧拉角，比'/move_base_simple/goal'好用多了，并且这个是通过位置PID输出速度的，所以在perceptive模式下也兼容。
设计了一个stepping_stones的场景，发现机器人非常不稳定，明明看上去蛮容易的，思考一下
