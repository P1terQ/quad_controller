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

# 5-12 
找到了几篇参考的文章。具体的代码可以参照deep_mimic或者GenLoco的，这是开源代码里最接近我的想法的了。继续学习isaac_sim和orbit