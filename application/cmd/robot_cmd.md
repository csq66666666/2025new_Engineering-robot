# robot_cmd

<p align='right'>liyong@stu.cdu.edu.cn</p>

## 运行流程

运行流程可以很直观的从`RobotCMDTask()`中看出。

1. 首先通过消息订阅机制，获取其他应用的反馈信息
2. 使用`CalcOffsetAngle()`计算底盘和云台的offset angle（使得底盘始终获知当前的正方向）
3. 接着根据当前是通过键鼠or遥控器控制，调用对应的函数，将控制指令量化为具体的控制信息
4. 得到控制信息之后，先不急着发布，而是检测重要的模块和应用是否掉线或出现异常，以及遥控器是否进入紧急停止模式，如果以上情况发生，那么将所有的控制信息都置零，即让电机和其他执行单元保持静止。
5. 最后通过pubsub机制，把具体的控制信息发布到对应话题，让其他应用获取。若为双板，则将原本要推送给底盘的信息通过CANComm进行发送。



### 遥控器控制模式:

<table>
    <tr>
        <td align="center"><b>左侧开关<b></td> 
        <td align="center"><b>右侧开关<b></td> 
        <td align="center"><b>功能</b></td>
        <td align="center"><b>模式</b></td>
   </tr>
    <tr>
        <td align="center" rowspan=3>上</td>
  		<td align="center">上</td>
        <td align="center" rowspan=3>底盘控制</td>
      	<td align="center">正常行进模式</td>
    </tr>
    <tr>
        <td align="center">中</td> 
        <td align="center">取矿行进模式</td>
    </tr>
    <tr>
        <td align="center">下</td>
        <td align="center">无力模式</td>   
    </tr>
    <tr>
        <td align="center" rowspan=3>中</td>
  		<td align="center">上</td>
        <td align="center" rowspan=3>机械臂控制</td>
      	<td align="center">lift/push/traverse/yaw</td>
    </tr>
    <tr>
        <td align="center">中</td>
        <td align="center">roll/pitch/pitch_differ/roll_differ</td>
    </tr>
    <tr>
        <td align="center">下</td>
        <td align="center">无力模式</td>
    </tr>
    <tr>
        <td align="center" rowspan=3>下</td>    
  		<td align="center">上</td>
        <td align="center" rowspan=3>其他</td>
      	<td align="center">软件复位</td> 
    </tr>
    <tr>
        <td align="center">中</td> 
        <td align="center">键鼠控制</td>    
    </tr>
    <tr>
        <td align="center">下</td> 
        <td align="center">无力模式</td>    
    </tr>
    <tr>
        <td align="center"><b>拨轮<b></td>
        <td align="center"><b>方式<b></td>
        <td align="center"><b>功能</b></td>
        <td align="center"><b>模式</b></td>
    </tr>
    <tr>
        <td align="center" rowspan=2>上</td>    
        <td align="center">长拨2s</td>
        <td align="center">校准</td> 
        <td align="center">校准模式</td> 
    </tr>
    <tr>
        <td align="center">点按</td>
        <td align="center" rowspan=2>气泵控制</td> 
        <td align="center">开</td> 
    </tr>
    <tr>
        <td align="center">下</td>
        <td align="center">点按</td>
        <td align="center">关</td> 
    </tr>
</table>

### 键鼠控制模式:

前后左右:WSAD

云台yaw/pitch: ctrl + shift + A:D / W:S

抬升/前伸/横移: shift + Q:E / W:S / A:D

yaw/大pitch: 鼠标左右/鼠标前后

大Roll/小pitch/小Roll: ctrl + E:Q / W:S / A:D

自定义控制器: V

切换视角: B

一位三矿: shift + F

小资源岛取矿: F

大资源岛取矿: G

退出模式: ctrl + x

打开/关闭真空泵: R / shift + R

打开/关闭弹仓盖: Z / ctrl + Z
