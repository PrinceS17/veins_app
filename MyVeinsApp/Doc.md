## Title
**bold** *Italy*
* wo No.
1. with No.

``` cd test/ ```

<http://github.com>

## 空天地一体化车联网仿真平台开发文档

### 简介
空天地一体化（Space-Air-Ground）车联网仿真平台基于Veins平台实现，结合SUMO和OMNeT++进行交通流仿真和网络仿真。本文档将说明平台的具体设计功能、使用方法和实现细节。

#### 仿真功能介绍
空天地一体化车联网中包含空、天、地三方节点，如地面的车辆、空中的UAV（无人机）和天空的卫星。各个节点之间通过无线通信，实现网络功能。本平台实现了可搭载于RSU、车辆和无人机的基于学习的任务卸载应用，也就是说，一个节点将自身产生的计算任务卸载给其他节点，以减小自身计算开销、降低任务计算延时等。结合G6高速公路的地图，平台可进行不同任务卸载算法的仿真。其次，本平台实现了UAV的移动性模型和信道模型，满足了UAV仿真的需要。另外，平台也试图加入LuST场景，以测试算法在城市场景下的性能，但目前仅实现了对LuST原始地图进行剪裁处理的脚本，尚未对应用代码进行完整测试。

#### Veins的整体代码架构

#### 代码清单
本平台在Veins基础上添加应用层、信道模型、移动性模型以及整体配置，新实现的代码列举如下（应用层.cc, .h, .ned分别为实现段、声明段、模块定义段，不再分开列举）：
* MyVeinsApp/
  1. TaskOffload.cc/.h/.ned：single offloading的架构和算法，经测成熟；
  2. ReplicaTask.cc/.h/.ned：
  3. MyVeinsApp.cc/.h/.ned
  4. AppOfHandler.cc/.h/.ned
  5. ToolFunction.cc/.h
  6. UAV.ned
  7. UAV_scenario.ned
  8. omnetpp.ini
  9. test_uav.ini
* Mobility/
  1. CircleMobility.cc/.h/.ned
  2. WayPointMobility.cc/.h/.ned
* lust_script
  1. lust_cut.sh
  2. lust_veins_cfg.sh
  
### 实现细节
#### omnetpp.ini 配置细节
* 交通流、网络联合仿真
* 参数扫描
  （如何设置参数，如何运行）

#### SUMO 交通流设置
* G6 高速参数设置
* LuST 场景生成（linux script实现）

#### 应用层实现
* 基本抽象
  wsm，send，schedule 等等；
* TaskOffload: Single Offloading 
  * 函数功能、实现
  * SeV_info 的数据结构
  * 状态机
 
* ReplicaTask: Replica Offloading
  * 函数功能、实现
  * SeV_info 的数据结构
  * 状态机

* AppOfHandler、MyVeinsApp: AVE Framework
（存在bug，不再维护）

#### UAV模块实现
* 移动性
* 含UAV的信道实现
