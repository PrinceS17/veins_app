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

#### Veins工程架构
由于需要联合sumo和OMNeT++进行实时仿真，Veins工程的结构比较复杂，试列举如下（其中\*为通配符）
- veins/
  - examples/
    - veins/
      - result/
        - \*.vec, \*.sca：仿真结果的标量或向量
      - \*.xml, \*.sumocfg：sumo工程文件
      - \*.ini：工程配置文件，如omnetpp.ini
      - \*Scenario.ned：总场景定义
      - config*.xml：物理层配置文件，选择信道、阴影等模型参数
      - ...
  - src/
    - veins/
      - base/：Veins各模块的基本定义，无需改动
      - modules/
        - analogueModel/
          - AGChannelModel.cc/.h, \*.cc/.h：各种信道模型，新实现含空地信道的信道模型
        - application/
          - ieee80211p/：基本应用层定义
          - traci/
            - TaskOffload.cc/.h/.ned, ReplicaTask.cc/.h/.ned, \*.cc/.h/.ned：应用层
        - mobility/
          - traci/：TraCI通信所定义的车辆移动性
          - CircleMobility.cc/.h/.ned, LinearMobility.cc/.h/.ned, WayPointMobility.cc/.h/.ned：Veins中移动性模型
        - ... (其他文件夹，无需改动)
      - nodes/
        - \*.ned：各类节点定义，新实现UAV.ned
        

#### 代码清单
本平台在Veins基础上添加应用层、信道模型、移动性模型以及整体配置，新实现的代码列举如下（应用层.cc, .h, .ned分别为实现段、声明段、模块定义段，不再分开列举）：
* MyVeinsApp/
  1. TaskOffload.cc/.h/.ned：single offloading的架构和算法（UCB、VUCB、AVUCB、AUCB、Random），经测成熟；
  2. ReplicaTask.cc/.h/.ned：replica offloading的架构和算法（LTRA），经测相对成熟，但多TaV场景下尚不完善；
  3. MyVeinsApp.cc/.h/.ned：AVE架构原始版本，未测试完成；
  4. AppOfHandler.cc/.h/.ned：AVE架构，相较MyVeinsApp使用msg-Handler架构，未测试完成；
  5. ToolFunction.cc/.h：应用层统一用到的部分工具函数；
  6. UAV.ned：UAV模块定义；
  7. UAV_scenario.ned：含UAV的场景模块定义；
  8. omnetpp.ini：G6地面车联网配置文件；
  9. test_uav.ini：含UAV的空地车联网配置文件；
* Mobility/
  1. CircleMobility.cc/.h/.ned：圆周运动；
  2. WayPointMobility.cc/.h/.ned：多端点的直线运动；
* lust_script
  1. lust_cut.sh：linux脚本，用于剪裁并生成较小的适合Veins仿真的LuST子场景；
  2. lust_veins_cfg.sh：linux脚本，用于将LuST子场景的sumo文件加入Veins并进行配置。

新实现代码在Veins目录中位置：
  * MyVeinsApp/1~5: veins/src/veins/modules/application/traci
  * UAV.ned: veins/src/veins/nodes/
  * UAV_scenario.ned, \*.ini: veins/examples/veins/
  * Mobility/\*: veins/src/modules/mobility
  * lust_script: 任意路径皆可
  
### 实现细节
本节将具体介绍配置文件、sumo、Veins应用层及UAV模块相关的实现细节。

#### omnetpp.ini 配置细节

* 交通流、网络联合仿真

Veins仿真通过omnetpp.ini来设置整个仿真的重要参数，其中较重要的有：
```
sim-time-limit = 400s               # 仿真总时间
*.node[*].appl.delay_limit = 0.6    # 任务deadline
*.node[*].appl.m = 20               # replica中[0, 1]量化区间数
*.node[*].appl.x_low = 0.24         # single中x-
*.node[*].appl.x_high = 0.24        # single中x+
*.node[*].appl.cur_ucb = 2          # 单次运行时single中算法类型，(01234) = (ucb vucb avucb aucb random)
*.node[*].appl.K = 2                # 单此运行时replica中的任务复制数

*.manager.launchConfig = xmldoc("G6_badaling.launchd.xml")    # sumo的工程配置文件

*.node[*].applType = "TaskOffload"  # 节点搭载的应用
```
其中，G6_badaling.launchd.xml中指定了所使用的sumo工程文件，余下参数则与Veins仿真有关。

* 参数扫描

Veins中可以对程序中的参数进行扫描，亦即逐次运行参数取不同值时的程序。以single算法类型cur_ucb为例，需进行如下设置：
  * 在模块定义TaskOffload.ned中定义该参数
  ```int cur_ucb = default(0);    # 若*.ini不指定，则默认取0```
  * 在omnetpp.ini中定义重复次数、随机种子和扫描范围
  ```
  repeat = 1                                  # 每个参数只运行一次
  seed-set = ${repetition}                    # 种子为重复数，这里每次的 repetition = 0，从而使得其具有可重复性
  *.node[*].appl.cur_ucb = ${n = 0..4 step1}  # cur_ucb取0、1、2、3、4，分别为UCB、VUCB、AVUCB、AUCB、Random
  ```
  * 在源文件TaskOffload.cc中获取omnetpp.ini的设置值，以在应用代码中使用
  ```long i = par("cur_ucb").longValue();```

需要注意的是，如果使用参数扫描设置，则需要注释对应的单次运行设置，如
```*.node[*].appl_cur_ucb = 2```

同时，扫描参数需要对应相应的应用。比如如果设置任务复制数K，则需要使用ReplicaTask作为任务卸载应用，否则设置无效。

* 运行设置

打开Veins的Run Configurations，可以选择要运行的仿真、用户界面等。选择Qt界面，可进行网络行为的可视化，便于阅读日志和直观感受任务卸载过程；选择cmd界面，将直接在命令行运行，便于快速运行和扫描参数。具体运行模式与相关设置参见<https://www.omnetpp.org/documentation>中User Guide的相关部分。运行时，注意不要勾选多进程，选择单CPU串行，以保证参数不同值时场景设定相同。

#### SUMO 交通流设置
* G6 高速参数设置
* LuST 场景生成（linux script实现）

#### 应用层实现
- 基本抽象
  wsm，send，schedule 等等；
  
* TaskOffload: Single Offloading 

Single Offloading所实现的应用分为TaV, SeV两部分。TaV接收SeV的beacon以获取SeV的移动性信息，每秒生成并依据调度算法选择一辆SeV卸载计算任务，接收SeV传回的数据并更新延时等结果；SeV每秒发送beacon，接收TaV传来的计算任务概要和数据，进行处理，结束时传回结果。具体函数介绍如下。

  * 流程函数
    * initialize()：初始化，定义和接收参数，并根据TaV/SeV类型注册不同的Handler函数，并第一次发送beacon或生成、卸载任务；
    * onWSM()：收到WaveShortMessage，此时根据msg-Handler表调用对应Handler函数；
    * onBSM()/onWSA()：DSRC中收到BasicSafetyMessage,WaveServiceAdvertisment时的处理函数，本应用中不使用；
    * handleSelfMsg()：收到Self Message，此时同样查表调用Handler。
  
  - Handler函数
    - handleTraffic()
    
    仅用于TaV应用：
    
    - handleBeacon()
    - handleOffload()
    - updateResult()
    
    仅用于SeV应用：
    
    - sendBeacon()
    - processBrief()
    - processTask()
    - sendResult()
    - sendDup()
    
  
  - SeV_info 的数据结构
  - 状态机
 
- ReplicaTask: Replica Offloading
  - 流程函数
  
  ReplicaTask流程函数同前。
  
  - Handler函数
    - handleTraffic()
    
    仅用于TaV应用
    
    - handleBeacon()
    - handleOffload()
    - sendDataDup()
    - updateResult()
    
    仅用于SeV应用
    
    - sendBeacon()
    - processBrief()
    - processTask()
    - sendResult()
    - sendDup()
    
  - SeV_info 的数据结构
  - 状态机

- AppOfHandler、MyVeinsApp: AVE Framework
（存在bug，不再维护）

#### UAV模块实现
* 移动性
* 含UAV的信道实现
