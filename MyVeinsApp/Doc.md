## Title
**bold** *Italy*
* wo No.
1. with No.

``` cd test/ ```

<http://github.com>

1<sub>down</sub>
  
2<sup>up</sup>

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
  1. 在模块定义TaskOffload.ned中定义该参数
  ```int cur_ucb = default(0);    # 若*.ini不指定，则默认取0```
  2. 在omnetpp.ini中定义重复次数、随机种子和扫描范围
  ```
  repeat = 1                                  # 每个参数只运行一次
  seed-set = ${repetition}                    # 种子为重复数，这里每次的 repetition = 0，从而使得其具有可重复性
  *.node[*].appl.cur_ucb = ${n = 0..4 step1}  # cur_ucb取0、1、2、3、4，分别为UCB、VUCB、AVUCB、AUCB、Random
  ```
  3. 在源文件TaskOffload.cc中获取omnetpp.ini的设置值，以在应用代码中使用
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
##### 基本实现机制

由于OMNeT++的消息触发性质，我们使用msg-Handler对，以统一收到消息时各个处理函数的调用过程。具体msg-Handler对应表如下：
SeV:
```
            Handler[selfB] = &TaskOffload::sendBeacon;
            Handler[onD] = &TaskOffload::processTask;
            Handler[onJ] = &TaskOffload::processBrief;
            Handler[selfR] = &TaskOffload::sendResult;
            Handler[selfDup] = &TaskOffload::sendDup; 
```
TaV:
```
            Handler[selfG] = &TaskOffload::handleOffload;
            Handler[onB] = &TaskOffload::handleBeacon;
            Handler[onD] = &TaskOffload::updateResult;
```
对于Replica情形，TaV增加发送数据的msg-Handler对：
```
            Handler[selfD] = &ReplicaTask::sendDataDup;
```

另外，应用中对任务统一抽象为struct task如下：
```
struct task
{
    double data_size;       // 任务数据量
    double result_size;     // 结果数据量
    double cycle_per_bit;   // 运算强度：计算该任务1bit所需的CPU周期数
    double start;           // 产生时间
    double delay;           // 任务延时
};
```

##### TaskOffload: Single Offloading 

Single Offloading所实现的应用分为TaV, SeV两部分。TaV接收SeV的beacon以获取SeV的移动性信息，每秒生成并依据调度算法选择一辆SeV卸载计算任务，接收SeV传回的数据并更新延时等结果；SeV每秒发送beacon，接收TaV传来的计算任务概要和数据，进行处理，结束时传回结果。

  - 重要数据结构
  
  节点主要维护的数据结构如下：
  ```
  SeV_class SeV_info;               // TaV中存储的SeV信息，包括ID、（比特）延时bit_delay、次数count、出现时间occur_time、更新时间last_time
  map<int, task> work_info;         // <ID, task>, 节点生成（TaV）或收到（SeV）的未完成任务信息，收到结果（TaV）或处理结束（SeV）时清除
  map<int, int> bp_list;            // 节点状态记录表，在每个Handler操作之后赋值，用以指示该节点接下来应收到的wsm类型   
  ```
  
  - 流程函数
    - initialize()：初始化，定义和接收参数，并根据TaV/SeV类型注册不同的Handler函数，并第一次发送beacon或生成、卸载任务；
    - onWSM()：收到WaveShortMessage，此时根据msg-Handler表调用对应Handler函数；
    - onBSM()/onWSA()：DSRC中收到BasicSafetyMessage,WaveServiceAdvertisment时的处理函数，本应用中不使用；
    - handleSelfMsg()：收到Self Message，此时同样查表调用Handler。
  
  - Handler函数
    - handleTraffic()：发送交通信息（Veins原有操作）；
    
    仅用于TaV应用：
    
    - handleBeacon()：从Beacon中读取SeV ID、空闲情况、位置、速度，判断可用性并进行存储或更新等操作；
      - 可用性：对于车辆（以高度小于10m判决）而言，若SeV与本TaV距离小于最远通信距离，且速度差小于阈值，则认为该SeV可用；对于UAV而言，只需速度差满足条件即认为可用；
      - 若该SeV可用且不在SeV_info表中，则在表中增加其信息，若在表中，则只更新最新时间；若该SeV不可用，则打印信息，若其在表中，则清除表中已有信息；
      
    - handleOffload()：以1s为周期生成任务，调度合适的SeV，并发送数据进行任务卸载；
      - 根据均值和极差按均匀分布生成任务数据量，并设定结果数据量、运算强度、时间等参数，
      ```
      double x_t = uniform(x_av - dx, x_av + dx, myId % num_rng);   // 输入数据量
      task myTask = {x_t, x_t * alpha0, w0, simTime().dbl()};       // 各任务参数
      ```
      - 调度SeV流程：若SeV表为空或表中所有SeV均忙碌，则在本地处理；否则，在表中可用SeV集合中进行选择，若存在从未连接的SeV，则直接调度它，反之，调用scheduling()计算效用函数，选择其中值最低的SeV进行调度。确定SeV ID后，发送任务数据，并更新相关数据结构；
      - 调度算法：考虑数据量上界x<sup>+</sup>和下界x<sub>-</sub>，根据不同的调度算法，计算不同的utility函数，具体参见基于学习的任务卸载算法<sup>[1]</sup>
      
    - updateResult()：收到SeV传回的结果后，更新SeV_info中的延时、次数等数据，并记录仿真的任务完成率reliability和延时job_delay；
    
    仅用于SeV应用：
    
    - sendBeacon()：以1s为周期发送beacon，发送自消息selfDup以调度sendDup()发送自己的空闲情况、位置、速度；
    - processBrief()：收到任务概要brief后，存储相关信息至work_info\[id\]；
    - processTask()：接收数据包，在收到最后一个包时，调出对应任务信息，进行处理，在处理结束时刻发送自消息调用sendResult()；
    - sendResult()：发送结果至对应TaV；
    - sendDup()：发送beacon；

  - 状态机
    - 任务卸载与执行的过程可以抽象为Mealy机，即下一个状态由输入消息和当前状态决定。由于不同状态实际上对应不同消息，我们可以用消息类型来标识当前状态。例如，onD代表收到数据的状态，若TaV在此状态下，将查表调用updateResult()更新结果，并等待selfG消息。因此，以消息类型表示的状态转移过程如下：
      - TaV转移过程：0/selfG -> onD -> selfG -> ...
      - SeV转移过程：0/selfR -> onJ -> onD -> selfR -> ...
 
    - 由于无线信道的复杂性，不同节点间的干扰较多，我们根据以上状态转移过程，使用bp_list记录卸载对端的当前消息类型。例如，对于SeV而言，在processBrief()末尾，会将对应TaV的当前消息类型值改为onJ。在调用各个handler时，如果必要，应用将调用on_data_check()，通过以下代码，
      ```
      if(wsm->getKind() != nextKind(curKind))
      ```
      检查消息类型是否为应收到的消息类型。例如，若curKind为onJ，则其nextKind()将返回onD，意味着此时该SeV关于这个TaV只能收到数据包消息。
 
##### ReplicaTask: Replica Offloading
ReplicaTask应用实现基于学习的任务复制卸载<sub>[2]</sub>。与TaskOffload不同的是，Replica情形下，TaV将任务复制多份，同时卸载至多辆SeV进行处理，并接收多份返回结果。最快的返回结果用于计算延时，其他结果的延时用于更新SeV的延时分布，从而提高学习速度。

  - 重要数据结构
  ReplicaTask中的重要数据结构如下。
  ```
  SeV_class SeV_info;
  map<Pid, task> work_info;
  map<Pid, int> bp_list;
  ```
  
  注意此时虽然这三种数据结构意义不变，但实现和使用机制与TaskOffload不尽相同。在Replica情形下，不同于TaskOffload，我们定义Pid为车辆ID与任务时间的组合，以唯一标识每一个任务。
  ```
class Pid
{
public:
    int id;
    double time;

public:
    Pid(int id = 0, double time = 0) {this->id = id; this -> time = time;}
    Pid(stringstream& ss) { ss >> id >> time; }
    void write(stringstream& ss) { ss << id <<' '<< time <<' '; }
    Pid transform(int vid) { return Pid(vid, time); }
    bool operator < (const Pid& pid)const { return (time < pid.time || (time == pid.time && id < pid.id)); }     // use time as the comparable basis
    bool operator == (const Pid& pid) const { return (time == pid.time && id == pid.id); }  // for find?
};
  ```
  
  因此，work\_info和bp\_list的键都变为Pid而不是原来的车辆ID。另一方面，SeV_class中将原来维护的单一延时bit\_delay项改为了所有SeV延时的分布
  ```
  map<int, vector<double> > F;    // CDF of delay of SeVs
  ```
  
  从而实现对于SeV组合的学习和判断。
  
  - Handler函数
  ReplicaTask同样由Veins自带的空应用MyVeinsApp修改而来，因此其流程控制函数与TaskOffload相同。
  而Handler函数除了增加了sendDataDup()函数外，与TaskOffload架构相同。函数操作的主要区别在handleOffload()和其调用的调度函数schueduling()。
    
    - handleOffload(): 该函数每1s由自消息selfG触发，实现周期性将任务复制卸载至多辆SeV。
    首先，若服务车为空，则在本地处理；反之，调用scheduling函数，选出学习后认为其最小延时期望最小的K个SeV，调用send_data()函数，将任务数据卸载至选中的多辆SeV上。
    
    - scheduling()：该函数被handleOffload()调用，在给定的SeV信息情况下，选出K个SeV。具体分为三种情况：
    1. 集合大小N是否小于应选数量K？如果是，则返回集合中的所有SeV；
    2. 判断是否存在SeV没有接受过该TaV的卸载，若是，则返回一个含有该SeV的子集；
    3. 如果1、2都不满足，则先计算分布F<sub>new</sub>，之后调用oracle()，在F<sub>new</sub>基础上，对[0,1]量化后用穷举法计算最小延时期望最小的K元素子集并返回。数学细节不再此处赘述。
    
  - 状态机
  在实际系统中，多TaV与多SeV之间的干扰一直是应用实现的一个重要考量。Single Offloading情况下，由于1s内整体任务数较少，SeV的服务资源较为集中，因此整体任务延时大多在1s内，因此同一辆车在同一时间只有一个任务。然而，在Replica Offloading情况下，任务数增多，因此车辆间干扰增多，使得SeV可能同时收到多个任务。在实现中，SeV使用队列进行处理，因此排队靠后的任务延时可能较长。实际测试中，大量任务延时超过1s，从而导致在下1s的时间中，同一辆TaV可能存在多个正在进行的任务，而它们处于不同状态。例如，第一个任务正在被SeV处理，而第二个任务的概要刚被SeV收到。为了区分不同任务的状态，无法继续使用车辆ID来标识状态机，因此定义Pid为车辆ID和产生时间，从而标识不同状态机。具体状态转移仍然同TaskOffload

##### AppOfHandler、MyVeinsApp: AVE Framework
（存在bug，不再维护）

#### UAV模块实现
* 移动性
* 含UAV的信道实现

### 参考文献
[1]
[2]
