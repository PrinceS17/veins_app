## Title
**bold** *Italy*
* wo No.
1. with No.

``` cd test/ ```

<http://github.com>


### 简介

#### 仿真需求介绍

#### Veins的整体代码架构

#### 代码清单

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
