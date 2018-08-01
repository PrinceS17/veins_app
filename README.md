#### 空天地一体化车联网仿真平台

本平台基于Veins，支持DSRC协议，可满足地面自治车联网及空地场景下移动边缘计算任务卸载仿真需要。交通流场景方面，本平台主要支持G6高速场景，也满足部分其他场景仿真需要。工程内容如下：

* MyVeinsApp：应用层代码实现
* lust_script：linux脚本，用于剪裁LuST地图生成子地图
* Mobility：Veins中移动性代码实现，为UAV提供移动模型
* veins：Veins工程中 example/veins，包含部分交通流模型

具体介绍参见 [开发文档][1]。

[1]:https://github.com/PrinceS17/veins_app/blob/master/MyVeinsApp/Doc.md
