# LCCP

## 原理

LCCP是Locally Convex Connected Patches的缩写，翻译成中文叫做“局部凸连接打包一波带走”。算法大致可以分成两个部分：1.基于超体聚类的过分割。2.在超体聚类的基础上再聚类。超体聚类作为一种过分割方法，在理想情况下是不会引入错误信息的，也就是说适合在此基础上再进行处理。LCCP方法并不依赖于点云颜色，所以只使用空间信息和法线信息，Wc=0,Ws=1,Wn=4。

点云完成超体聚类之后，对于过分割的点云需要计算不同的块之间凹凸关系。凹凸关系通过CC（Extented Convexity Criterion）和SC（Sanity Criterion）判据来进行判断。其中CC利用相邻两片中心连线相邻与法向量的夹角来判断两片是凹是凸。显示，如果途中a1>a1则为凹，反之为凸。
![在这里插入图片描述](https://image-1312312327.cos.ap-shanghai.myqcloud.com/20201013154550937.png)

## 程序

vs2019打开sln文件

## 实验结果

原始pcd文件

![OCID1_PNG](https://image-1312312327.cos.ap-shanghai.myqcloud.com/OCID1_PNG.png)

分割后结果

![OCID1_LCCP](https://image-1312312327.cos.ap-shanghai.myqcloud.com/OCID1_LCCP.png)