# PGFuzz for ArduPilot

## XML Parser

通过读取官方文档中给的参数信息（XML文件），处理得到各个参数对应的：

  (1) Parameter name（参数名）
  (2) Description（描述）
  (3) Valid range（有效范围）
  (4) Increment unit（增加粒度）
  (5) Read-only or not（是否只读）

## Dynamic Analysis

动态分析将用户命令参数和环境参数映射到各个状态上<u>（为什么没有配置参数，配置参数是静态分析？）</u>，之后在分析状态时就只用变异与其有关的参数。

<u>（看PGFuzz文档里面，生成的结果很少，状态最多只与两三个用户命令参数和环境参数相关，还有一些状态没有相关的用户命令参数和环境参数）</u>

PGFuzz将**34**个状态分为**15**类，动态分析与这15类状态有关的用户命令参数和环境参数

<u>（运行动态分析时，会将所有输出文件创建（如不存在）或清空。自己实验发现，PGFuzz在GitHub上的输出的15类状态相关参数，与自己复现过程中生成的参数有差异），例如position.txt文件。PGFuzz原本相关参数是RC1和RC2，但是自己跑的时候变为SIM_ACC2_BIAS_Y</u>

![image-20221007152925832](C:\Users\Roscky\PycharmProjects\PGFuzz\ArduPilot\md_image\image-20221007152925832.png)

