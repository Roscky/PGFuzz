## PGFUZZ项目实验流程

### Dynamic Analysis

#### 全局变量

##### avg_(name) list

其中全局变量有很多avg_(name)列表，存储了无人机飞行过程中各个参数的不同时间的值，为了后面使用Standard_deviation(list)函数计算方差，判断无人机在这段时间内该参数有没有发生较大的变化。

#### 主程序流程

1. 调用mavlink_connection()函数，通过串口、 tcp 或 udp 信道建立与 MAVLink 系统的通信链路

2. 使用request_data_stream_send函数指定无人机发送消息的类型（所有）以及发送速率

3. 设置无人机模式为“GUIDED"

4. 发送arm指令使无人机能够起飞

5. 设置高度为100，使无人机takeoff起飞

6. 创建守护线程t1，保持无人机throttle参数（控制无人机油门）在中间（为了保持无人机稳定，相当于无人机摇杆保持中位值）

7. 设置无人机模式为“STABILIZE"

8. 创建守护线程t2，循环接收无人机消息，并根据消息的类型和具体内容更新信息（特别是当前位置信息和状态历史信息列表）

9. 从"cmds.txt"读取可输入的命令和对应的索引号，从"envs.txt"读取环境参数

10. 在当前目录下创建result文件夹，并在该文件夹下创建输出文件（以写方式打开，如果之前存在内容会被清空）

11. 将"preconditions.txt"中的预先设置参数发送给无人机

12. 使用profile()函数分析各个环境参数和各个命令参数

    

### pymavlink常用函数

#### 创建链接-mavlink_connection()

[Python (mavgen) · MAVLink Developer Guide](https://mavlink.io/en/mavgen_python/#setting_up_connection)

```python
from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14540')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Once connected, use 'the_connection' to get and send messages
```

#### 请求数据-request_data_stream_send()

使用master.mav.request_data_stream_send指定无人机发送消息的类型（所有）以及发送速率，然后使用master.recv_match读取特定参数

```
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 6, 1)
```

#### 读取参数-recv_match()

等待并拦截到达的消息

```python
def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
    '''Receive the next MAVLink message that matches the given type and condition
    type:        Message name(s) as a string or list of strings - e.g. 'SYS_STATUS'
    condition:   Condition based on message values - e.g. 'SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4'
    blocking:    Set to wait until message arrives before method completes. 
    timeout:     ? <!-- timeout for blocking message when the system will return. Is this just a time? -->
    '''
```

#### 飞行模式Dict-master.mode_mapping()

mapping mode names to id, or None if unknown

#### 发送命令设置模式参数-set_mode_send()

```
# Set new mode
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
```

#### 接收ACK

发送完消息后，需要确认收到对方传回ACK，确认消息发送状态

```
# Check ACK
ack = False
while not ack:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    # “Command is valid (is supported and has valid parameters), and was executed.”
    break
```

如果正常设置参数，则print消息为“Command is valid (is supported and has valid parameters), and was executed.”

#### 发送命令command_long_send()

除了<message>_send外（如set_mode_send），还有一个发送命令的函数command_long_send，

[Messages (common) · MAVLink Developer Guide](https://mavlink.io/en/messages/common.html#COMMAND_LONG)



### 补充知识

#### failsafe

因为无线电波在传输过程中可能受到干扰或是数据丢失等等问题，当接收机无法接收到发射器的数据时，通常会进入保护状态，也就是仍旧向无人机发送控制信号，此时的信号就是接收机收到遥控器发射器最后一次的有效数据。这样因为信号丢失而发送的保护数数据通常叫做failsafe数据。

#### pwm脉宽调制

遥控器与无人机的通信协议，需要在无人机接收机接上全部pwm输出通道，每一个通道就要接一组线，解析程序需要根据每一个通道的pwm高电平时长计算通道数值。

无人机使用的pwm值在500-2500之间，而常用的值在1000-2000之间。

ArduPilot使用的pwm值在1000-2000之间，pwm值表示输出信号，代表着输出的百分比。比如对throttle channel输出pwm值为1500，则是将电机转速设置为满转速的50%。

[无人机遥控器的通道（遥控器三大通道详解） | 我爱无人机网 (woiwrj.com)](https://www.woiwrj.com/wurenjibaike/54992/)

[如果根据pwm值计算固定翼无人机的电机转速？ - 知乎 (zhihu.com)](https://www.zhihu.com/question/61271260)

#### Ardupilot的RC channel

RC输入通道的默认映射，set_rc_channel_pwm函数rc_channels_override_send函数设置RC通道数值

| Channel | Meaning      |
| ------: | :----------- |
|       1 | Pitch        |
|       2 | Roll         |
|       3 | Throttle     |
|       4 | Yaw          |
|       5 | Forward      |
|       6 | Lateral      |
|       7 | Camera Pan   |
|       8 | Camera Tilt* |

