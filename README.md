# BMP180 驱动软件包



## 1 简介

bmp180 软件包是基于RT-Thread sensor框架实现的一个驱动包。基于该软件包，RT-Thread应用程序可以使用标准的sensor接口访问bmp180，获取传感器数据。



### 1.1 目录结构

| 名称            | 说明                  |
| --------------- | --------------------- |
| bmp180.h        | 头文件                |
| bmp80.c         | 源文件                |
| bmp180_sample.c | 测试应用程序          |
| README.md       | 软件包使用说明        |
| SConscript      | RT-Thread默认构建脚本 |
| LICENSE         | 许可证文件            |



### 1.2 许可证

bmp180 软件包遵循 Apache license v2.0 许可，详见 `LICENSE` 文件。

<br>

## 2 传感器介绍

bmp180  是 Bosch（博世）公司开发的一款环境传感器，支持气压和温度测量。bmp180 是一款上市比较久的传感器，很多功能并未支持，如电源模式、数据输出速率等不支持设置。



| 功能 | 量程        | 精度    |
| ---- | ----------- | ------- |
| 气压 | 300—1100hPa | 0.01hPa |
| 温度 | -40—80℃     | 0.1℃    |

<br>

## 3 支持情况



| 包含设备     | 气压计 | 温度计 |
| ------------ | ------ | ------ |
| **通信接口** |        |        |
| IIC          | √      | √      |
| SPI          |        |        |
| **工作模式** |        |        |
| 轮询         | √      | √      |
| 中断         |        |        |
| FIFO         |        |        |

<br>

## 4 使用说明

### 4.1 依赖

- RT-Thread 4.0.0+
- sensor 框架组件
- I2C 驱动，bmp180 设备使用 I2C 进行数据通讯，需要系统 I2C 驱动框架支持



### 4.2 获取软件包

使用 bmp180 package 需要在 RT-Thread 的包管理器中选择它，具体路径如下。然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。

```
RT-Thread online packages --->
    peripheral libraries and drivers --->
        sensors drivers --->
            [*] BMP180: BMP180 sensor driver package, support: barometric,temperature.
                    Version (latest)  --->
```

>  **Version**：软件包版本选择，默认选择最新版本。 



### 4.3 初始化

bmp180 软件包初始化函数如下所示：

```
int rt_hw_bmp180_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有:

- 根据配置信息配置i2c名称、i2c地址等（可增加其他配置信息），然后初始化设备

- 注册相应的传感器设备，完成 bmp180 设备的注册

  

**参考示例：**

```
#include "bmp180.h"

static int rt_hw_bmp180_port(void)
{
    struct rt_sensor_config cfg;
    	
	cfg.intf.dev_name = "i2c1"; 		/* i2c bus */
    cfg.intf.user_data = (void *)0x77;	/* i2c slave addr */
    rt_hw_bmp180_init("bmp180", &cfg);		/* bmp180 */

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_bmp180_port);
```



### 4.4 读取数据

bmp180 软件包基于sensor框架，sensor框架继承于RT-Thread标准设备框架，可以使用RT-Thread标准设备接口"open/read"读取传感器数据。



**参考伪代码:**

```
baro_dev = rt_device_find("baro_bmp180");
rt_device_open(baro_dev, RT_DEVICE_FLAG_RDONLY)；
rt_device_open(temp_dev, RT_DEVICE_FLAG_RDONLY)；
rt_device_control(baro_dev, RT_SENSOR_CTRL_SET_ODR, (void *)(1));
rt_device_read(baro_dev, 0, &sensor_data, 1);
```



### 4.5 msh/finsh测试

**查看设备注册**

```
msh >list_device
device           type         ref count
-------- -------------------- ----------
temp_bmp Sensor Device        1       
baro_bmp Sensor Device        1       
i2c1     I2C Bus              0       
pin      Miscellaneous Device 0       
uart4    Character Device     2       
```



**周期数据读取**

```
 \ | /
- RT -     Thread Operating System
 / | \     4.0.1 build Mar 12 2020
 2006 - 2019 Copyright by rt-thread team
[I/sensor] rt_sensor init success
[I/sensor] rt_sensor init success
msh >baro[100655Pa],temp[24.9C],timestamp[7]
baro[100649Pa],temp[24.7C],timestamp[511]
baro[100652Pa],temp[24.6C],timestamp[1015]
baro[100649Pa],temp[24.6C],timestamp[1519]
baro[100647Pa],temp[25.3C],timestamp[2023]
baro[100655Pa],temp[25.0C],timestamp[2527]
```



<br>

## 5 注意事项

暂无

<br>

## 6 联系方式

- 维护：[Acuity](https://github.com/Prry)
- 主页：<https://github.com/Prry/rtt-bmp180>      