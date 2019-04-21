## 软件环境

- Toolchain/IDE : Arduino 1.8.7
- 处理器 : ATmega328P(Old Bootloader)

## 注意事项

- 需依据自己的模块连线定义相应的引脚
- 缺少位置环，小车无法回到原来的位置

## 调试流程

## 硬件设置

- Arduino NANO
- HC06蓝牙模块
- MPU6050模块
- TB6621电机驱动

### 模块测试

通过使能相关宏定义来测试模块，宏定义写在Control.cpp中

- TEST_IMU IMU模块测试
- TEST_HC06 蓝牙模块测试
- TEST_CHASSIS 底盘编码器测试

### PID参数整定

通过使能宏定义TEST_PID来测试模块，宏定义写在Control.cpp中

- 直立环：通过在Control.cpp的BalancePWM_Set()函数中最后添加```Balance_pwm = out1;```来整定
- 角速度环：通过在Control.cpp的BalancePWM_Set()函数整定
- 速度环：通过在Control.cpp的VelocityPWM_Set()函数整定