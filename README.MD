# 本工程用于franka与Fliexiv之间的交互

使用本系统前，请先激活环境
```
source set_env.bash
```
* 需要开启MqttServer
```
cd MqttServer
npm start # 启动服务
```

* 开启订阅器
```
cd subscriber
./control_client_sub_flexiv.py
./control_client_sub_franka.py
```

* 直接运行相关案例
在案例中有相关的详细操作介绍，这里就不细讲了
```
control_client_pub.py

```