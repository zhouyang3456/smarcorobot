增加：
	- 添加视频下载播放功能
	- 将提示语音路径加入配置文件

注意：
	- 更改配置文件
	- 更改config.yaml文件路径

大车：
	- roslaunch turn_on_... navigation.launch  
	  启动，再杀死，再启动，否则读不到pose数据
                 ps -ef | grep rosmaster
	 kill -2 xxxx

注意工控机上新建：Videos文件夹 用于保存下载的视频

测试使用的是测试小车车牌