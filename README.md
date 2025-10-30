<div align="center">

# RVIZ人机交互对话面板插件

**一个基于QT实现的RVIZ人机交互对话面板插件，用于在RVIZ中与机器人进行自然语言的交互。**

</div>


## 功能
* Qt对话聊天框基于https://github.com/ShaShiDiZhuanLan/Demo_MessageChat_Qt.git
* 既可以在RVIZ中以panel插件的形式调用，也可以单独作为ROS包启动。
* 在输入框，输入可以通过键盘打字，也可以通过语音输入。
* 以微信聊天风格显示输出结果，可以通过tts模块将文本转换为语音播放。
* 通过ROS topic实现的信息交互，包括用户输入和机器人输出。


## 安装
### 1. 下载源代码到~/catkin_ws/src/下

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/dialogue_panel.git
```

### 2. 正确安装好[ASR](speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online/README.md)和[TTS](Kokoro-82M-v1.1-zh/README.md)的依赖

### 3. 编译

```bash
cd ~/catkin_ws
catkin_make
```

### 4. 运行
#### 4.1 启动TTS服务

如果需要tts运行结果，则需要提前在安装了kokoro的虚拟环境下运行script/tts_service.py

```bash
conda activate kokoro
python script/tts_service.py
```

#### 4.2 终端中打开RVIZ

```bash
roscore
rviz
```

#### 4.3 打开RVIZ后，点击左上角第二个按钮"Panels"->"Add New Panel"->"dialogue_panel"->"DialoguePanel"

#### 4.4 运行与测试
* 在文本框中输入内容后，点击"发送"按钮或者按下Enter键，即可发送消息到"/user_msg"话题。
* 按住“说话”按钮，同时说话，识别到的文字会加入到输入框中。
* 对话框订阅"/bot_msg"话题，获取机器人的输出结果。
* 输出结果会以微信聊天风格显示在对话框中，按下气泡旁边的小喇叭图标可以播放语音。

### 5. 关闭时直接Ctrl+C不好用！！！应该点击左上角第二个按钮"Panels"->"Remove Panel"->"DialoguePanel"，然后会自己闪退

### 6. 修改订阅话题在"src/mainwindow.cpp"的38-39行，mainwindow类的构造函数里

```cpp
MainWindow::MainWindow(QWidget *parent) : 
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    resize(600, 800);
    InitASRWorker();
    InitTTSWorker();

    // 主要在这里修改
    static ros::NodeHandle nh;
    m_botMsgSub = nh.subscribe("/bot_msg", 10, &MainWindow::bot_msg_Callback, this);    // 机器人消息，修改为GPT生成的日志消息，即对话框中对面发送过来的消息，例如“已到达餐桌”
    m_userMsgPub = nh.advertise<std_msgs::String>("/user_msg", 10); // 用户消息，修改为用户输入的消息，例如“我想去喝水”

    tts_client = nh.serviceClient<dialogue_panel::TTS>("tts_request");
}
```