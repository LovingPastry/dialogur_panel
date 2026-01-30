import rospy
import rospkg
import threading  # 新增：导入线程模块
from dialogue_panel.srv import TTS, TTSResponse
from kokoro import KModel, KPipeline  # KokoroTTS依赖（需Python 3.9+）
import sounddevice as sd
import numpy as np
import sys
import os
import torch
import json

print("当前python解释器路径:", sys.executable)
class TTSService:
    def __init__(self):
        # 1. 获取dialogue_panel包的绝对路径
        rp = rospkg.RosPack()
        package_path = rp.get_path("dialogue_panel")  # 获取当前包的路径
        
        # 2. 拼接模型文件路径
        model_path = f"{package_path}/Kokoro-82M-v1.1-zh"  # 根据实际目录结构调整
        
        # 3. 加载模型
        with open(os.path.join(model_path, "config.json"), 'r') as f:
            config = json.load(f)
            

        kmodel = KModel(
            repo_id='hexgrad/Kokoro-82M-v1.1-zh',
            config=config,
            model=os.path.join(model_path, "kokoro-v1_1-zh.pth")
            )

        self.pipeline = KPipeline(lang_code='z',
                            repo_id='hexgrad/Kokoro-82M-v1.1-zh',
                            model=kmodel
                            )
        
        self.voice_tensor = torch.load(os.path.join(model_path, 'voices/zf_001.pt'), weights_only=True)

    def tts_infer(self, text,debug=False)->torch.Tensor:
        generator = self.pipeline(
            text, voice=self.voice_tensor,
            speed=1, split_pattern=r'\n+'
        )
        
        gs, ps, audio = next(generator)
        if debug:
            print(gs)
            print(ps)
            print(audio.shape)
            
        # self.play_audio(audio)  # 播放生成的音频
        return audio
    
    def play_audio(self, audio_tensor, sample_rate=22050):
        """将torch.Tensor音频通过扬声器播放"""
        # 转换为numpy数组并确保是float32格式
        audio_np = audio_tensor.numpy().astype(np.float32)
        
        # 标准化到[-1, 1]范围
        audio_np /= np.max(np.abs(audio_np))
        
        # 通过sounddevice播放
        sd.play(audio_np, samplerate=sample_rate)
        sd.wait()  # 等待播放完成
    
    def _play_audio_thread(self, audio_data):
        """子线程：实际执行音频播放"""
        try:
            sd.play(audio_data, samplerate=22050)
            sd.wait()
        except Exception as e:
            rospy.logerr(f"音频播放异常: {str(e)}")

    def handle_tts_request(self, req):
        try:
            # 调用TTS生成音频（仍在主线程执行）
            audio_data = self.tts_infer(req.text)
            # 启动子线程播放音频（非阻塞）
            threading.Thread(target=self._play_audio_thread, args=(audio_data,)).start()
            # 立即返回成功响应（无需等待播放完成）
            return TTSResponse(success=True, error_msg="")
        except Exception as e:
            return TTSResponse(success=False, error_msg=str(e))

if __name__ == "__main__":
    rospy.init_node("tts_service_node")
    tts_service = TTSService()
    s = rospy.Service("tts_request", TTS, tts_service.handle_tts_request)
    rospy.loginfo("TTS服务已启动，等待请求...")
    rospy.spin()
