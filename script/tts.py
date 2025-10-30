import sys
print("当前python解释器路径:", sys.executable)
from kokoro import KModel, KPipeline
import torch
import json
import os

import sounddevice as sd  # 新增音频播放库
import numpy as np

class KororoTTS():
    def __init__(self, model_path="./Kokoro-82M-v1.1-zh"):

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
            
        self.play_audio(audio)  # 播放生成的音频
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

# 使用示例
if __name__ == "__main__":
    tts = KororoTTS()
    audio = tts.tts_infer("你好，这是一条测试语音")
    tts.play_audio(audio)  # 播放生成的音频