#!/home/agilex/miniconda3/envs/asr/bin/python3
import sys
import threading  # 新增：导入threading模块
print("当前python解释器路径:", sys.executable)
from funasr import AutoModel
import logging
import warnings
import soundfile as sf
import os
import sounddevice as sd
import numpy as np
import queue
import time


# 设置日志同时输出到控制台和文件
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("asr_debug.log"),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger("ASR")

chunk_size = [0, 10, 5] #[0, 10, 5] 600ms, [0, 8, 4] 480ms
encoder_chunk_look_back = 4 #number of chunks to lookback for encoder self-attention
decoder_chunk_look_back = 1 #number of encoder chunks to lookback for decoder cross-attention
chunk_stride = chunk_size[1] * 960 # 600ms

def play_audio(file_path):
    """
    播放音频文件
    :param file_path: 音频文件路径
    """
    data, fs = sf.read(file_path)
    sd.play(data, fs)
    sd.wait()

def test_microphone(save_path='test_mic.wav'):
    """
    测试麦克风是否正常工作。运行此函数后，程序会录制5秒的音频并保存为test_mic.wav文件。
    """
    fs = 16000  # 采样率
    duration = 5  # 录音5秒

    # 可以先测试可用的音频设备
    print("可用音频设备:")
    print(sd.query_devices())
    print(f"默认输入设备: {sd.query_devices(kind='input')}")

    print("开始录音测试，请说话...")
    recording = sd.rec(int(duration * fs), samplerate=fs, channels=1)
    sd.wait()  # 等待录音完成
    print("录音完成")

    # 保存录音文件
    sf.write(f'{save_path}', recording, fs)
    print(f"已保存到{save_path}，请检查文件是否有声音")

    # 显示录音信息
    print(f"录音信息: 形状={recording.shape}, 最小值={np.min(recording)}, 最大值={np.max(recording)}")




class FunASRModel:
    def __init__(self):
        # 建议添加模型路径检查
        
        pwd = os.path.dirname(__file__)
        print(f"当前工作目录: {pwd}")
        model_path = os.path.join(pwd, 
                                  "../speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online")
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found at {model_path}")
        
        self.model = AutoModel(
            model=model_path,  # 使用绝对路径
            disable_update=True,
            disable_pbar=True,
            disable_log=True,
            progress_bar=False,
            verbose=False 
            )
        self.running = False
        self.audio_stream = None
        self.audio_queue = None
        self.infer_thread = None
        self.cpp_callback = None
        
        
    def _safe_callback(self, text):
        if self.cpp_callback:  # 检查回调是否可用
            print(type(self.cpp_callback))
            print(dir(self.cpp_callback))
            print(self.cpp_callback.__class__)

            try:
                print(text)
                self.cpp_callback(text)
                print("回调执行成功")
            except Exception as e:
                print(f"回调执行失败: {repr(e)}")


    def infer_by_wav(self, wav_file=None,debug=False):
        """
        :param wav_file: path to the wav file
        :return: text
        """
        if wav_file is None:
            wav_file = os.path.join(self.model.model_path, "example/asr_example.wav")
        if not os.path.exists(wav_file):
            raise FileNotFoundError(f"File {wav_file} does not exist.")
        if not wav_file.endswith(".wav"):
            raise ValueError(f"File {wav_file} is not a .wav file.")
        if os.path.getsize(wav_file) == 0:
            raise ValueError(f"File {wav_file} is empty.")
        if os.path.getsize(wav_file) > 100 * 1024 * 1024:
            raise ValueError(f"File {wav_file} is too large.")

        # Read the audio file
        speech, sample_rate = sf.read(wav_file)
        if sample_rate != 16000:
            warnings.warn(f"Sample rate is {sample_rate}, but the model expects 16000. Resampling may be needed.")
        
        # Inference

        cache = {}
        text = ''
        total_chunk_num = int((len(speech) - 1) / chunk_stride) + 1
        for i in range(total_chunk_num):
            speech_chunk = speech[i*chunk_stride:(i+1)*chunk_stride]
            is_final = i == total_chunk_num - 1
            res = self.model.generate(input=speech_chunk, cache=cache, 
                                is_final=is_final, chunk_size=chunk_size, 
                                encoder_chunk_look_back=encoder_chunk_look_back, 
                                decoder_chunk_look_back=decoder_chunk_look_back,
                                disable_pbar=True,progress_bar=False,verbose=False
                                )
            # print(res)
            text += res[0]['text']
            if debug:
                print(res[0]['text'])
        
        return text

    def is_running(self):
        return self.running
    def start(self, debug=False):
        """启动线程进行语音识别"""
        if self.running:
            print("ASR 已经在运行")
            return

        self.running = True
        self.infer_thread = threading.Thread(
            target=self.infer_by_microphone,
            kwargs={"debug": debug}
        )
        self.infer_thread.start()

    def infer_by_microphone(self, debug=False):
        """
        :param debug: if True, print the result
        :return: text
        """         
        
        # 配置音频参数
        FS = 16000  # 必须与模型训练采样率一致
        CHANNELS = 1
        DEVICE = None  # 使用默认设备
        self.audio_queue = queue.Queue()
        cache = {}  # 上下文缓存
        current_text = ""
        self.running = True
        
        # 用于调试: 保存所有音频块
        debug_audio_chunks = []
    
        def audio_callback(indata, frames, time, status):
            """音频回调函数"""
            if not self.running:
                raise sd.CallbackStop()
            # 转换为float32格式
            audio_data = indata.flatten().copy().astype(np.float32)
            
            # 如果使用int16，需要转换为模型可以处理的范围
            if indata.dtype == np.int16:
                audio_data = audio_data / 32768.0
            
            self.audio_queue.put(audio_data)
            
        print("开始录音，请说话...\n")
        start_time = time.time()
        try:
            self.audio_stream = sd.InputStream(
                samplerate=FS,
                channels=CHANNELS,
                blocksize=chunk_stride,
                dtype='float32',
                device=DEVICE,
                callback=audio_callback
            )
            self.audio_stream.start() 
            while self.running and threading.current_thread().is_alive():
                try:
                    chunk = self.audio_queue.get(timeout=0.1)
                    if chunk is None:  # 接收到 None，说明 stop() 触发了退出
                        print("[infer] 接收到停止信号，退出识别循环")
                        break
                except queue.Empty:
                    if not self.running:
                        break
                    continue


                # 调试信息和保存音频块
                if debug:
                    print(f"[DEBUG] 获取音频块耗时: {time.time() - start_time:.3f}s")
                    signal_level = np.max(np.abs(chunk))
                    print(f"音频块: 形状={chunk.shape}, 信号强度={signal_level:.6f}")
                    debug_audio_chunks.append(chunk)
                    
                
                # 执行推理
                res = self.model.generate(input=chunk, cache=cache, 
                                is_final=False, chunk_size=chunk_size, 
                                encoder_chunk_look_back=encoder_chunk_look_back, 
                                decoder_chunk_look_back=decoder_chunk_look_back,
                                disable_pbar=(not debug), progress_bar=debug, verbose=False
                                )
                if debug:
                    process_time = time.time() - start_time
                    print(f"[DEBUG] 模型推理耗时: {process_time:.3f}s")
                    
                # 显示识别结果
                if res and len(res) > 0 and 'text' in res[0] and res[0]['text']:
                    new_text = res[0]['text']
                    if new_text != current_text and new_text.strip():
                        current_text += new_text
                        print(current_text + " "*20, end="\r")  # 清除行尾
                        sys.stdout.flush()

                        if self.cpp_callback:
                            self._safe_callback(current_text)
                time.sleep(0.1)  # 避免CPU过载
                    
        except KeyboardInterrupt:
            self.running = False
            print("\n\n录音已停止")
        except Exception as e:
            self.running = False
            print(f"发生错误: {e}")
        finally:
            # 在调试模式下保存收集的音频数据
            if debug and debug_audio_chunks:
                try:
                    # 将所有音频块拼接成一个连续的数组
                    all_audio = np.concatenate(debug_audio_chunks)
                    debug_file_path = f"debug_audio.wav"
                    # 保存为WAV文件
                    sf.write(debug_file_path, all_audio, FS)
                    print(f"\n调试音频已保存到: {debug_file_path}")
                except Exception as e:
                    print(f"\n保存调试音频时出错: {e}")
            
    def stop(self):
        """停止语音识别（增强版）"""
        print("正在停止ASR...")
        self.running = False  # 设置标志位
        
        # 通知 audio_queue 中断阻塞（关键补充！）
        try:
            self.audio_queue.put_nowait(None)
        except Exception as e:
            print(f"[stop] 发送 None 到 audio_queue 失败: {e}")
        
        # 停止音频流
        try:
            if self.audio_stream:
                self.audio_stream.stop()
                self.audio_stream.close()
                self.audio_stream = None
        except Exception as e:
            print(f"关闭音频流失败: {e}")
        
        # 等待线程退出
        if self.infer_thread and self.infer_thread.is_alive():
            self.infer_thread.join(timeout=2.0)
            self.infer_thread = None
        
        # 清空队列（非必须）
        try:
            while not self.audio_queue.empty():
                self.audio_queue.get_nowait()
        except queue.Empty:
            pass

        print("ASR停止完成")



def export_onnx(model:AutoModel=None, output_dir="./onnx_models", quantize=False):
    """
    导出模型为ONNX格式
    :param model: 模型对象
    :param output_dir: 输出目录
    :param quantize: 是否量化
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    if model is None:
        model = AutoModel(
        model="../speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online",
        model_revision="v2.0.4",
        type="paraformer",  # 明确指定模型类型
        export=True  # 启用导出模式
        )
    
    model.export(
        output_dir="./onnx_models",
        type="onnx",         # 导出格式
        quantize=False,      # 是否量化
        verbose=True,
        # 必须指定输入shape [batch, time]
        input_shape=[1, 16000],  # 16k采样率对应1秒音频
        streaming=True,  # 启用流式导出
        chunk_size=16000, # 与模型配置一致
        encoder_chunk_look_back=4,
        decoder_chunk_look_back=1
    )
        
        
    print(f"模型已导出到 {output_dir}")
        
if __name__ == "__main__":
    
    # # 测试代码
    test_microphone()
    model = FunASRModel()
    model.infer_by_microphone(debug=False)
    
    # # 测试wav文件
    # text = model.infer_by_wav(wav_file=None, debug=False)
    # print(f"识别结果: {text}")
    
    # # 测试麦克风
    # model.infer_by_microphone(debug=True)

    # # 导出onnx
    # model = AutoModel(
    #     model="/home/sumi/nav_ws/src/dialogue_panel/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online",
    #     model_revision="v2.0.4",
    #     type="paraformer",  # 明确指定模型类型
    #     export=True  # 启用导出模式
    # )

    # export_onnx(model=model, output_dir="./onnx_models", quantize=False)