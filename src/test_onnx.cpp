#include <onnxruntime_cxx_api.h>
#include <vector>
#include <iostream>
#include <sndfile.h>

int main() {
    // 初始化ONNX环境
    Ort::Env ort_env(ORT_LOGGING_LEVEL_WARNING, "ASRTest");
    Ort::SessionOptions session_options;
    
    // 加载模型（路径与export_onnx.py中的output_dir一致）
    std::string model_path = "/home/sumi/nav_ws/src/dialogue_panel/onnx_models/model.onnx";
    Ort::Session session(ort_env, model_path.c_str(), session_options);

    // 加载测试音频（需与Python端相同的预处理）
    SF_INFO sfinfo;
    SNDFILE* wav_file = sf_open("test.wav", SFM_READ, &sfinfo);
    std::vector<float> audio_data(sfinfo.frames * sfinfo.channels);
    sf_read_float(wav_file, audio_data.data(), audio_data.size());
    sf_close(wav_file);

    // 创建输入tensor（匹配模型输入shape [1, 16000]）
    std::vector<int64_t> input_shape = {1, 16000};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU),
        audio_data.data(),
        audio_data.size(),
        input_shape.data(),
        input_shape.size()
    );

    // 执行推理
    const char* input_names[] = {"input"};
    const char* output_names[] = {"output"};
    auto outputs = session.Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input_tensor,
        1,
        output_names,
        1
    );

    // 解析输出（根据实际模型输出结构调整）
    float* result = outputs[0].GetTensorMutableData<float>();
    std::cout << "识别结果: " << result << std::endl;

    return 0;
}
