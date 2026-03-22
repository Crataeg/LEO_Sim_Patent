---
tags:
  - 论文解读
  - STFT
  - LeNet
  - 干扰识别
---

# STFT与干扰识别论文组

## 论文集合
- [07_Gradient_Based_Learning_Applied_to_Document_Recognition_LeNet.pdf](../03_论文库/PDF/07_Gradient_Based_Learning_Applied_to_Document_Recognition_LeNet.pdf)
- [08_Hierarchical_Classification_Method_for_RFI_Recognition_and_Characterization_in_Satcom.pdf](../03_论文库/PDF/08_Hierarchical_Classification_Method_for_RFI_Recognition_and_Characterization_in_Satcom.pdf)
- [09_RF_Based_Low_SNR_Classification_of_UAVs_Using_CNNs.pdf](../03_论文库/PDF/09_RF_Based_Low_SNR_Classification_of_UAVs_Using_CNNs.pdf)
- [10_Modulation_Classification_Through_Deep_Learning_Using_Resolution_Transformed_Spectrograms.pdf](../03_论文库/PDF/10_Modulation_Classification_Through_Deep_Learning_Using_Resolution_Transformed_Spectrograms.pdf)

## 这组论文支撑什么
- STFT 图像化的合理性
- 轻量 CNN / LeNet 做时频图分类的可行性
- 低信噪比、干扰识别、频谱图分类的实际先例

## 与代码的关系
- `generateDatasetSimpleSTFT.m`
  - 生成训练/验证/测试图像数据集
- `trainLeNetSTFT.m`
  - 训练 `LeNet`
- `classifyInterferenceTimeline_powerSampler.m`
  - 从功率级生成 IQ，再转 STFT 图像，再分类
- `exportSimKeyframeSTFT.m`
  - 导出仿真关键帧图像

## 平台里的价值
- 它给 Dashboard 增加了“干扰类型标签层”
- 这样平台不只知道链路变差，还能知道“哪类干扰在主导”
