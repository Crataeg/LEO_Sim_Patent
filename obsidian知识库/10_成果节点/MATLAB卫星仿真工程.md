---
title: "MATLAB卫星仿真工程"
task_type: "成果"
source_path: "D:\一汽项目\LEO_Sim"
copied_path: "D:\专利自有\成果本身\代码工程\LEO_Sim"
---
# MATLAB卫星仿真工程

## 节点总结

该节点保留 MATLAB 低轨卫星通信 EMC 仿真工程本体，是专利自有线的核心技术成果。

## 写作价值

可直接支撑专利技术效果、系统结构、参数配置、最坏工况搜索与结果论证。

## 原始位置

- `D:\一汽项目\LEO_Sim`

## 复制后位置

- `D:\专利自有\成果本身\代码工程\LEO_Sim`

## 文件规模

- 文件数：`3835`

## 关键文件

- `成果本身\代码工程\LEO_Sim\LEO_Sim_V5_Upgrade.m`
- `成果本身\代码工程\LEO_Sim\LEO_StarNet_EMC_V6_2_SamplerOptimized.m`
- `成果本身\代码工程\LEO_Sim\LEO_StarNet_EMC_V6_3_Route1_InfoGAN_NoPrior.m`
- `成果本身\代码工程\LEO_Sim\LEO_StarNet_EMC_V7_0_Engineering.m`
- `成果本身\代码工程\LEO_Sim\LEO_StarNet_EMC_V7_0_Engineering_Merged.m`
- `成果本身\代码工程\LEO_Sim\run_LEO_EMC_Sim.m`
- `成果本身\代码工程\LEO_Sim\run_V7_default.m`
- `成果本身\代码工程\LEO_Sim\LEO_Sim\build_leo_emc_model.m`
- `成果本身\代码工程\LEO_Sim\v7proj\classifyInterferenceTimeline.m`
- `成果本身\代码工程\LEO_Sim\v7proj\classifyInterferenceTimeline_powerSampler.m`
- `成果本身\代码工程\LEO_Sim\LEO_Sim\config_leo_emc.m`
- `成果本身\代码工程\LEO_Sim\v7proj\defaultPowerAlignedSamplerCfg.m`

## 代表性内容预览

### `成果本身\代码工程\LEO_Sim\LEO_Sim_V5_Upgrade.m`

```text
%% LEO StarNet EMC + GA(GAN) Worst-case + 3D Linked Dashboard (R2021a)  [V6.2 FULL]
% 完全体增强点（在 V6.1 基础上“不改主仿真逻辑”，只补齐你 gen_set/train_lenet 的“图片生成链路”）：
%  A) 生成 STFT 数据集图像（train/val/test, 4类：none/tone/pbnj/mod）
%  B) 导出数据集样例拼图（montage）+ 测试集混淆矩阵
%  C) 导出仿真时间轴关键时刻 STFT 图（用于PPT展示：干扰出现/保护/共信道等时刻）
%  D) Dashboard 仍实时显示分类结果条形图 + 3D联动竖线 + 链路高亮
%
% 依赖：
%  - Satellite Communications Toolbox
%  - Deep Learning Toolbox
%  - Optimization Toolbox (ga)
%  - Image Processing Toolbox（你已安装：im2single/im2gray/imresize/montage等可用）
%
% 输出目录（默认）：
%  - dataset_stft_r2021a/ (train/val/test 四类图片)
%  - dataset_stft_r2021a/_exports/ (样例拼图、混淆矩阵、仿真关键帧STFT图)
 
clear; clc; close all;
rng(7);
 
fprintf('============================================================\n');
fprintf('  LEO StarNet EMC | Worst-case Search (GA on GAN) | R2021a V6.2 FULL\n');
fprintf('============================================================\n');
 
%% =========================
% PART 0: 参数区（可改）
% =========================
Epoch = datetime(2026,1,23,0,0,0,'TimeZone','UTC');   % 固定Epoch：不要用 now
 
Re  = 6371e3;
mu  = 3.986004418e14;
c   = 299792458;
 
% --- 星座 ---
h      = 1200e3;
a      = Re + h;
ecc    = 0.0;
incDeg = 53;
 
numPlanes    = 12;
satsPerPlane = 8;
F_phasing    = 1;
Nsat = numPlanes*satsPerPlane;
 
% --- 仿真时长：一圈轨道 ---
T_orbit = 2*pi*sqrt(a^3/mu);
sample_time = 10;     % 秒
sim_start = Epoch;
sim_stop  = sim_start + seconds(T
```

### `成果本身\代码工程\LEO_Sim\LEO_StarNet_EMC_V6_2_SamplerOptimized.m`

```text
%% LEO StarNet EMC + GA(GAN) Worst-case + 3D Linked Dashboard (R2021a)  [V6.2 FULL]
% 完全体增强点（在 V6.1 基础上“不改主仿真逻辑”，只补齐你 gen_set/train_lenet 的“图片生成链路”）：
%  A) 生成 STFT 数据集图像（train/val/test, 4类：none/tone/pbnj/mod）
%  B) 导出数据集样例拼图（montage）+ 测试集混淆矩阵
%  C) 导出仿真时间轴关键时刻 STFT 图（用于PPT展示：干扰出现/保护/共信道等时刻）
%  D) Dashboard 仍实时显示分类结果条形图 + 3D联动竖线 + 链路高亮
%
% 依赖：
%  - Satellite Communications Toolbox
%  - Deep Learning Toolbox
%  - Optimization Toolbox (ga)
%  - Image Processing Toolbox（你已安装：im2single/im2gray/imresize/montage等可用）
%
% 输出目录（默认）：
%  - dataset_stft_r2021a/ (train/val/test 四类图片)
%  - dataset_stft_r2021a/_exports/ (样例拼图、混淆矩阵、仿真关键帧STFT图)
 
clear; clc; close all;
rng(7);
 
fprintf('============================================================\n');
fprintf('  LEO StarNet EMC | Worst-case Search (GA on GAN) | R2021a V6.2 FULL\n');
fprintf('============================================================\n');
 
%% =========================
% PART 0: 参数区（可改）
% =========================
Epoch = datetime(2026,1,23,0,0,0,'TimeZone','UTC');   % 固定Epoch：不要用 now
 
Re  = 6371e3;
mu  = 3.986004418e14;
c   = 299792458;
 
% --- 星座 ---
h      = 1200e3;
a      = Re + h;
ecc    = 0.0;
incDeg = 53;
 
numPlanes    = 12;
satsPerPlane = 8;
F_phasing    = 1;
Nsat = numPlanes*satsPerPlane;
 
% --- 仿真时长：一圈轨道 ---
T_orbit = 2*pi*sqrt(a^3/mu);
sample_time = 10;     % 秒
sim_start = Epoch;
sim_stop  = sim_start + seconds(T
```

### `成果本身\代码工程\LEO_Sim\LEO_StarNet_EMC_V6_3_Route1_InfoGAN_NoPrior.m`

```text
%% LEO StarNet EMC + GA(GAN) Worst-case + 3D Linked Dashboard (R2021a)  [V6.2 FULL]
% 完全体增强点（在 V6.1 基础上“不改主仿真逻辑”，只补齐你 gen_set/train_lenet 的“图片生成链路”）：
%  A) 生成 STFT 数据集图像（train/val/test, 4类：none/tone/pbnj/mod）
%  B) 导出数据集样例拼图（montage）+ 测试集混淆矩阵
%  C) 导出仿真时间轴关键时刻 STFT 图（用于PPT展示：干扰出现/保护/共信道等时刻）
%  D) Dashboard 仍实时显示分类结果条形图 + 3D联动竖线 + 链路高亮
%
% 依赖：
%  - Satellite Communications Toolbox
%  - Deep Learning Toolbox
%  - Optimization Toolbox (ga)
%  - Image Processing Toolbox（你已安装：im2single/im2gray/imresize/montage等可用）
%
% 输出目录（默认）：
%  - dataset_stft_r2021a/ (train/val/test 四类图片)
%  - dataset_stft_r2021a/_exports/ (样例拼图、混淆矩阵、仿真关键帧STFT图)
 
clear; clc; close all;
rng(7);
 
fprintf('============================================================\n');
fprintf('  LEO StarNet EMC | Worst-case Search (GA on GAN) | R2021a V6.2 FULL\n');
fprintf('============================================================\n');
 
%% =========================
% PART 0: 参数区（可改）
% =========================
Epoch = datetime(2026,1,23,0,0,0,'TimeZone','UTC');   % 固定Epoch：不要用 now
 
Re  = 6371e3;
mu  = 3.986004418e14;
c   = 299792458;
 
% --- 星座 ---
h      = 1200e3;
a      = Re + h;
ecc    = 0.0;
incDeg = 53;
 
numPlanes    = 12;
satsPerPlane = 8;
F_phasing    = 1;
Nsat = numPlanes*satsPerPlane;
 
% --- 仿真时长：一圈轨道 ---
T_orbit = 2*pi*sqrt(a^3/mu);
sample_time = 10;     % 秒
sim_start = Epoch;
sim_stop  = sim_start + seconds(T
```
