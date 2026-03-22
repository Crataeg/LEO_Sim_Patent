---
title: "一汽项目_卫星EMC原型工程与数据集"
task_type: "成果"
source_path: "D:\一汽项目"
copied_path: "D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集"
---
# 一汽项目_卫星EMC原型工程与数据集

## 节点总结

该节点汇总一汽项目中的 Simulink 原型工程、根目录系统模型、STFT 数据集和训练模型，是专利自有线继承更早期工程实现的重要补充。

## 写作价值

可支撑专利中对原型验证链、识别模块、数据集构建与训练依据的补充说明。

## 原始位置

- `D:\一汽项目`

## 来源条目

- `LEO_EMC_Sim`
- `dataset_stft_r2021a`
- `GAN_Jammer_R2021a.mat`
- `lenet_stft_model_r2021a.mat`
- `build_LEO_EMC_Sim.m`
- `LEO_EMC_System.slx`
- `LEO_EMC_System.slx.autosave`
- `LEO_EMC_System.slxc`
- `slprj`

## 复制后位置

- `D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集`

## 文件规模

- 文件数：`3759`

## 关键文件

- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\build_LEO_EMC_Sim.m`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\build_LEO_EMC_Sim.m`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\cfo_comp.m`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\interf_gen.m`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\LEO_Sim_V1.m`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\preamble_insert.m`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\run_LEO_EMC_Sim.m`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\LEO_EMC_Sim_说明报告.docx`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\LEO_EMC_仿真技术_主流方法与文献支撑_说明报告.docx`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\.vscode\settings.json`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\.vscode\tasks.json`
- `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\dataset_stft_r2021a\_exports\confusion_test.png`

## 代表性内容预览

### `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\build_LEO_EMC_Sim.m`

```text
function build_LEO_EMC_Sim()
% LEO EMC 通信视角 Simulink 自动建模脚本（Comm Toolbox + Stateflow 写 Script）
% R2021a兼容：MATLAB Function block 在Simulink层面可能表现为SubSystem；需通过 Stateflow.EMChart.Script 写入代码

assert(license('test','communication_toolbox')==1, "Communications Toolbox 未授权");
assert(license('test','stateflow')==1, "Stateflow 未授权（MATLAB Function block 需要）");

model = 'LEO_EMC_Sim_Simulink';
if bdIsLoaded(model); close_system(model,0); end
if exist([model '.slx'],'file'); delete([model '.slx']); end

new_system(model);
open_system(model);

cfg = defaultCfg();
assignin('base','cfg',cfg);

set_param(model,'Solver','FixedStepDiscrete');
set_param(model,'FixedStep', num2str(cfg.sim.Ts));
set_param(model,'StopTime',  cfg.sim.StopTime);

% 顶层子系统
add_block('built-in/Subsystem',[model '/TX'],          'Position',[80 80 260 260]);
add_block('built-in/Subsystem',[model '/Channel_EMC'], 'Position',[320 80 540 260]);
add_block('built-in/Subsystem',[model '/RX'],          'Position',[600 80 820 260]);
add_block('built-in/Subsystem',[model '/Metrics'],     'Position',[880 80 1120 260]);

buildTX([model '/TX']);
buildChannelEMC([model '/Channel_EMC']);
buildRX([model '/RX']);
buildMetrics([model '/Metrics']);

set_param(model,'SimulationCommand','update');

% 顶层连线
add_line(model,'TX/1','Channel_EMC/1','autorouting','on');
add_line(model,'Channel_EMC/1','RX/1','autorouting','on');
add_line(model,'RX/1
```

### `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\build_LEO_EMC_Sim.m`

```text
function build_LEO_EMC_Sim()
% build_LEO_EMC_Sim
% 一键生成 Simulink 模型：LEO卫星通信 EMC（通信视角）仿真骨架
%
% 依赖（推荐）：
%  - Simulink
%  - Communications Toolbox（用于QPSK调制/解调、卷积码/Viterbi、误码统计等模块）
%
% 生成：
%  - LEO_EMC_Sim_Simulink.slx

model = 'LEO_EMC_Sim_Simulink';
if bdIsLoaded(model); close_system(model,0); end
if exist([model '.slx'],'file'); delete([model '.slx']); end

new_system(model);
open_system(model);

% ---------- 全局参数（写入 Base Workspace，方便外行只改这里） ----------
cfg = defaultCfg();
assignin('base','cfg',cfg);

% ---------- 画布基础设置 ----------
set_param(model,'StopTime','cfg.sim.StopTime');
set_param(model,'Solver','FixedStepDiscrete');
set_param(model,'FixedStep','cfg.sim.Ts');

% ---------- 添加子系统：TX / Channel+EMC / RX / Metrics ----------
add_block('built-in/Subsystem',[model '/TX'], 'Position',[80 80 240 240]);
add_block('built-in/Subsystem',[model '/Channel_EMC'], 'Position',[320 80 520 240]);
add_block('built-in/Subsystem',[model '/RX'], 'Position',[600 80 760 240]);
add_block('built-in/Subsystem',[model '/Metrics'], 'Position',[840 80 1020 240]);

% 顶层连接：TX -> Channel_EMC -> RX -> Metrics

% ---------- 构建 TX ----------
buildTX([model '/TX']);

% ---------- 构建 Channel + EMC ----------
buildChannelEMC([model '/Channel_EMC']);

% ---------- 构建 RX ----------
buildRX([model '/RX']);

% ---------- 构建 Metrics ----------
buildMetrics([model '/Metrics']);
add_line(model,'TX/1','Channel_EMC/1','
```

### `成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\cfo_comp.m`

```text
function y = cfo_comp(r)
cfg = evalin('base','cfg');

rx = r(:);
N = length(rx);
Rs = cfg.phy.Rs;
t = (0:N-1).';

if cfg.rx.cfoMethod == 1
    cfo_hat = cfg.leo.fD_Hz;
else
    L = cfg.phy.preambleHalfLen;
    if N < 2*L
        cfo_hat = 0;
    else
        r1 = rx(1:L);
        r2 = rx(L+1:2*L);
        P = sum(conj(r1).*r2);
        cfo_hat = angle(P) * Rs / (2*pi*L);
    end
end

y = rx .* exp(-1j*2*pi*cfo_hat*t/Rs);
end

```

## 原文-工程行级对应与分析

### 1. 本地复制的原型工程 -> 自动建模主入口
- 原文抓手：该节点内复制的 `LEO_EMC_Sim` 和根目录 `build_LEO_EMC_Sim.m`。
- 工程对应：`D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\build_LEO_EMC_Sim.m:1-20`；`D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\build_LEO_EMC_Sim.m:47-68`；`D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\build_LEO_EMC_Sim.m:107-182`
- 分析：专利自有线里的这批原型资产并非备份，而是保留了自动搭建 Simulink 通信链、配置物理层/干扰层和输出指标层的最小实现。

### 2. 本地复制的干扰与补偿算法
- 原文抓手：节点内 `cfo_comp.m`、`interf_gen.m`、`run_LEO_EMC_Sim.m`、`preamble_insert.m`。
- 工程对应：`D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\interf_gen.m:1-37`；`D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\cfo_comp.m:1-23`；`D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\run_LEO_EMC_Sim.m:1-15`；`D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\preamble_insert.m:1-25`
- 分析：这几份文件构成了专利自有线最值得继承的“干扰注入-接收补偿-参数入口-帧结构”四件套，是以后定义第二个专利实施例时最容易被复用的代码块。

### 3. 本地复制的 V1 链路模型 -> 物理层证据链
- 原文抓手：节点内 `LEO_Sim_V1.m`。
- 工程对应：`D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\LEO_Sim_V1.m:123-178`；`D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\LEO_Sim_V1.m:229-321`；`D:\专利自有\成果本身\代码工程\一汽项目_卫星EMC原型工程与数据集\LEO_EMC_Sim\LEO_Sim_V1.m:585-634`
- 分析：V1 代码把场景、C/N/I、干扰波形和蒙特卡罗链路评价明确写成了可复现实验，是专利自有线提炼“测试方法/系统功能模块”时最底层的物理证据。

### 4. 数据集与训练模型在工程中的真实位置
- 原文抓手：该节点同时包含 `dataset_stft_r2021a`、`GAN_Jammer_R2021a.mat`、`lenet_stft_model_r2021a.mat`。
- 工程对应：`D:\论文卫星\成果本身\代码工程\LEO_Sim\v7proj\generateDatasetSimpleSTFT.m:1-20`；`D:\论文卫星\成果本身\代码工程\LEO_Sim\v7proj\trainLeNetSTFT.m:1-45`；`D:\论文卫星\成果本身\代码工程\LEO_Sim\v7proj\classifyInterferenceTimeline_powerSampler.m:1-84`
- 分析：专利自有节点里虽然复制了数据资产，但这些资产的生成、训练和在线使用逻辑仍然在卫星主工程侧完成，因此它们在本节点里的意义是“可继承训练底座”，而不是独立算法实现。
