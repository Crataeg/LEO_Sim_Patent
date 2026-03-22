---
tags:
  - 论文解读
  - LEO
  - 干扰
---

# LEO与干扰共存论文组

## 论文集合
- [01_A_Survey_on_Non_Geostationary_Satellite_Systems_The_Communication_Perspective.pdf](../03_论文库/PDF/01_A_Survey_on_Non_Geostationary_Satellite_Systems_The_Communication_Perspective.pdf)
- [02_LEO_Satellite_Access_Network_Towards_6G_The_Road_to_Space_Coverage.pdf](../03_论文库/PDF/02_LEO_Satellite_Access_Network_Towards_6G_The_Road_to_Space_Coverage.pdf)
- [03_Emerging_NGSO_Constellations_Spectral_Coexistence_with_GSO_Systems.pdf](../03_论文库/PDF/03_Emerging_NGSO_Constellations_Spectral_Coexistence_with_GSO_Systems.pdf)
- [04_Evaluating_S_Band_Interference_Impact_of_Satellite_Systems_on_Terrestrial_Networks.pdf](../03_论文库/PDF/04_Evaluating_S_Band_Interference_Impact_of_Satellite_Systems_on_Terrestrial_Networks.pdf)
- [05_Null_Shaping_for_Interference_Mitigation_in_LEO_Satellites.pdf](../03_论文库/PDF/05_Null_Shaping_for_Interference_Mitigation_in_LEO_Satellites.pdf)

## 这组论文支撑什么
- `satelliteScenario` 对应的 LEO / NGSO 场景合理性
- 频谱复用、共信道干扰、跨系统共存
- jammer 或定向抑制的干扰控制思路

## 与代码的关系
- `LEO_StarNet_EMC_V7_0_Engineering.m:80-156` 需要它们支撑星座、接入和时变几何背景
- `:199-279` 和 `simulateStarNetV7.m` 需要它们支撑干扰、频谱共存和链路退化理解

## 推荐阅读顺序
1. 先看 `01` 和 `02`，建立 LEO/NGSO 通信系统全貌
2. 再看 `03` 和 `04`，进入干扰与共存问题
3. 最后看 `05`，理解平台里 jammer / anti-jam 相关设定的研究背景
