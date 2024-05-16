<div id="top" align="center">

# MS-Mapping: Multi-session LiDAR Mapping with Wasserstein-based Keyframe Selection and Balanced Pose Graph

![image-20240516093245914](./README/image-20240516093245914.png)
</div>

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![GitHub Stars](https://img.shields.io/github/stars/JokerJohn/PALoc.svg)](https://github.com/JokerJohn/MS-Mapping/stargazers)
[![GitHub Issues](https://img.shields.io/github/issues/JokerJohn/PALoc.svg)](https://github.com/JokerJohn/MS-Mapping/issues)

## Table of Contents

- [Introduction](#introduction)
- [News](#news)
- [Dataset](#dataset)
- [Results](#results)
- [Citations](#citations)
- [License](#license)

## Introduction

**MS-Mapping** presents a novel multi-session LiDAR mapping system that employs an incremental mapping scheme and flexibly supports various LiDAR-based odometry front-ends, enabling high-precision and consistent map assembly in large-scale environments. 
<div align="center">

[image-20240516093525041](./README/image-20240516093525041.png)
</div>

## News

- **2024/05/20**: submit to a journal.

## Dataset

### [Fusion Portable V2 Dataset](https://fusionportable.github.io/dataset/fusionportable_v2/)

Our algorithms were rigorously tested on the [Fusion Portable V2 Dataset](https://fusionportable.github.io/dataset/fusionportable_v2//). 

### Self-collected Dataset



### Map Evaluation

<div align="center">

![image-20240516093903006](./README/image-20240516093903006.png)
</div>

![image-20240516094035919](./README/image-20240516094035919.png)

### Time Analysis

<div align="center">

![image-20240516093925114](./README/image-20240516093925114.png)
</div>

To plot the results, you can follow this [scripts](https://github.com/JokerJohn/SLAMTools/blob/main/Run_Time_analysis/time_analysis.py).



## Citations

The map evaluation metrics of this work follow [Cloud_Map_Evaluation](https://github.com/JokerJohn/Cloud_Map_Evaluation). Please cite:
```
@article{jiao2024fp,
  author    = {Jianhao Jiao and Hexiang Wei and Tianshuai Hu and Xiangcheng Hu and Yilong Zhu and Zhijian He and Jin Wu and Jingwen Yu and Xupeng Xie and Huaiyang Huang and Ruoyu Geng and Lujia Wang and Ming Liu},
  title     = {FusionPortable: A Multi-Sensor Campus-Scene Dataset for Evaluation of Localization and Mapping Accuracy on Diverse Platforms},
  booktitle = {2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2022}
}
```

## License

This project's code is available under the [MIT LICENSE](./LICENSE).
