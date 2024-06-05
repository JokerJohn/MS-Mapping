<div id="top" align="center">

# MS-Mapping: Multi-session LiDAR Mapping with Wasserstein-based Keyframe Selection and Balanced Pose Graph

![image-20240516093245914](./README/image-20240516093245914.png)
</div>

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![GitHub Stars](https://img.shields.io/github/stars/JokerJohn/MS-Mapping.svg)](https://github.com/JokerJohn/MS-Mapping/stargazers)
[![GitHub Issues](https://img.shields.io/github/issues/JokerJohn/MS-Mapping.svg)](https://github.com/JokerJohn/MS-Mapping/issues)

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

![image-20240516093525041](./README/image-20240516093525041.png)
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

Please cite:
```
@misc{BibEntry2024Jun,
	title = {{MS-Mapping: Multi-session LiDAR Mapping with Wasserstein-based Keyframe Selection}},
	journal = {arXiv},
	year = {2024},
	month = jun,
	note = {[Online; accessed 5. Jun. 2024]},
	url = {https://arxiv.org/html/2406.02096v1}
}
```

## License

This project's code is available under the [MIT LICENSE](./LICENSE).
