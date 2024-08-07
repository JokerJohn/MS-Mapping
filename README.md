<div id="top" align="center">

# MS-Mapping: An Uncertainty-Aware Large-Scale Multi-Session LiDAR Mapping System

<div align="center">

![image (16)](./README/image%20(16).png)

![cp5-gn-100](./README/cp5-gn-100.gif)

</div>

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![GitHub Stars](https://img.shields.io/github/stars/JokerJohn/MS-Mapping.svg)](https://github.com/JokerJohn/MS-Mapping/stargazers)
[![GitHub Issues](https://img.shields.io/github/issues/JokerJohn/MS-Mapping.svg)](https://github.com/JokerJohn/MS-Mapping/issues)

## Table of Contents
- [Introduction](#introduction)
- [News](#news)
- [RUN](#Run)
- [Dataset](#dataset)
- [Results](#results)
- [Citations](#citations)
- [License](#license)

## Introduction

**Author**: Xiangcheng Hu, [Jin Wu](https://github.com/zarathustr), [Jianhao Jiao](https://github.com/gogojjh), [Binqian Jiang](https://github.com/lewisjiang), Wei Zhang, [Wenshuo Wang](https://github.com/wenshuowang) and Ping Tan

**MS-Mapping** presents a novel multi-session LiDAR mapping system that employs an incremental mapping scheme, enabling high-precision and consistent map assembly in large-scale environments. 

![image-20240730152813297](./README/image-20240730152813297.png)

<div align="center">

| ![image-20240516093525041](./README/image-20240516093525041.png) | ![image-20240730151727768](./README/image-20240730151727768.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

</div>

## News

- **2024/07/19**: accepted by [ICRA@40](https://icra40.ieee.org/) as a [extended abstract](https://arxiv.org/pdf/2406.02096).
- **2024/06/03**: submit to a [workshop](https://arxiv.org/html/2406.02096v1).

## Run

Lets take `CP5` as the old session, and use `CP2` to do incremental mapping base on it.

- Build the base map using single-session uncertainty  SLAM. It's important to know the cov of each edge in pose graph. Keyframes number must be the same with the poses number.

![image-20240806191013216](./README/image-20240806191013216.png)

- Set the base map as prior map folder, and preparing for the parameters of Ms-Mapping (initial pose). You can use Cloudcompare to align the first point cloud (at the map folder) with the base map to get the initial pose.  (This problem can be solved when you integrate a place recognition algorithm into our system).

  ![image-20240807094852686](./README/image-20240807094852686.png)

  ![image-20240807094808981](./README/image-20240807094808981.png)


- Run the Ms-Mapping and save the merged data. The keyframe folder  only save the keyframes of the new session data.

![image-20240807095156108](./README/image-20240807095156108.png)

![image-20240807095347540](./README/image-20240807095347540.png)

- Use the [python scripts](https://github.com/JokerJohn/SLAMTools/tree/main/PCD) to get and visulize all the session trajectory and session map, together with the merged map.

![image-20240807101744799](./README/image-20240807101744799.png)

![image-20240807101620547](./README/image-20240807101620547.png)

![image-20240807101845991](./README/image-20240807101845991.png)

![image-20240807110418976](./README/image-20240807110418976.png)

- set the new base map `CP5-CP2` for the next mapping round. You need to add the keyframes of `CP5` into this folder. There must be a `map.pcd ` file in the map folder. Check the keyframes number with the pose files.

  ![image-20240807100001179](./README/image-20240807100001179.png)

## Dataset

#### [Fusion Portable V2 Dataset](https://fusionportable.github.io/dataset/fusionportable_v2/)

#### [Newer College](https://ori-drs.github.io/newer-college-dataset/)

#### [Urban-Nav](https://github.com/IPNL-POLYU/UrbanNavDataset)

#### [MS-Dataset](https://github.com/JokerJohn/MS-Dataset)

![image-20240730151834570](./README/image-20240730151834570.png)



### Trajectory Evaluation

![image-20240711111837423](./README/image-20240711111837423.png)

| ![image-20240730153021873](./README/image-20240730153021873.png) | ![image-20240730153037085](./README/image-20240730153037085.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |



### Map Evaluation

| ![image-20240711111417041](./README/image-20240711111417041.png) | ![image-20240711111504116](./README/image-20240711111504116.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |



<div align="center">


![image-20240730152951528](./README/image-20240730152951528.png)

</div>


<div align="center">

![image-20240730152904480](./README/image-20240730152904480.png)

</div>

### Time Analysis

<div align="center">

![image-20240711111322055](./README/image-20240711111322055.png)
</div>

To plot the results, you can follow this [scripts](https://github.com/JokerJohn/SLAMTools/blob/main/Run_Time_analysis/time_analysis.py).



## Citations

Please cite:
```
@misc{hu2024msmapping,
      title={MS-Mapping: Multi-session LiDAR Mapping with Wasserstein-based Keyframe Selection}, 
      author={Xiangcheng Hu, Jin Wu, Jianhao Jiao, Wei Zhang and Ping Tan},
      year={2024},
      eprint={2406.02096},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## License

This project's code is available under the [MIT LICENSE](./LICENSE).
