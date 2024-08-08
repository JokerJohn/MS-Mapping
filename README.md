<div id="top" align="center">
# [MS-Mapping: An Uncertainty-Aware Large-Scale Multi-Session LiDAR Mapping System](https://arxiv.org/abs/2408.03723)

<div align="center">

![image (16)](./README/image%20(16).png)

![cp5-gn-100](./README/cp5-gn-100.gif)

</div>

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT) [![GitHub Stars](https://img.shields.io/github/stars/JokerJohn/MS-Mapping.svg)](https://github.com/JokerJohn/MS-Mapping/stargazers) [![GitHub Issues](https://img.shields.io/github/issues/JokerJohn/MS-Mapping.svg)](https://github.com/JokerJohn/MS-Mapping/issues)<a href="https://github.com/JokerJohn/MS-Mapping/blob/main/"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>

## Table of Contents

[TOC]

## Introduction

**Author**: Xiangcheng Hu, [Jin Wu](https://github.com/zarathustr), [Jianhao Jiao](https://github.com/gogojjh), [Binqian Jiang](https://github.com/lewisjiang), Wei Zhang, [Wenshuo Wang](https://github.com/wenshuowang) and Ping Tan

MS-Mapping is a novel multi-session LiDAR mapping system designed for large-scale environments. It addresses challenges in data redundancy, robustness, and accuracy with three key innovations:

- **Distribution-aware keyframe selection**: Captures the contributions of each point cloud frame by analyzing map distribution similarities. This reduces data redundancy and optimizes graph size and speed.
- **Uncertainty model**: Automatically adjusts using the covariance matrix during graph optimization, enhancing precision and robustness without scene-specific tuning. It monitors pose uncertainty to avoid ill-posed optimizations.
- **Enhanced evaluation**: Redesigned baseline comparisons and benchmarks demonstrate MS-Mapping's superior accuracy over state-of-the-art methods.

Applications include surveying, autonomous driving, crowd-sourced mapping, and multi-agent navigation.

![image-20240730152813297](./README/image-20240730152813297.png)

<div align="center">

| ![image-20240516093525041](./README/image-20240516093525041.png) | ![image-20240730151727768](./README/image-20240730151727768.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

</div>



## News

- **2024/08/08**: We released the first version of MS-Mapping on [arXiv](https://arxiv.org/pdf/2408.03723), together with the example [merged data](http://gofile.me/4jm56/oO3TvgSvl)  and related [YouTube](https://www.youtube.com/watch?v=1z8EOhCmegM) and [bilibili]() videos. 
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

- Use the [python scripts](https://github.com/JokerJohn/SLAMTools/tree/main/Ms_mapping) to get and visualize all the session trajectory and session map, together with the merged map.

![image-20240807101744799](./README/image-20240807101744799.png)

![image-20240807101620547](./README/image-20240807101620547.png)

![image-20240807101845991](./README/image-20240807101845991.png)

![image-20240807110418976](./README/image-20240807110418976.png)

- set the new base map `CP5-CP2` for the next mapping round. You need to add the keyframes of `CP5` into this folder. There must be a `map.pcd ` file in the map folder. Check the keyframes number with the pose files.

  ![image-20240807100001179](./README/image-20240807100001179.png)

## Example

We provide example merged data for 8 sessions [here](http://gofile.me/4jm56/xNhE1scBX), The session order is: ` CP5-CP2-CS1-CC1-PK1-IA3-IA4-NG`. **One must clean the separate map to remove the point cloud noise caused by the glass**, since this study do not focus on this.  The cleaned map also can be [download here](http://gofile.me/4jm56/jyhJf373S). Note that these example data may be updated since it is not the best results.

![image-20240808085525833](./README/image-20240808085525833.png)

![image-20240808085750123](./README/image-20240808085750123.png)

[Plot the results](https://github.com/JokerJohn/SLAMTools/tree/main/Ms_mapping):

```python
#trjectory
python3 tum-trajectory-plotter.py 

#map
pcl_viewer merged_map_session_*
```

![image-20240808090003879](./README/image-20240808090003879.png)

![image-20240808090223073](./README/image-20240808090223073.png)

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
```latex
@misc{hu2024msmapping,
      title={MS-Mapping: Multi-session LiDAR Mapping with Wasserstein-based Keyframe Selection}, 
      author={Xiangcheng Hu, Jin Wu, Jianhao Jiao, Wei Zhang and Ping Tan},
      year={2024},
      eprint={2406.02096},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}

@misc{hu2024msmappinguncertaintyawarelargescalemultisession,
      title={MS-Mapping: An Uncertainty-Aware Large-Scale Multi-Session LiDAR Mapping System}, 
      author={Xiangcheng Hu, Jin Wu, Jianhao Jiao, Binqian Jiang, Wei Zhang, Wenshuo Wang and Ping Tan},
      year={2024},
      eprint={2408.03723},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2408.03723}, 
}
```

## Acknowledgment

The code in this project is adapted from the following projects:

- The odometry  method is adapted from [FAST-LIO2](https://github.com/hku-mars/FAST_LIO).
- The basic framework for pose graph optimization (PGO) is adapted from [SC-A-LOAM](https://github.com/gisbi-kim/SC-A-LOAM).
- The Point-to-Plane method is adapted from [LOAM](https://github.com/laboshinl/loam_velodyne).

## Contributors

<a href="https://github.com/JokerJohn/MS-Mapping/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=JokerJohn/MS-Mapping" />
</a>

## License

This project's code is available under the [MIT LICENSE](./LICENSE).

