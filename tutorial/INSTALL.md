### Baseline: F2F or M2F

Note that: 

- the simplest way to quickly run is that directly download my **old session results** + **new session bag (`RB2`)**, performing F2F.
- harder way is   download my **old session bag (`PK1`)** + **new session bag (`RB2`)**,  single session mode to build the old session data of `PK1`. Then use the new session bag to incremental mapping on `PK1`.
- The hardest way is to adapt to your own data.

### Minimal Case with 2 session data 

1. download data. For demo results you can directly download the [old session results](https://hkustconnect-my.sharepoint.com/:u:/g/personal/xhubd_connect_ust_hk/EcoaRBlVdEhMkB4z0jyHkmQBO2feRKSono_fSsVkkCZNOg?e=a8S0SB).

   | [old session bag (`PK1`)](https://ramlab-ust.direct.quickconnect.to:5001/sharing/t9SM1iPZr) | [new session bag (`RB2`)](https://hkustconnect-my.sharepoint.com/personal/xhubd_connect_ust_hk/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fxhubd%5Fconnect%5Fust%5Fhk%2FDocuments%2Fdataset%2Fpaloc%2FParkinglot%2DRedBird%2D2023%2D10%2D28%2D19%2D09%2D04%2Ezip&parent=%2Fpersonal%2Fxhubd%5Fconnect%5Fust%5Fhk%2FDocuments%2Fdataset%2Fpaloc&ga=1) | [old session results](https://hkustconnect-my.sharepoint.com/:u:/g/personal/xhubd_connect_ust_hk/EcoaRBlVdEhMkB4z0jyHkmQBO2feRKSono_fSsVkkCZNOg?e=a8S0SB) |
   | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
   | ![image (17)](./INSTALL/image%20(17).png)                    | ![image-20250116000038243](./INSTALL/image-20250116000038243.png) | ![image-20250223170438039](./INSTALL/image-20250223170438039.png) |

 You can also use the single session mode to generate the map data of the [old session](https://ramlab-ust.direct.quickconnect.to:5001/sharing/t9SM1iPZr), just set parameters `useMultiMode` as `false`. Check the map folder, three files must exists, `key_point_frame`, `pose_graph.g2o` and `optimized_poses_tum.txt`.

![image-20250224125924133](./INSTALL/image-20250224125924133.png)

2. set important parameters of file path: `save_directory`, `map_directory`,`bag_path`.

   ```launch
   <param name="save_directory" type="string" value="/home/xchu/data/pose_slam_prior_result/"/>
   <param name="map_directory" type="string" value="/home/xchu/data/prior_map/PK01/"/>
   
   <!--set your data bag path-->
   <arg name="bag_path" default="/media/xchu/新加卷/HKUSTGZ-DATASET/2023-10-28-19-09-04-Parkinglot-RedBird02/Parkinglot-RedBird-2023-10-28-19-09-04.bag"/>
   ```

2. run launch file, blue trajectoty for the old session and red for the new session.

```
roslaunch ms_mapping ms.launch
```

![image-20250224012231920](./INSTALL/image-20250224012231920.png)

only save data for the new session part, finally use the [python scripts](https://github.com/JokerJohn/SLAMTools/tree/main/Ms_mapping) to get the merged map for analysis.

```
rosservice call /save_map
```

![image-20250223172305661](./INSTALL/image-20250223172305661.png)

### Map Merging With the third session

| [RB3 data bag](https://github.com/JokerJohn/PALoc)           | [PK1-RB2 results](https://hkustconnect-my.sharepoint.com/:u:/g/personal/xhubd_connect_ust_hk/ERsuQfkHh8NEsK2qMfkubngBQuPrWqbxNXD_W6hG08IK_g?e=vdGzgn) | [PK1-RB2-RB3 results](https://hkustconnect-my.sharepoint.com/:u:/g/personal/xhubd_connect_ust_hk/Ef1WFIyW5nBNnKcWt_MKstkBWfKiRrSmoqw2x5IFJwVqyA?e=2VTfhe) |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image-20250122200925827](./INSTALL/image-20250122200925827.png) | ![image-20250327004149392](./INSTALL/image-20250327004149392.png) | ![image-20250327004443119](./INSTALL/image-20250327004443119.png) |

1. Now we have merging `RB02` based on `PK01`, we can directly get the necessary data. We have **3760** key-frames with **8820** poses. Note that the `final_map_lidar.pcd` represents the merged map of `PK1` and `RB2`, but only the `RB2` part.

![image-20250326233529179](./INSTALL/image-20250326233529179.png)

**We need manually copy the keyframes of `PK01` into the `PK1-RB2-TEST/key_point_frame` folder,** then the number of poses  is equal to the keyframes.  (you can directly download my data here)

![image-20250326234001201](./INSTALL/image-20250326234001201.png)

use python scripts [multi-session-map-merger2.py](https://github.com/JokerJohn/SLAMTools/blob/main/Ms_mapping/multi-session-map-merger2.py)  to get the merged map of `PK1` and `RB2`. And move it to 

`PK1-RB2-TEST` folder, renamed as `map.pcd`.

![image-20250327001311697](./INSTALL/image-20250327001311697.png)

![image-20250327001450561](./INSTALL/image-20250327001450561.png)

2. set parameters in `ms.launch`. `map_directory`  set as the new folder `PK1-RB2-TEST`.

```yaml
<param name="map_directory" type="string" value="/home/xchu/data/pose_slam_prior_result/PK1-RB2-TEST/"/>

<arg name="bag_path" default="/media/xchu/新加卷/HKUSTGZ-DATASET/2023-11-26_redbird_03/2023-11-26-19-36-49.bag"/>

<arg name="sequence" default="PK1-RB2-RB3-TEST"/>
```

3. get the initial pose. Directly run the `ms.launch`and quickly stop it to get the first point cloud of `PK01` and map.

```launch
roslaunch ms_mapping ms.launch
```

![image-20250326235544075](./INSTALL/image-20250326235544075.png)

set the initial pose in `ms.yaml`

![image-20250326235800371](./INSTALL/image-20250326235800371.png)

run `ms.launch` again, and this time it will succeed!

```launch
roslaunch ms_mapping ms.launch
```

![image-20250327001529771](./INSTALL/image-20250327001529771.png)

save results

```launch
 rosservice call /save_map 
```

4. use python scripts [multi-session-map-merger2.py](https://github.com/JokerJohn/SLAMTools/blob/main/Ms_mapping/multi-session-map-merger2.py)  to get the merged map again.

   ![image-20250327003221482](./INSTALL/image-20250327003221482.png)

   ![image-20250327003010034](./INSTALL/image-20250327003010034.png)

use [scripts](https://github.com/JokerJohn/SLAMTools/blob/main/Ms_mapping/multi-session-map-merger_writetum.py)  ` multi-session-map-merger_writetum.py` to generate the separate trjectory

```python
python3 multi-session-map-merger_writetum.py
```

![image-20250327003352783](./INSTALL/image-20250327003352783.png)

use [scripts](https://github.com/JokerJohn/SLAMTools/blob/main/Ms_mapping/tum-trajectory-plotter.py) ` tum-trajectory-plotter.py`  plot the results:

```python3
python3 tum-trajectory-plotter.py
```

![image-20250327003453188](./INSTALL/image-20250327003453188.png)

| ![image-20250327003531228](./INSTALL/image-20250327003531228.png) | ![image-20250327003633859](./INSTALL/image-20250327003633859.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |



### 8 Sessions Demo Results

Lets take `CP5` as the old session, and use `CP2` to do incremental mapping base on it.

- Build the base map using single-session uncertainty  SLAM. It's important to know the cov of each edge in pose graph. Keyframes number must be the same with the poses number.

![image-20240806191013216](./INSTALL/image-20240806191013216.png)

- Set the base map as prior map folder, and preparing for the parameters of Ms-Mapping (initial pose). You can use Cloudcompare to align the first point cloud (at the map folder) with the base map to get the initial pose.  (This problem can be solved when you integrate a place recognition algorithm into our system).

  ![image-20240807094852686](./INSTALL/image-20240807094852686.png)

  ![image-20240807094808981](./INSTALL/image-20240807094808981.png)


- Run the Ms-Mapping and save the merged data. The keyframe folder  only save the keyframes of the new session data.

![image-20240807095156108](./INSTALL/image-20240807095156108.png)

![image-20240807095347540](./INSTALL/image-20240807095347540.png)

- Use the [python scripts](https://github.com/JokerJohn/SLAMTools/tree/main/Ms_mapping) to get and visualize all the session trajectory and session map, together with the merged map.

![image-20240807101744799](./INSTALL/image-20240807101744799.png)

![image-20240807101620547](./INSTALL/image-20240807101620547.png)

| ![image-20240807101845991](./INSTALL/image-20240807101845991.png) | ![image-20240807110418976](./INSTALL/image-20240807110418976.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |



- set the new base map `CP5-CP2` for the next mapping round. You need to add the keyframes of `CP5` into this folder. There must be a `map.pcd ` file in the map folder. Check the keyframes number with the pose files.

  ![image-20240807100001179](./INSTALL/image-20240807100001179.png)

## Example

We provide example merged data for 8 sessions [here](http://gofile.me/4jm56/xNhE1scBX), The session order is: ` CP5-CP2-CS1-CC1-PK1-IA3-IA4-NG`. **One must clean the separate map to remove the point cloud noise caused by the glass**, since this study do not focus on this.  The cleaned map also can be [download here](http://gofile.me/4jm56/jyhJf373S). Note that these example data may be updated since it is not the best results.

![image-20240808085525833](./INSTALL/image-20240808085525833-1740306786482-1.png)

![image-20240808085750123](./INSTALL/image-20240808085750123-1740306786482-2.png)

[Plot the results](https://github.com/JokerJohn/SLAMTools/tree/main/Ms_mapping):

```python
#trjectory
python3 tum-trajectory-plotter.py 

#map
pcl_viewer merged_map_session_*
```

| ![image-20240808090003879](./INSTALL/image-20240808090003879.png) | ![image-20240808090223073](./INSTALL/image-20240808090223073.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

