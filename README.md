# object_segmenter
Package for segmenting object from a point cloud using point indices.


## Usage of package

### Prerequisites
Prerequisites for using package:

* point cloud
* file with point indices of objects (txt)

Example of file with point indices:

```
91870
91871
92509
92510
92511
...
```


### Run package
Add package to catkin_ws.
Build catkin workspace:

```
cd ~/catkin_ws
catkin_make
```

Source catkin workspace:


```
source devel/setup.bash
```

Run package:

```
rosrun object_segmenter segment_object <point_cloud>.pcd --indices <indices_file>.txt
```

Result of work:

![object_segmenter_result.png](https://bitbucket.org/repo/KrML5Lj/images/4172871832-object_segmenter_result.png)
![ScreenShot](https://raw.github.com/vovaekb/object_segmenter/master/images/object_segmenter_result.png)

Test example of point cloud is located in directory **test**.
