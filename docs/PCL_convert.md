# How to convert from ROS msgs to PCL Point Cloud

ROSでPCL関連のパッケージはperception_rosに集まっています．
Tutorialに簡単な使い方が記載されていますが，ここでは型に
注目してください．

ROSでは点群は**sensor_msgs/PointCloud2**型になっていますが，
PCLで点群の処理を行うためには**pcl::PointCloud**型である必要があります．
このための相互変換には，**pcl::fromROSMsg**,**pcl::toROSMsg**を
使います．

## How to analyze Point Cloud format
**sensor_msgs/PointCloud2**型から点群の素性を得るためには，
**width**,**height**,**fields**を見てみましょう．
前２つはそのままですが，次の**fields**は事前に調べておくと有用です．
その点群がどのような情報を持っているかを調べられます．

!!! TODO 
	realsense で確かめる

例えば，realsense D435の点群の**fields**を調べてみましょう．

```
rostopic echo /realsense2/points/fields
```

すると，以下のような情報が得られます．

```
-
 name: x
 offset: 0
 datatype: 7
 count: 1
-
 name: y
 offset: 4
 datataype: 7
 count: 1
-
 name: z
 offset: 8
 datatype: 7
 count: 1
-
 name: rgb
 offset:16
 datatype: 7
 count: 1
```
これは，realsenseの点群がx,y,zの位置情報と，rgbの色情報を持っていることを表しています．
[Commonn PointCloud2 field names](http://wiki.ros.org/pcl/Overview#Common_PointCloud2_field_names)に種類が記載されていますが，この他にも点群は法線情報**normal_{x,y,z}**などを持つことができます．
