# pcl_utility.hpp

#### show_help()

Usageを表示する関数．間違った使い方をしたときに表示させる．

##### out

##### in

| Argument order | Type  | Description               |
| -------------- | ----- | ------------------------- |
| 0              | char* | program_name. **argv[0]** |

---

### load_file()

プログラムに引数として与えられた文字列の中から **PCD, PLY, OBJ** ファイルのいずれかを見つけ出し，

その最初の１ファイルを読み込み，返す関数．

##### out

pcl::PointCloud\<pcl::PointXYZ\>::Ptr 

ファイルから読み込んだ点群データのポインタ．もし，点群が見つからなからなかったときはプログラムを終了させる．

##### in

| Argument order | Type   | Description |
| -------------- | ------ | ----------- |
| 0              | int    | argc        |
| 1              | char** | argv        |

---

### get_obb()

Oriented Bounding Boxを計算し，その結果得られたものを返す．

##### out

std::tuple\<pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ, Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f\>

| Return tuple order | Type            | Description                                |
| ------------------ | --------------- | ------------------------------------------ |
| 0                  | pcl::PointXYZ   | minimum point obb                          |
| 1                  | pcl::PointXYZ   | maximum point obb                          |
| 2                  | pcl::PointXYZ   | Bounding Box's Central position coordinate |
| 3                  | Eigen::Matrix3f | Bounding Box's Rotational matrix           |
| 4                  | Eigen::Vector3f | Major vector                               |
| 5                  | Eigen::Vector3f | Middle vector                              |
| 6                  | Eigen::Vector3f | Minor vector                               |
| 7                  | Eigen::Vector3f | Mass center                                |

##### in

| Argument order | Type                                  | Description               |
| -------------- | ------------------------------------- | ------------------------- |
| 0              | pcl::PointCloud\<pcl::PointXYZ\>::Ptr | souce_cloud_ptr. 入力点群 |

---

### add_wire_flame_cube_to_visualizer()

OBBをpcl::visualization::PC:Visualizerにワイヤフレームで表示させる関数．これを使わないと，ただの箱を表示することになる．

##### in

| Argument order | Type                               | Description                                       |
| -------------- | ---------------------------------- | ------------------------------------------------- |
| 0              | pcl::visualization::PCLVisualizer& | viewer.表示用                                     |
| 1              | pcl::PointXYZ                      | position_obb. OBBのCentral positionを渡す         |
| 2              | Eigen::Matrix3f                    | rotation_matrix_obb. OBBのRotational Matrixを渡す |
| 3              | pcl::PointXYZ                      | max_point_obb. OBBのmaximum pointを渡す           |
| 4              | pcl::PointXYZ                      | min_point_obb. OBBのminimum pointを渡す           |

##### out

----

### add_axis_to_visualizer()

OBBの結果得られた点群の基底をviewerに表示する関数

##### in

| Argument order | Type                               | Description                                 |
| -------------- | ---------------------------------- | ------------------------------------------- |
| 0              | pcl::visualization::PCLVisualizer& | viewer.表示用                               |
| 1              | Eigen::Vector3f                    | center_vector. OBBのMass Centerを渡す．重心 |
| 2              | Eigen::Vector3f                    | major_vector. OBBのMajor vectorを渡す       |
| 3              | Eigen::Vector3f                    | middle_vector. OBBのMiddle vectorを渡す     |
| 4              | Eigen::Vector3f                    | minor_vector. OBBのMinor vectorを渡す       |

#### out

