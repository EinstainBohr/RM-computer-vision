# RM-computer-vision


# 环境说明
    opencv３以上

# 内容说明
  PlanA
    １.main.cpp是传坐标差版本，现场调个阈值就行了，关键部分写了些注释。
　
  PlanB
    2.VersionAnger.cpp是传移动角度的版本
    (1)carmera文件夹是采集标定图像的代码。
    (2)camera_calib是获得相机内参calib.xml的代码。
    (3)自己要标定相机获得calib.xml
    注意：传角度代码存在理论误差，坐标差版本无理论误差。好处是电控有写直接移动到绝对位姿角度的后端处理就快。
    
