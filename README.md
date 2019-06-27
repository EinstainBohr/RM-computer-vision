# RM-computer-vision


# 环境说明
    opencv2以上就可以了

# 内容说明
    main.cpp是关键算法，现场调个阈值就行了，关键部分写了些注释。
# PlanB说明
    VersionAnger.cpp是传移动角度的版本，需要opencv3以上。
    carmera文件夹是采集标定图像的代码。
    camera_calib是获得相机内参calib.xml的代码。
    自己要标定相机获得calib.xml
    提示：传角度代码存在理论误差，坐标差版本无理论误差。好处是电控有写直接运行到特定角度后端处理就快。
