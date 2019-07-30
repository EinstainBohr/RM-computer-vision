# RM-computer-vision


# 环境说明
    opencv３以上

# 内容说明

# PlanA
    main.cpp是传坐标差版本，现场调个阈值就行了，关键部分写了些注释。
　
# PlanB

    VersionAnger.cpp是传移动角度的版本
    carmera文件夹是采集标定图像的代码。
    camera_calib是获得相机内参calib.xml的代码。
    自己要标定相机获得calib.xml
    注意：传角度代码存在理论误差，坐标差版本无理论误差。好处是电控有写直接移动到绝对位姿角度的后端处理就快。
    

using_network和pb结尾的文件还有test图片是深度学习算法用的，由于没有实用价值请无视。
    
