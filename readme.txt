该程序在ROS melodic成功运行
“obstacle_points”
功能：将输入的激光雷达点云与占据地图相匹配，删除与障碍物太近的点，并将过滤后的点云发布出来。
输入topic：“/scan”、“/map”
输出topic：“/object_point_cloud”

“object_tracking”
功能：将输入的点云信息进行聚类，然后与上一帧数据进行关联，再用最小二乘法估计类别的线速度
输入topic：“object_point_cloud”
输出topic：“/object_velocity”、“/point_category”、“rqt_plot_test”

