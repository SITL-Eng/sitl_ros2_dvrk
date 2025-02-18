from ros2_numpy import point_cloud2

def pcl2nparray(pcl_msg):
    pclarray = point_cloud2.pointcloud2_to_xyz_array(pcl_msg)
    try:
        return pclarray.reshape(-1, 3)
    except:
        print(pclarray.shape)