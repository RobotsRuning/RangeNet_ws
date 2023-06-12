# RangeNet_ws
# 更改00，01，02，...

std::string kitti_num = "10"; // Replace "01" with the actual value you want <br>
std::string base_path = "/home/fairlee/KITTI/data_odometry/data_odometry_velodyne/" + kitti_num + "/velodyne/";  <br>
std::string file_extension = ".bin";  <br>

 // Calculate the number of files in the directory <br>
int N = std::distance(fs::directory_iterator(base_path), fs::directory_iterator{});  <br>

for(int file_num = 0; file_num < N; ++file_num) {  <br>
