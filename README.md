/* Copyright (c) 2023 lifeiya, Chongqing University.
 *
 *  This file is part of advanced rangenet_lib.
 *
 */
 
# RangeNet_ws
# 更改00，01，02，...
# 更改相关路径

// opencv stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz.hpp>

// c++ stuff
#include <chrono>
#include <iomanip>  // for setfill
#include <iostream>
#include <string>
#include <sstream>

// net stuff
#include <selector.hpp>
namespace cl = rangenet::segmentation;

// boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

typedef std::tuple< u_char, u_char, u_char> color;


int main(int argc, const char *argv[]) {
  // define options
  std::string scan;
  std::string path;
  std::string backend = "tensorrt";
 // 如果verbose为true，则程序会输出更多的运行过程信息，如果为false，则只输出最基本的信息。
  bool verbose = false;
  std::ostringstream scanStream;

  std::string kitti_num = "10"; // Replace "01" with the actual value you want
  std::string base_path = "/home/fairlee/dataset/KITTI/sequences_kitti_00_21/" + kitti_num + "/velodyne/";
  std::string file_extension = ".bin";

 // Calculate the number of files in the directory
  int N = std::distance(fs::directory_iterator(base_path), fs::directory_iterator{});

for(int file_num = 1000; file_num < N; ++file_num) {



// cout<<"正在处理-----"<<file_num<<"/"<< N <<"数据"<<endl;

std::cout << std::left << "正在处理 " <<kitti_num<< " 数据集中的第： "<< file_num << " / "  << N << " 帧数据" << std::endl;


  // Parse options
  try {
    po::options_description desc{"Options"};
    desc.add_options()("help,h", "Help screen")(
        "scan,s", po::value<std::string>(&scan),
        "LiDAR scan to infer. No Default")(
        "path,p", po::value<std::string>(),
        "Directory to get the inference model from. No default")(
        "verbose,v", po::bool_switch(),
        "Verbose mode. Calculates profile (time to run)");

    po::variables_map vm;
    po::store(parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    std::cout << std::setfill('=') << std::setw(80) << "" << std::endl;

    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 0;
    }


 std::ostringstream scanStream;
    scanStream << base_path << std::setfill('0') << std::setw(6) << file_num << file_extension;
    scan = scanStream.str();

    // make defaults count, parameter check, and print
      path = "/home/fairlee/darknet53/";

    if (vm.count("verbose")) {
      verbose = vm["verbose"].as<bool>();
      std::cout << "verbose: " << verbose << std::endl;
    } else {
      std::cout << "verbose: " << verbose << ". Using default!" << std::endl;
    }

    std::cout << std::setfill('=') << std::setw(80) << "" << std::endl;
  } catch (const po::error &ex) {
    std::cerr << ex.what() << std::endl;
    return 1;
  }

  // create a network
  std::unique_ptr<cl::Net> net = cl::make_net(path, backend);

  // set verbosity
  net->verbosity(verbose);

  // predict each image
  std::cout << std::setfill('=') << std::setw(80) << "" << std::endl;
  std::cout << "Predicting image: " << scan << std::endl;

  // Open a scan
  std::ifstream in(scan.c_str(), std::ios::binary);
  if (!in.is_open()) {
      std::cerr << "Could not open the scan!" << std::endl;
      return 1;
  }

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (4 * sizeof(float));
  in.seekg(0, std::ios::beg);

  std::vector<float> values(4 * num_points);
  in.read((char*)&values[0], 4 * num_points * sizeof(float));

  // predict
  std::vector<std::vector<float>> semantic_scan = net->infer(values, num_points);

  // get point cloud
  std::vector<cv::Vec3f> points = net->getPoints(values, num_points);

  // get color mask
  std::vector<cv::Vec3b> color_mask = net->getLabels(semantic_scan, num_points);


   // Create output filename
    std::ostringstream outfileNameStream;
    outfileNameStream << "/home/fairlee/dataset/KITTI/sequences_kitti_00_21/" << kitti_num << "/RangeNet_point/" << std::setfill('0') << std::setw(6) << file_num << ".txt";
    std::string outfileName = outfileNameStream.str();

    // Create an ofstream object
    std::ofstream outfile(outfileName);

    if (!outfile) {
        std::cerr << "Unable to open output file: " << outfileName << std::endl;
        return 1;
    }

    // Iterate through each point and corresponding color
    for (size_t i = 0; i < points.size(); ++i) {
        // Write the point coordinates and color to the file
        outfile << points[i][0] << " " << points[i][1] << " " << points[i][2];
        outfile << " " << static_cast<int>(color_mask[i][0]) << " " << static_cast<int>(color_mask[i][1]) << " " << static_cast<int>(color_mask[i][2]) << "\n";
    }

    // Close the file
    outfile.close();

  // print the output
  if (verbose) {
    cv::viz::Viz3d window("semantic scan");
    cv::viz::WCloud cloudWidget(points, color_mask);
    while (!window.wasStopped()) {
      window.showWidget("cloud", cloudWidget);
      window.spinOnce(30, true);
    }
  }
  std::cout << std::setfill('=') << std::setw(80) << "" << std::endl;
  std::cout << "Example finished! "<< std::endl;
}
  return 0;
}

