#include <gtest/gtest.h>
#include <visnav/imu_dataset.h>
#include <tbb/concurrent_unordered_map.h>
#include "../include/visnav/imu/imu_types.h"
#include "visnav/calibration.h"
#include <visnav/serialization.h>
#include "../include/visnav/imu/preintegration.h"
#include "../include/visnav/imu/utils/calib_bias.hpp"
#include <visnav/common_types.h>
#include <queue>
using namespace visnav;
Calibration calib_cam;
std::string calib_path = "../../euroc_ds_calib_visnav_type.json";
TEST(ImuTestSuite, LoadImuData) {
  std::string dataset_type = "euroc";
  std::string dataset_path = "../../data/euro_data/MH_01_easy";
  DatasetIoInterfacePtr dataset_io =
      DatasetIoFactory::getDatasetIo(dataset_type);
  dataset_io->read(dataset_path);
  Eigen::Vector3d test_acc_data =
      dataset_io->get_data()->get_accel_data()[0].data;
  Timestamp test_acc_ts =
      dataset_io->get_data()->get_accel_data()[0].timestamp_ns;
  Timestamp test_gyro_ts =
      dataset_io->get_data()->get_gyro_data()[0].timestamp_ns;
  ASSERT_EQ(test_acc_ts, test_gyro_ts);
  ASSERT_EQ(test_acc_ts, 1403636579758555392);
  Eigen::Vector3d test_gyro_data =
      dataset_io->get_data()->get_gyro_data()[0].data;
  //  std::cout << test_acc_data[0] << "\n";
  ASSERT_EQ(test_acc_data[0], 8.1476917083333333);
  ASSERT_EQ(test_acc_data[1], -0.37592158333333331);
  ASSERT_EQ(test_acc_data[2], -2.4026292499999999);

  ASSERT_EQ(test_gyro_data[0], -0.099134701513277898);
  ASSERT_EQ(test_gyro_data[1], 0.14730578886832138);
  ASSERT_EQ(test_gyro_data[2], 0.02722713633111154);
  ASSERT_EQ(dataset_io->get_data()->get_accel_data().size(), 36821 - 1);
}

void feed_imu(DatasetIoInterfacePtr& dataset_io,
              tbb::concurrent_unordered_map<Timestamp, ImuData<double>::Ptr>&
                  imu_data_map,
              std::vector<Timestamp>& timestamps) {
  for (size_t i = 0; i < dataset_io->get_data()->get_accel_data().size(); i++) {
    ImuData<double>::Ptr data(new ImuData<double>);
    data->t_ns = dataset_io->get_data()->get_gyro_data()[i].timestamp_ns;
    timestamps.push_back(data->t_ns);
    data->accel = dataset_io->get_data()->get_accel_data()[i].data;
    data->gyro = dataset_io->get_data()->get_gyro_data()[i].data;
    imu_data_map.emplace(std::make_pair(data->t_ns, data));
  }
}
TEST(ImuTestSuite, LoadImuDataToQueue) {
  tbb::concurrent_unordered_map<Timestamp, ImuData<double>::Ptr> imu_data_map;
  std::vector<Timestamp> timestamps;
  std::string dataset_type = "euroc";
  std::string dataset_path = "../../data/euro_data/MH_01_easy";
  DatasetIoInterfacePtr dataset_io =
      DatasetIoFactory::getDatasetIo(dataset_type);
  dataset_io->read(dataset_path);
  feed_imu(dataset_io, imu_data_map, timestamps);
  ASSERT_EQ(imu_data_map.size(), 36821 - 1);
  // 1403636579823555584,-0.11728612573401893,-0.019547687622336492,0.02583087292951608,7.8371477916666663,-0.4494714583333333,-2.852100708333333
  ASSERT_TRUE(imu_data_map.find(1403636579823555584) != imu_data_map.end());
  ASSERT_TRUE(timestamps[0] == 1403636579758555392);
  ASSERT_EQ(timestamps.size(), 36821 - 1);
  ImuData<double>::Ptr data(imu_data_map[1403636579823555584]);
  ASSERT_EQ(data->accel[0], 7.8371477916666663);
  ASSERT_EQ(data->gyro[0], -0.11728612573401893);
}

TEST(ImuTestSuite, ImuDataIntegrate) {
  // Loading the calib file
  std::ifstream os(calib_path, std::ios::binary);

  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(calib_cam);
    std::cout << "Loaded camera from " << calib_path << "\n";
    std::cout << "The loaded imu bias data: \n";
    std::cout << "calib_accel_bias: \n" << calib_cam.calib_accel_bias << "\n";
    std::cout << "calib_gyro_bias: \n" << calib_cam.calib_gyro_bias << "\n";
    std::cout << "accel_noise_std: \n" << calib_cam.accel_noise_std << "\n";
    std::cout << "gyro_noise_std: \n" << calib_cam.gyro_noise_std << "\n";
    std::cout << "imu_update_rate: \n" << calib_cam.imu_update_rate << "\n";
  } else {
    std::cerr << "could not load camera calibration " << calib_path
              << std::endl;
    std::abort();
  }
  CalibAccelBias<double> calib_accel;
  CalibGyroBias<double> calib_gyro;

  calib_accel.getParam() = calib_cam.calib_accel_bias;
  calib_gyro.getParam() = calib_cam.calib_gyro_bias;
  std::cout << calib_accel.getParam() << "\n";
  std::cout << calib_gyro.getParam() << "\n";

  // IMU calibration data loaded completed ...
  std::queue<std::pair<Timestamp, ImuData<double>::Ptr>> imu_data_queue;
  tbb::concurrent_unordered_map<Timestamp, ImuData<double>::Ptr> imu_data_map;
  tbb::concurrent_unordered_map<Timestamp, PoseVelState<double>> states;
  std::vector<Timestamp> timestamps_imu;
  std::vector<Timestamp> timestamps_frame;
  std::string dataset_type = "euroc";
  std::string dataset_path = "../../data/euro_data/MH_01_easy";

  DatasetIoInterfacePtr dataset_io =
      DatasetIoFactory::getDatasetIo(dataset_type);
  dataset_io->read(dataset_path);

  for (size_t i = 0; i < dataset_io->get_data()->get_accel_data().size(); i++) {
    ImuData<double>::Ptr data(new ImuData<double>);
    data->t_ns = dataset_io->get_data()->get_gyro_data()[i].timestamp_ns;
    timestamps_imu.push_back(data->t_ns);
    data->accel = calib_accel.getCalibrated(
        dataset_io->get_data()->get_accel_data()[i].data);
    data->gyro = calib_gyro.getCalibrated(
        dataset_io->get_data()->get_gyro_data()[i].data);
    //    data->accel = dataset_io->get_data()->get_accel_data()[i].data;
    //    data->gyro = dataset_io->get_data()->get_gyro_data()[i].data;
    imu_data_queue.push(std::make_pair(data->t_ns, data));
  }
  // Testing if the queue work as expected
  //  int size_before_pop = imu_data_queue.size();
  //  ImuData<double>::Ptr data = imu_data_queue.front().second;
  //  imu_data_queue.pop();
  //  int size_after_pop = imu_data_queue.size();
  //  ASSERT_EQ(size_before_pop - 1, size_after_pop);
  //  ASSERT_EQ(data->t_ns, 1403636579758555392);
  const std::string timestams_path = dataset_path + "/mav0/cam0/data.csv";

  {
    std::ifstream times(timestams_path);

    int id = 0;

    while (times) {
      std::string line;
      std::getline(times, line);

      if (line.size() < 20 || line[0] == '#' || id > 2700) continue;

      {
        std::string timestamp_str = line.substr(0, 19);
        std::istringstream ss(timestamp_str);
        Timestamp timestamp;
        ss >> timestamp;
        timestamps_frame.push_back(timestamp);
      }
      id++;
    }
  }

  ASSERT_EQ(timestamps_frame[0], 1403636579763555584);

  ImuData<double>::Ptr data = imu_data_queue.front().second;
  imu_data_queue.pop();
  while (data->t_ns < timestamps_frame[0]) {
    data = imu_data_queue.front().second;
    imu_data_queue.pop();
  }
  using Vec3 = Eigen::Matrix<double, 3, 1>;
  const Vec3 accel_cov =
      (calib_cam.accel_noise_std * std::sqrt(calib_cam.imu_update_rate))
          .array()
          .square();

  const Vec3 gyro_cov =
      (calib_cam.gyro_noise_std * std::sqrt(calib_cam.imu_update_rate))
          .array()
          .square();

  // Initialize the pose following the pipeline in basalt
  Vec3 vel_w_i_init;
  vel_w_i_init.setZero();
  Sophus::SE3d T_w_i_init;
  T_w_i_init.setQuaternion(
      Eigen::Quaternion<double>::FromTwoVectors(data->accel, Vec3::UnitZ()));
  Timestamp last_t_ns = timestamps_frame[0];
  PoseVelState<double> last_state;
  last_state.T_w_i = T_w_i_init;
  last_state.vel_w_i = vel_w_i_init;
  states.emplace(std::make_pair(last_t_ns, last_state));

  IntegratedImuMeasurement<double>::Ptr imu_mea;

  for (int i = 1; i < timestamps_frame.size(); i++) {
    Timestamp curr_frame = timestamps_frame[i];

    imu_mea.reset(new IntegratedImuMeasurement<double>(
        last_t_ns, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
    //    imu_mea.reset(new IntegratedImuMeasurement<double>(
    //        last_t_ns, calib_cam.calib_gyro_bias.template head<3>(),
    //        calib_cam.calib_accel_bias.template head<3>()));
    while (data->t_ns <= last_t_ns) {
      data = imu_data_queue.front().second;
      imu_data_queue.pop();
    }

    while (data->t_ns <= curr_frame) {
      imu_mea->integrate(*data, accel_cov, gyro_cov);
      data = imu_data_queue.front().second;
      imu_data_queue.pop();
    }

    PoseVelState<double> next_state = states[last_t_ns];

    imu_mea->predictState(next_state, constants::g, next_state);

    states.emplace(std::make_pair(curr_frame, next_state));
    last_t_ns = curr_frame;
  }

  //  std::cout << calib_cam.calib_accel_bias << "\n";
  //  std::cout << calib_cam.calib_gyro_bias << "\n";

  // Frame 91, Frame 92 calculated by camera
  Sophus::SE3d pose1 = states[1403636584313555456].T_w_i;
  Sophus::SE3d pose2 = states[1403636584363555584].T_w_i;
  Vec3 omega_gt(3.13923, 3.13815, 3.13901);
  Vec3 trans_gt(0.0288212, -2.30568e-05, -0.0097389);
  // Frame 55, Frame 56 calculated by camera
  //  Sophus::SE3d pose1 = states[1403636582513555456].T_w_i;
  //  Sophus::SE3d pose2 = states[1403636582563555584].T_w_i;
  //  Vec3 omega_gt(3.13831, 3.13183, 3.14129);
  //  Vec3 trans_gt(0.0312661, -0.000510182, -0.0107195);
  // Frame 71, Frame 72 calculated by camera
  //  Sophus::SE3d pose1 = states[1403636583313555456].T_w_i;
  //  Sophus::SE3d pose2 = states[1403636583363555584].T_w_i;
  //  Vec3 omega_gt(3.13831, 3.13183, 3.14129);
  //  Vec3 trans_gt(-0.0261785, 0.00106192, 0.00805227);
  Sophus::SE3d relpose = pose1.inverse() * pose2;
  std::cout << relpose.rotationMatrix() << std::endl;
  std::cout << "Translation_imu = [" << relpose.translation() << " ]"
            << std::endl;
  Eigen::Matrix<double, 3, 1> omega =
      relpose.rotationMatrix().eulerAngles(0, 1, 2);
  std::cout << "Orientation_imu = [ " << omega << " ]" << std::endl;
  std::cout << "Orientation Error = [ " << omega - omega_gt << " ]"
            << std::endl;

  //  ASSERT_LE(std::abs(omega[0] - omega_gt[0]), 1e-3);
  //  ASSERT_LE(std::abs(omega[1] - omega_gt[1]), 1e-3);
  //  ASSERT_LE(std::abs(omega[2] - omega_gt[2]), 1e-3);

  std::cout << "Translation Error = [ " << relpose.translation() - trans_gt
            << " ]" << std::endl;

  //  ASSERT_TRUE(std::abs(relpose.translation()[0] - trans_gt[0]) < 1e-3)
  //      << "Error: " << std::abs(relpose.translation()[0] - trans_gt[0]);
  //  ASSERT_TRUE(std::abs(relpose.translation()[1] - trans_gt[1]) < 1e-3)
  //      << "Error: " << std::abs(relpose.translation()[1] - trans_gt[1]);
  //  ASSERT_TRUE(std::abs(relpose.translation()[2] - trans_gt[2]) < 1e-3)
  //      << "Error: " << std::abs(relpose.translation()[2] - trans_gt[2]);
}

TEST(ImuTestSuite, ImuDataMap) {
  // Loading the calib file
  std::ifstream os(calib_path, std::ios::binary);

  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(calib_cam);
    std::cout << "Loaded camera from " << calib_path << "\n";
    std::cout << "The loaded imu bias data: \n";
    std::cout << "calib_accel_bias: \n" << calib_cam.calib_accel_bias << "\n";
    std::cout << "calib_gyro_bias: \n" << calib_cam.calib_gyro_bias << "\n";
    std::cout << "accel_noise_std: \n" << calib_cam.accel_noise_std << "\n";
    std::cout << "gyro_noise_std: \n" << calib_cam.gyro_noise_std << "\n";
    std::cout << "imu_update_rate: \n" << calib_cam.imu_update_rate << "\n";
  } else {
    std::cerr << "could not load camera calibration " << calib_path
              << std::endl;
    std::abort();
  }
  CalibAccelBias<double> calib_accel;
  CalibGyroBias<double> calib_gyro;

  calib_accel.getParam() = calib_cam.calib_accel_bias;
  calib_gyro.getParam() = calib_cam.calib_gyro_bias;
  std::cout << calib_accel.getParam() << "\n";
  std::cout << calib_gyro.getParam() << "\n";

  // IMU calibration data loaded completed ...
  std::queue<std::pair<Timestamp, ImuData<double>::Ptr>> imu_data_queue;
  Eigen::aligned_map<Timestamp, PoseVelState<double>> states;
  std::vector<Timestamp> timestamps_imu;
  std::vector<Timestamp> timestamps_frame;
  std::string dataset_type = "euroc";
  std::string dataset_path = "../../data/euro_data/MH_01_easy";

  DatasetIoInterfacePtr dataset_io =
      DatasetIoFactory::getDatasetIo(dataset_type);
  dataset_io->read(dataset_path);

  for (size_t i = 0; i < dataset_io->get_data()->get_accel_data().size(); i++) {
    ImuData<double>::Ptr data(new ImuData<double>);
    data->t_ns = dataset_io->get_data()->get_gyro_data()[i].timestamp_ns;
    timestamps_imu.push_back(data->t_ns);
    data->accel = calib_accel.getCalibrated(
        dataset_io->get_data()->get_accel_data()[i].data);
    data->gyro = calib_gyro.getCalibrated(
        dataset_io->get_data()->get_gyro_data()[i].data);
    //    data->accel = dataset_io->get_data()->get_accel_data()[i].data;
    //    data->gyro = dataset_io->get_data()->get_gyro_data()[i].data;
    imu_data_queue.push(std::make_pair(data->t_ns, data));
  }
  // Testing if the queue work as expected
  //  int size_before_pop = imu_data_queue.size();
  //  ImuData<double>::Ptr data = imu_data_queue.front().second;
  //  imu_data_queue.pop();
  //  int size_after_pop = imu_data_queue.size();
  //  ASSERT_EQ(size_before_pop - 1, size_after_pop);
  //  ASSERT_EQ(data->t_ns, 1403636579758555392);
  const std::string timestams_path = dataset_path + "/mav0/cam0/data.csv";

  {
    std::ifstream times(timestams_path);

    int id = 0;

    while (times) {
      std::string line;
      std::getline(times, line);

      if (line.size() < 20 || line[0] == '#' || id > 2700) continue;

      {
        std::string timestamp_str = line.substr(0, 19);
        std::istringstream ss(timestamp_str);
        Timestamp timestamp;
        ss >> timestamp;
        timestamps_frame.push_back(timestamp);
      }
      id++;
    }
  }

  ASSERT_EQ(timestamps_frame[0], 1403636579763555584);

  ImuData<double>::Ptr data = imu_data_queue.front().second;
  imu_data_queue.pop();
  while (data->t_ns < timestamps_frame[0]) {
    data = imu_data_queue.front().second;
    imu_data_queue.pop();
  }
  using Vec3 = Eigen::Matrix<double, 3, 1>;
  const Vec3 accel_cov =
      (calib_cam.accel_noise_std * std::sqrt(calib_cam.imu_update_rate))
          .array()
          .square();

  const Vec3 gyro_cov =
      (calib_cam.gyro_noise_std * std::sqrt(calib_cam.imu_update_rate))
          .array()
          .square();

  // Initialize the pose following the pipeline in basalt
  Vec3 vel_w_i_init;
  vel_w_i_init.setZero();
  Sophus::SE3d T_w_i_init;
  T_w_i_init.setQuaternion(
      Eigen::Quaternion<double>::FromTwoVectors(data->accel, Vec3::UnitZ()));
  Timestamp last_t_ns = timestamps_frame[0];
  PoseVelState<double> last_state;
  last_state.T_w_i = T_w_i_init;
  last_state.vel_w_i = vel_w_i_init;
  //  states.emplace(std::make_pair(last_t_ns, last_state));
  states[last_t_ns] = last_state;
  IntegratedImuMeasurement<double>::Ptr imu_mea;

  for (int i = 1; i < timestamps_frame.size(); i++) {
    Timestamp curr_frame = timestamps_frame[i];

    imu_mea.reset(new IntegratedImuMeasurement<double>(
        last_t_ns, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
    //    imu_mea.reset(new IntegratedImuMeasurement<double>(
    //        last_t_ns, calib_cam.calib_gyro_bias.template head<3>(),
    //        calib_cam.calib_accel_bias.template head<3>()));
    while (data->t_ns <= last_t_ns) {
      data = imu_data_queue.front().second;
      imu_data_queue.pop();
    }

    while (data->t_ns <= curr_frame) {
      imu_mea->integrate(*data, accel_cov, gyro_cov);
      data = imu_data_queue.front().second;
      imu_data_queue.pop();
    }

    PoseVelState<double> next_state = states[last_t_ns];

    imu_mea->predictState(next_state, constants::g, next_state);

    states.emplace(std::make_pair(curr_frame, next_state));
    states[curr_frame] = next_state;
    last_t_ns = curr_frame;
  }
  std::cout << "The length of the map is: " << states.size() << std::endl;
  int iter_counter = 0;
  auto iter = states.rbegin();
  while (iter_counter < 3) {
    std::cout << iter->first << std::endl;
    //    if (iter_counter == 0) {
    //      ASSERT_EQ(iter->first, 1403636763853555456);
    //    }
    //    if (iter_counter == 1) {
    //      ASSERT_EQ(iter->first, 1403636763848555520);
    //    }
    //    if (iter_counter == 2) {
    //      ASSERT_EQ(iter->first, 1403636763843555584);
    //    }

    ++iter;
    iter_counter++;
  }
}
