#include <gtest/gtest.h>
#include <visnav/imu_dataset.h>
#include <tbb/concurrent_unordered_map.h>
#include <visnav/calibration.h>
#include <visnav/serialization.h>
using namespace visnav;

Calibration calib_cam;
std::string calib_path = "../euroc_ds_calib_visnav_type.json";

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
                  imu_data_map) {
  for (size_t i = 0; i < dataset_io->get_data()->get_accel_data().size(); i++) {
    ImuData<double>::Ptr data(new ImuData<double>);
    data->t_ns = dataset_io->get_data()->get_gyro_data()[i].timestamp_ns;
    data->accel = dataset_io->get_data()->get_accel_data()[i].data;
    data->gyro = dataset_io->get_data()->get_gyro_data()[i].data;
    imu_data_map.emplace(std::make_pair(data->t_ns, data));
  }
}

void feed_calibrated_imu(
    DatasetIoInterfacePtr& dataset_io,
    tbb::concurrent_unordered_map<Timestamp, ImuData<double>::Ptr>&
        imu_data_map) {
  for (size_t i = 0; i < dataset_io->get_data()->get_accel_data().size(); i++) {
    ImuData<double>::Ptr data(new ImuData<double>);
    data->t_ns = dataset_io->get_data()->get_gyro_data()[i].timestamp_ns;
    data->accel = getCalibratedAccel<double>(
        dataset_io->get_data()->get_accel_data()[i].data,
        calib_cam.calib_accel_bias);
    data->gyro = getCalibratedGyro<double>(
        dataset_io->get_data()->get_gyro_data()[i].data,
        calib_cam.calib_gyro_bias);
    imu_data_map.emplace(std::make_pair(data->t_ns, data));
  }
}

template <class Scalar>
Eigen::Matrix<Scalar, 3, 1> invertAccelCalibration(
    const Eigen::Matrix<Scalar, 3, 1>& calibrated_measurement,
    Eigen::Matrix<double, 9, 1> calib_accel_bias) {
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Mat33 = Eigen::Matrix<Scalar, 3, 3>;
  Vec3 accel_bias;
  Mat33 accel_scale;

  accel_bias = calib_accel_bias.template head<3>();

  accel_scale.setZero();
  accel_scale.col(0) = calib_accel_bias.template segment<3>(3);
  accel_scale(1, 1) = calib_accel_bias(6);
  accel_scale(2, 1) = calib_accel_bias(7);
  accel_scale(2, 2) = calib_accel_bias(8);

  Mat33 accel_scale_inv = (Eigen::Matrix3d::Identity() + accel_scale).inverse();

  return accel_scale_inv * (calibrated_measurement + accel_bias);
}

template <class Scalar>
Eigen::Matrix<Scalar, 3, 1> invertGyroCalibration(
    const Eigen::Matrix<Scalar, 3, 1>& calibrated_measurement,
    Eigen::Matrix<double, 12, 1> calib_gyro_bias) {
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Mat33 = Eigen::Matrix<Scalar, 3, 3>;
  Vec3 gyro_bias;
  Mat33 gyro_scale;

  gyro_bias = calib_gyro_bias.template head<3>();
  gyro_scale.col(0) = calib_gyro_bias.template segment<3>(3);
  gyro_scale.col(1) = calib_gyro_bias.template segment<3>(6);
  gyro_scale.col(2) = calib_gyro_bias.template segment<3>(9);

  Mat33 gyro_scale_inv = (Eigen::Matrix3d::Identity() + gyro_scale).inverse();

  return gyro_scale_inv * (calibrated_measurement + gyro_bias);
}

TEST(ImuTestSuite, LoadImuDataToQueue) {
  tbb::concurrent_unordered_map<Timestamp, ImuData<double>::Ptr> imu_data_map;
  std::string dataset_type = "euroc";
  std::string dataset_path = "../../data/euro_data/MH_01_easy";
  DatasetIoInterfacePtr dataset_io =
      DatasetIoFactory::getDatasetIo(dataset_type);
  dataset_io->read(dataset_path);
  feed_imu(dataset_io, imu_data_map);
  ASSERT_EQ(imu_data_map.size(), 36821 - 1);
  // 1403636579823555584,-0.11728612573401893,-0.019547687622336492,0.02583087292951608,7.8371477916666663,-0.4494714583333333,-2.852100708333333
  ASSERT_TRUE(imu_data_map.find(1403636579823555584) != imu_data_map.end());
  ImuData<double>::Ptr data(imu_data_map[1403636579823555584]);
  ASSERT_EQ(data->accel[0], 7.8371477916666663);
  ASSERT_EQ(data->gyro[0], -0.11728612573401893);
}

TEST(ImuTestSuite, CalibratedImu) {
  std::ifstream os(calib_path, std::ios::binary);

  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(calib_cam);
    std::cout << "Loaded camera from " << calib_path << "\n";
    std::cout << "The loaded imu bias data: \n";
    std::cout << calib_cam.calib_accel_bias << "\n";
    std::cout << calib_cam.calib_gyro_bias << "\n";
  } else {
    std::cerr << "could not load camera calibration " << calib_path
              << std::endl;
    std::abort();
  }
  tbb::concurrent_unordered_map<Timestamp, ImuData<double>::Ptr> imu_data_map;
  tbb::concurrent_unordered_map<Timestamp, ImuData<double>::Ptr>
      imu_calibrated_data_map;
  std::string dataset_type = "euroc";
  std::string dataset_path = "../../data/euro_data/MH_01_easy";
  DatasetIoInterfacePtr dataset_io =
      DatasetIoFactory::getDatasetIo(dataset_type);
  dataset_io->read(dataset_path);
  feed_imu(dataset_io, imu_data_map);
  feed_calibrated_imu(dataset_io, imu_calibrated_data_map);
  ASSERT_EQ(imu_data_map.size(), 36821 - 1);
  ASSERT_EQ(imu_data_map.size(), imu_calibrated_data_map.size());

  ImuData<double>::Ptr data(imu_data_map[1403636579823555584]);
  ImuData<double>::Ptr data_calibrated(
      imu_calibrated_data_map[1403636579823555584]);
  ASSERT_EQ(data->accel[0],
            invertAccelCalibration<double>(data_calibrated->accel,
                                           calib_cam.calib_accel_bias)[0]);
  ASSERT_EQ(data->gyro[0],
            invertGyroCalibration<double>(data_calibrated->gyro,
                                          calib_cam.calib_gyro_bias)[0]);
}
