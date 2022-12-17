#include <gtest/gtest.h>
#include <visnav/imu_dataset.h>
#include <tbb/concurrent_unordered_map.h>

using namespace visnav;

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
