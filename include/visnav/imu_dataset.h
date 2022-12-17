#ifndef IMU_DATASET_H
#define IMU_DATASET_H
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <fstream>
#include <visnav/common_types.h>
namespace visnav {
// Define the data structure for Gyro
struct GyroData {
  Timestamp timestamp_ns;
  Eigen::Vector3d data;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// Define the data structure for Accelerometer
struct AccelData {
  Timestamp timestamp_ns;
  Eigen::Vector3d data;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// Define the data structure for the 6d pose
struct PoseData {
  Timestamp timestamp_ns;
  Sophus::SE3d data;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// Define the base ImuDataset class
class ImuDataset {
 public:
  virtual ~ImuDataset(){};

  virtual const std::vector<AccelData>& get_accel_data() const = 0;
  virtual const std::vector<GyroData>& get_gyro_data() const = 0;
  virtual int64_t get_mocap_to_imu_offset_ns() const = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// Define the derived ImuDataset for the Euroc data
class EurocImuDataset : public ImuDataset {
  std::string path;
  std::vector<AccelData> accel_data;
  std::vector<GyroData> gyro_data;
  int64_t mocap_to_imu_offset_ns = 0;

 public:
  ~EurocImuDataset(){};

  const std::vector<AccelData>& get_accel_data() const { return accel_data; }
  const std::vector<GyroData>& get_gyro_data() const { return gyro_data; }

  int64_t get_mocap_to_imu_offset_ns() const { return mocap_to_imu_offset_ns; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class EurocIO;
};

typedef std::shared_ptr<ImuDataset> ImuDatasetPtr;

// Define the base class for IO interface for the ImuDataset
class DatasetIoInterface {
 public:
  virtual void read(const std::string& path) = 0;
  virtual void reset() = 0;
  virtual ImuDatasetPtr get_data() = 0;

  virtual ~DatasetIoInterface(){};
};
// Define the derived class for IO interface for the Euroc ImuDataset
class EurocIO : public DatasetIoInterface {
 public:
  EurocIO() {}

  void read(const std::string& path) {
    std::ifstream os(path, std::ios::binary);
    if (!os.is_open()) {
      std::cerr << "No dataset found in " << path << std::endl;
    } else {
      data.reset(new EurocImuDataset);

      data->path = path;

      read_imu_data(path + "/mav0/imu0/");
    }
  }

  void reset() { data.reset(); }

  ImuDatasetPtr get_data() { return data; }

 private:
  void read_imu_data(const std::string& path) {
    data->accel_data.clear();
    data->gyro_data.clear();

    std::ifstream f(path + "data.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      uint64_t timestamp;
      Eigen::Vector3d gyro, accel;

      ss >> timestamp >> tmp >> gyro[0] >> tmp >> gyro[1] >> tmp >> gyro[2] >>
          tmp >> accel[0] >> tmp >> accel[1] >> tmp >> accel[2];

      data->accel_data.emplace_back();
      data->accel_data.back().timestamp_ns = timestamp;
      data->accel_data.back().data = accel;

      data->gyro_data.emplace_back();
      data->gyro_data.back().timestamp_ns = timestamp;
      data->gyro_data.back().data = gyro;
    }
  }

  std::shared_ptr<EurocImuDataset> data;
};

typedef std::shared_ptr<DatasetIoInterface> DatasetIoInterfacePtr;

// It's possible that our program could support multiple types of dataset,
// so we need to keep this structure for future development
class DatasetIoFactory {
 public:
  static DatasetIoInterfacePtr getDatasetIo(const std::string& dataset_type) {
    if (dataset_type == "euroc") {
      return DatasetIoInterfacePtr(new EurocIO());
    } else {
      std::cerr << "Dataset type " << dataset_type << " is not supported"
                << std::endl;
      std::abort();
    }
  }
};

template <class Scalar>
struct ImuData {
  using Ptr = std::shared_ptr<ImuData>;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

  Timestamp t_ns;  // Timestamp in nanoseconds
  Vec3 accel;
  Vec3 gyro;

  // Constructor
  ImuData() {
    t_ns = 0;
    accel.setZero();
    gyro.setZero();
  }
};
}  // namespace visnav
#endif  // IMU_DATASET_H
