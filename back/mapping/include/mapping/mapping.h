#pragma once
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <vector>

namespace mapping {

template <typename _Datatype>
struct RingBuffer {
 public:
  int size_x, size_y, size_z;
  std::vector<_Datatype> data;

  inline void setup(int _size_x, int _size_y, int _size_z) {
    size_x = _size_x;
    size_y = _size_y;
    size_z = _size_z;
    data.resize(size_x * size_y * size_z);
  }
  inline const int idx2add(int x, int N) const {
    // return x % N >= 0 ? x % N : x % N + N;
    // NOTE this is much faster than before!!
    return (x & N) >= 0 ? (x & N) : (x & N) + N;
  }
  inline const Eigen::Vector3i idx2add(const Eigen::Vector3i& id) const {
    return Eigen::Vector3i(idx2add(id.x(), size_x - 1),
                           idx2add(id.y(), size_y - 1),
                           idx2add(id.z(), size_z - 1));
  }
  // NOTE dangerous!! ad should be the address in the data
  inline const _Datatype& at(const Eigen::Vector3i& ad) const {
    return data[(ad.z() * size_y + ad.y()) * size_x + ad.x()];
  }
  inline _Datatype& at(const Eigen::Vector3i& ad) {
    return data[(ad.z() * size_y + ad.y()) * size_x + ad.x()];
  }
  inline _Datatype* atPtr(const Eigen::Vector3i& ad) {
    return &(data[(ad.z() * size_y + ad.y()) * size_x + ad.x()]);
  }
  inline const _Datatype& atId(const Eigen::Vector3i& id) const {
    return at(idx2add(id));
  }
  inline _Datatype& atId(const Eigen::Vector3i& id) {
    return at(idx2add(id));
  }
  inline _Datatype* atIdPtr(const Eigen::Vector3i& id) {
    return atPtr(idx2add(id));
  }
  inline void fillData(const _Datatype& val) {
    std::fill(data.begin(), data.end(), val);
  }
};

struct OccGridMap {
 public:
  // parameters
  int p_min, p_max, p_hit, p_mis, p_occ, p_def;
  int inflate_size;
  double sensor_range;
  // states
  bool init_finished = false;
  int offset_x, offset_y, offset_z;
  // ring buffer
  double resolution;
  int size_x, size_y, size_z;

 private:
  RingBuffer<int8_t> infocc;  // -128 ~ 127  1 for occupied, 0 for known, -1 for free
  RingBuffer<int8_t> vis;     // 1 for occupied, -1 for raycasted, 0 for free or unvisited
  RingBuffer<int16_t> pro;
  RingBuffer<u_int16_t> occ;  // 0 ~ 65535  half: 32768

 public:
  inline void setup(const double& res,
                    const Eigen::Vector3d& map_size,
                    const double& cam_range,
                    bool use_global_map = false) {
    resolution = res;
    size_x = exp2(int(log2(map_size.x() / res)));
    size_y = exp2(int(log2(map_size.y() / res)));
    size_z = exp2(int(log2(map_size.z() / res)));
    if (use_global_map) {
      offset_x = -size_x;
      offset_y = -size_y;
      offset_z = -size_z;
      size_x *= 2;
      size_y *= 2;
      size_z *= 2;
    }
    infocc.setup(size_x, size_y, size_z);
    occ.setup(size_x, size_y, size_z);
    vis.setup(size_x, size_y, size_z);
    occ.fillData(0);
    infocc.fillData(0);
    //------------------------------
    pro.setup(size_x, size_y, size_z);
    pro.fillData(p_def);
    sensor_range = cam_range;
    //--------------------------------
    if (use_global_map) {
      return;
    }
    pro.setup(size_x, size_y, size_z);
    pro.fillData(p_def);
    sensor_range = cam_range;
  }
  inline void setupP(const int& _p_min,
                     const int& _p_max,
                     const int& _p_hit,
                     const int& _p_mis,
                     const int& _p_occ,
                     const int& _p_def) {
    // NOTE logit(x) = log(x/(1-x))
    p_min = _p_min;  // 0.12 -> -199
    p_max = _p_max;  // 0.90 ->  220
    p_hit = _p_hit;  // 0.65 ->   62
    p_mis = _p_mis;  // 0.35 ->   62
    p_occ = _p_occ;  // 0.80 ->  139
    p_def = _p_def;  // 0.12 -> -199
    // std::cout << "setupP called: p_def = " << p_def << std::endl;  // 添加打印
  }

  inline const Eigen::Vector3i pos2idx(const Eigen::Vector3d& pt) const {
    return (pt / resolution).array().floor().cast<int>();
  }
  inline const Eigen::Vector3d idx2pos(const Eigen::Vector3i& id) const {
    return (id.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * resolution;
  }
  inline const bool isInMap(const Eigen::Vector3i& id) const {
    return !((id.x() - offset_x) & (~(size_x - 1))) &&
           !((id.y() - offset_y) & (~(size_y - 1))) &&
           !((id.z() - offset_z) & (~(size_z - 1)));
  }
  inline const bool isInMap(const Eigen::Vector3d& p) const {
    return isInMap(pos2idx(p));
  }
  inline const bool isInSmallMap(const Eigen::Vector3i& id) const {
    return id.x() - offset_x >= inflate_size && id.x() - offset_x < size_x - inflate_size &&
           id.y() - offset_y >= inflate_size && id.y() - offset_y < size_y - inflate_size &&
           id.z() - offset_z >= inflate_size && id.z() - offset_z < size_z - inflate_size;
  }
  inline const bool isInSmallMap(const Eigen::Vector3d& p) const {
    return isInSmallMap(pos2idx(p));
  }
  template <typename _Msgtype>
  inline void to_msg(_Msgtype& msg) {
    msg.resolution = resolution;
    msg.size_x = size_x;
    msg.size_y = size_y;
    msg.size_z = size_z;
    msg.offset_x = offset_x;
    msg.offset_y = offset_y;
    msg.offset_z = offset_z;
    msg.data = infocc.data;
  }
  template <typename _Msgtype>
  inline void from_msg(const _Msgtype& msg) {
    resolution = msg.resolution;
    size_x = msg.size_x;
    size_y = msg.size_y;
    size_z = msg.size_z;
    offset_x = msg.offset_x;
    offset_y = msg.offset_y;
    offset_z = msg.offset_z;
    infocc.setup(size_x, size_y, size_z);
    infocc.data = msg.data;
  }
  inline void setOcc(const Eigen::Vector3d& p) {
    infocc.atId(pos2idx(p)) = 1;
  }

  inline void setOcc2(const Eigen::Vector3d& p){
    infocc.atId(pos2idx(p)) = 1;

  }

  inline void setFree2(const Eigen::Vector3i& id) {
    if (isInMap(id)) {
      // std::cout << "is_normal?" << std::endl;
      // const auto& ad = occ.idx2add(id);  // 转换三维索引为一维地址ad
      // vis.at(ad) = 0;
    //   const auto& ad = pro.idx2add(id);  // 获取pro的内部地址
    // int total_size = pro.size_x * pro.size_y * pro.size_z;
    // int linear_ad = (ad.z() * pro.size_y + ad.y()) * pro.size_x + ad.x();
    // if (linear_ad < 0 || linear_ad >= total_size) {
    //   std::cerr << "pro地址越界！linear_ad = " << linear_ad << ", total_size = " << total_size << std::endl;
    //   return;
    // }
      vis.atId(id) = 0;
      occ.atId(id) = 0;
      infocc.atId(id) = 0;
      // std::cout << "p_def = " << p_def <<std::endl;
      pro.atId(id) = p_def;
    }
  }
  inline void RemoveOcc(const Eigen::Vector3i& id) {
    infocc.atId(id) = 0;
  }

  inline void testhit(const Eigen::Vector3i& idx){
    if (vis.atId(idx) != 1) {
      hit(idx);
    }
  }

  inline void testmis(const Eigen::Vector3i &idx){
    if(vis.atId(idx) !=1 ){
      mis(idx);
    }
  }

 private:
  std::vector<Eigen::Vector3i> v0, v1;
  // return true if in range; id_filtered will be limited in range
  inline bool filter(const Eigen::Vector3d& sensor_p,
                     const Eigen::Vector3d& p,
                     Eigen::Vector3d& pt) const {
    Eigen::Vector3i id = pos2idx(p);
    Eigen::Vector3d dp = p - sensor_p;
    double dist = dp.norm();
    pt = p;
    if (dist < sensor_range && isInSmallMap(id)) {
      return true;
    } else if (dist >= sensor_range) {
      pt = sensor_range / dist * dp + sensor_p;
    }
    if (isInSmallMap(pt)) {
      return false;
    } else {
      dp = pt - sensor_p;
      Eigen::Array3d v = dp.array().abs() / resolution;
      Eigen::Array3d d;
      d.x() = v.x() <= size_x / 2 - 1 - inflate_size ? 0 : v.x() - size_x / 2 + 1 + inflate_size;
      d.y() = v.y() <= size_y / 2 - 1 - inflate_size ? 0 : v.y() - size_y / 2 + 1 + inflate_size;
      d.z() = v.z() <= size_z / 2 - 1 - inflate_size ? 0 : v.z() - size_z / 2 + 1 + inflate_size;
      double t_max = 0;
      for (int i = 0; i < 3; ++i) {
        t_max = (d[i] > 0 && d[i] / v[i] > t_max) ? d[i] / v[i] : t_max;
      }
      pt = pt - dp * t_max;
      return false;
    }
  }
  inline void free2occ(const Eigen::Vector3i& idx) {
    occ.atId(idx) += 32767;
    Eigen::Vector3i id;
    for (id.x() = idx.x() - inflate_size; id.x() <= idx.x() + inflate_size; ++id.x())
      for (id.y() = idx.y() - inflate_size; id.y() <= idx.y() + inflate_size; ++id.y())
        for (id.z() = idx.z() - inflate_size; id.z() <= idx.z() + inflate_size; ++id.z()) {
          occ.atId(id)++;
          infocc.atId(id) = 1;
        }
  }
  inline void occ2free(const Eigen::Vector3i& idx) {
    occ.atId(idx) -= 32767;
    Eigen::Vector3i id;
    for (id.x() = idx.x() - inflate_size; id.x() <= idx.x() + inflate_size; ++id.x())
      for (id.y() = idx.y() - inflate_size; id.y() <= idx.y() + inflate_size; ++id.y())
        for (id.z() = idx.z() - inflate_size; id.z() <= idx.z() + inflate_size; ++id.z()) {
          occ.atId(id)--;
          infocc.atId(id) = occ.atId(id) > 0 ? 1 : -1;
        }
  }
  inline void hit(const Eigen::Vector3i& idx) {
    const auto& ad = occ.idx2add(idx);
    bool occ_pre = pro.at(ad) > p_occ;
    pro.at(ad) = pro.at(ad) + p_hit > p_max ? p_max : pro.at(ad) + p_hit;
    std::cout << "pro.at(ad) = " << pro.at(ad) << std::endl;
    bool occ_now = pro.at(ad) > p_occ;
    std::cout << "occ_now = " << pro.at(ad) << std::endl;
    if (!occ_pre && occ_now) {
      free2occ(idx);
    }
    vis.at(ad) = 1;  // set occupied
  }
  inline void mis(const Eigen::Vector3i& idx) {
    const auto& ad = occ.idx2add(idx);
    bool occ_pre = pro.at(ad) > p_occ;
    pro.at(ad) = pro.at(ad) - p_mis < p_min ? p_min : pro.at(ad) - p_mis;
    bool occ_now = pro.at(ad) > p_occ;
    if (occ_pre && !occ_now) {
      occ2free(idx);
    }
    vis.at(ad) = -1;  // set raycasted
  }
  inline void hit2(const Eigen::Vector3i& idx) {
    const auto& ad = occ.idx2add(idx);
    free2occ(idx);
    // occ.atId(idx) = 100;
    // infocc.atId(idx) = 1;
    vis.at(ad) = 1;
  }
  inline void mis2(const Eigen::Vector3i& idx) {
    const auto& ad = occ.idx2add(idx);
    // occ.atId(idx) = -1;
    // infocc.atId(idx) = -1;
    occ2free(idx);
    vis.at(ad) = -1;
  }




 public:
  inline const bool isOccupied(const Eigen::Vector3i& id) const {
    return isInMap(id) && infocc.atId(id) == 1;
  }
  inline const bool isOccupied(const Eigen::Vector3d& p) const {
    return isOccupied(pos2idx(p));
  }
  inline const bool isUnKnown(const Eigen::Vector3i& id) const {
    return (!isInMap(id)) || infocc.atId(id) == 0;
  }
  inline const bool isUnKnown(const Eigen::Vector3d& p) const {
    return isUnKnown(pos2idx(p));
  }
  inline void setFree(const Eigen::Vector3i& id) {
    if (isInMap(id)) {
      infocc.atId(id) = -1;
    }
  }
  inline void setFree(const Eigen::Vector3d& p) {
    Eigen::Vector3i id = pos2idx(p);
    setFree(id);
  }
  inline void setFree(const Eigen::Vector3d& ld, const Eigen::Vector3d& ru) {
    Eigen::Vector3i id_ld = pos2idx(ld);
    Eigen::Vector3i id_ru = pos2idx(ru);
    Eigen::Vector3i id;
    for (id.x() = id_ld.x(); id.x() <= id_ru.x(); ++id.x())
      for (id.y() = id_ld.y(); id.y() <= id_ru.y(); ++id.y())
        for (id.z() = id_ld.z(); id.z() <= id_ru.z(); ++id.z()) {
          setFree(id);
        }
  }


  // -----------new add ---------
  inline void setOccGlobal(const Eigen::Vector3d& p) {
    Eigen::Vector3i idx = pos2idx(p);
    auto ad = occ.idx2add(idx);

    // 1) 概率层：设为占据态
    pro.at(ad) = p_occ + 1; 

    // 2) 计数层：同步 infocc=1
    free2occ(idx);

    // 3) 可选：本帧标记
    // vis.at(ad) = 1;
  }

  // 将全局地图点作为一次观测 (hit) 加入，占据程度可调
  inline void addGlobalAsHit(const Eigen::Vector3d& p, int weight = -1, bool inflate = false) {
    Eigen::Vector3i idx = pos2idx(p);
    auto ad = occ.idx2add(idx);

    // 1) 取权重：默认用传感器权重，否则用指定值
    int w = (weight > 0) ? weight : p_hit;

    // 2) 判定当前是否已占据
    bool occ_pre = pro.at(ad) > p_occ;

    // 3) 增加概率值
    pro.at(ad) = std::min(p_max, pro.at(ad) + w);

    // 4) 判定是否由自由 → 占据
    bool occ_now = pro.at(ad) > p_occ;
    if (!occ_pre && occ_now) {
        if (inflate) {
            free2occ(idx);   // 膨胀：中心 + 邻域都设为占据
        } else {
            infocc.atId(idx) = 1;  // 只设置当前格子占据
        }
    }

    // 5) 标记为本帧占据
    vis.at(ad) = 1;
  }

  // 将全局地图点作为一次完整观测 (hit + raycasting mis) 加入
  inline void addGlobalAsObservation(const Eigen::Vector3d& sensor_p,
                                   const Eigen::Vector3d& p,
                                   int weight = -1) {
    Eigen::Vector3i sensor_idx = pos2idx(sensor_p);
    Eigen::Vector3i idx = pos2idx(p);
    auto ad = occ.idx2add(idx);

    // 权重选择：默认用 p_hit
    int w = (weight > 0) ? weight : p_hit;

    // === Step 1: hit 终点格子 ===
    bool occ_pre = pro.at(ad) > p_occ;
    pro.at(ad) = std::min(p_max, pro.at(ad) + w);
    bool occ_now = pro.at(ad) > p_occ;
    if (!occ_pre && occ_now) {
        free2occ(idx);   // 占据 & 膨胀
    }
    vis.at(ad) = 1;

    // === Step 2: 射线 mis 清空路径 ===
    Eigen::Vector3i d_idx = sensor_idx - idx;
    Eigen::Vector3i step = d_idx.array().sign().cast<int>();
    Eigen::Vector3d dp = sensor_p - p;

    Eigen::Vector3d delta_t;
    for (int i = 0; i < 3; ++i) {
        delta_t(i) = dp(i) == 0 ? std::numeric_limits<double>::max() : 1.0 / std::fabs(dp(i));
    }

    Eigen::Vector3d t_max;
    for (int i = 0; i < 3; ++i) {
        t_max(i) = step(i) > 0 ? (idx(i) + 1) - p(i) / resolution
                               : p(i) / resolution - idx(i);
    }
    t_max = t_max.cwiseProduct(delta_t);

    Eigen::Vector3i rayIdx = idx;
    while ((rayIdx - sensor_idx).squaredNorm() != 1) {
        // 选择步进最小的方向
        int s_dim = 0;
        for (int i = 1; i < 3; ++i) {
            if (t_max(i) < t_max(s_dim)) s_dim = i;
        }
        rayIdx(s_dim) += step(s_dim);
        t_max(s_dim) += delta_t(s_dim);

        if (vis.atId(rayIdx) != 1) {
            mis(rayIdx);  // 沿途清空
        }
    }
  }





  //--------------------------------------


  void updateMap(const Eigen::Vector3d& sensor_p,
                 const std::vector<Eigen::Vector3d>& pc);
  void occ2pc(sensor_msgs::PointCloud2& msg);
  void occ2pc(sensor_msgs::PointCloud2& msg, double floor, double ceil);
  void inflate_once();
  void inflate_xy();
  void inflate_last();
  void inflate(int inflate_size);
};

}  // namespace mapping