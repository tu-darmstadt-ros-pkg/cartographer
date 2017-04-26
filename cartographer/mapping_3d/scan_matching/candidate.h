

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CANDIDATE_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CANDIDATE_H_


namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

struct DiscreteScan {
  transform::Rigid3f pose;
  // Contains a vector of discretized scans for each 'depth'.
  std::vector<std::vector<Eigen::Array3i>> cell_indices_per_depth;
};

struct ContinuousScan {
  transform::Rigid3f pose;
  // Contains a vector of discretized scans for each 'depth'.
  std::vector<std::vector<Eigen::Array3f>> transformed_points_per_depth;
};

struct Candidate {
  Candidate(const int scan_index, const Eigen::Array3i& offset)
      : scan_index(scan_index), offset(offset) {}

  // Index into the discrete scans vectors.
  int scan_index;

  // Linear offset from the initial pose in cell indices. For lower resolution
  // candidates this is the lowest offset of the 2^depth x 2^depth x 2^depth
  // block of possibilities.
  Eigen::Array3i offset;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate& other) const { return score < other.score; }
  bool operator>(const Candidate& other) const { return score > other.score; }
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CANDIDATE_H_
