#ifndef DESCRIPTOR_PROJECTION_BUILD_PROJECTION_MATRIX_H_
#define DESCRIPTOR_PROJECTION_BUILD_PROJECTION_MATRIX_H_

#include <algorithm>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>

namespace descriptor_projection {
    typedef std::pair<unsigned int, unsigned int> DescriptorMatch;
    void ComputeCovariance(
        const Eigen::MatrixXf& data, Eigen::MatrixXf* covariance);

    void BuildListOfMatchesAndNonMatches(
        const Eigen::MatrixXf& all_descriptors, const std::vector<std::vector<int>>& tracks,
        std::vector<descriptor_projection::DescriptorMatch>* matches,
        std::vector<descriptor_projection::DescriptorMatch>* non_matches);

    void BuildCovarianceMatricesOfMatchesAndNonMatches(
        unsigned int descriptor_size, const Eigen::MatrixXf& all_descriptors,
        const std::vector<std::vector<int>>& tracks, unsigned int* sample_size_matches,
        unsigned int* sample_size_non_matches, Eigen::MatrixXf* cov_matches,
        Eigen::MatrixXf* cov_non_matches);

    void ComputeProjectionMatrix(
        const Eigen::MatrixXf& cov_matches, const Eigen::MatrixXf& cov_non_matches,
        Eigen::MatrixXf* A);
    
    void ProjectDescriptor(
        const Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& raw_descriptor,
        const Eigen::MatrixXf& projection_matrix, int target_dimensions,
        Eigen::VectorXf& projected_descriptor);
    
    void DescriptorToEigenMatrix(
        const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& descriptor,
        Eigen::MatrixXf& matrix_const);
}  // namespace descriptor_projection
#endif  // DESCRIPTOR_PROJECTION_BUILD_PROJECTION_MATRIX_H_
