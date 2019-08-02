#include <algorithm>
#include <unordered_map>
#include <iostream>

#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <descriptor-projection/build-projection-matrix.h>
#include <descriptor-projection/flags.h>

namespace descriptor_projection {

// Compute the covariance of the descriptors, rows are states, columns are
// samples.
    void ComputeCovariance(
        const Eigen::MatrixXf& data, Eigen::MatrixXf* covariance) {
    CHECK_NOTNULL(covariance);
        CHECK_GT(data.cols(), 0) << "Data must not be empty!";
        VLOG(4) << "Got " << data.cols()
                << " samples to compute the covariance from.";
        covariance->setZero(data.rows(), data.rows());
        constexpr int kBlockSize = 10000;
        const int num_blocks = data.cols() / kBlockSize + 1;
        for (int i = 0; i < num_blocks; ++i) {
            const int block_start = i * kBlockSize;
            const int block_size =
                std::min<int>((i + 1) * kBlockSize, data.cols()) - block_start;
            const Eigen::Block<const Eigen::MatrixXf>& data_block =
                data.block(0, block_start, data.rows(), block_size);

            const Eigen::MatrixXf centered =
                data_block.colwise() - data_block.rowwise().mean();
            double normalizer = std::max(static_cast<int>(data_block.cols() - 1), 1);
            covariance->noalias() += (centered * centered.adjoint()) / normalizer;
        }
        (*covariance) /= num_blocks;
    }

    void BuildListOfMatchesAndNonMatches(
        const Eigen::MatrixXf& all_descriptors, const std::vector<std::vector<int>>& tracks,
        std::vector<descriptor_projection::DescriptorMatch>* matches,
        std::vector<descriptor_projection::DescriptorMatch>* non_matches) {
        CHECK_NOTNULL(matches);
        CHECK_NOTNULL(non_matches);
        for (const std::vector<int>& track : tracks) {
            // Add pairs of matching descriptors to the list of matches.
            // TODO(slynen): Consider taking random pairs to not under-estimate the
            // variance.
            for (size_t i = 1; i < track.size(); ++i) {
            matches->emplace_back(track[i - 1], track[i]);
            }
        }

        std::random_device device;
        std::mt19937 generator(device());
        std::uniform_int_distribution<> distribution(0, all_descriptors.cols() - 1);

        for (size_t i = 1; i < static_cast<size_t>(all_descriptors.cols()) &&
                            i < matches->size() * 2;
            ++i) {
            unsigned int index_a = distribution(generator);
            unsigned int index_b = distribution(generator);
            if (index_a == index_b) {
            continue;
            }
            non_matches->emplace_back(index_a, index_b);
        }
    }

    void BuildCovarianceMatricesOfMatchesAndNonMatches(
        unsigned int descriptor_size, const Eigen::MatrixXf& all_descriptors,
        const std::vector<std::vector<int>>& tracks, unsigned int* sample_size_matches,
        unsigned int* sample_size_non_matches, Eigen::MatrixXf* cov_matches,
        Eigen::MatrixXf* cov_non_matches) {
        CHECK_NOTNULL(sample_size_matches);
        CHECK_NOTNULL(sample_size_non_matches);
        CHECK_NOTNULL(cov_matches);
        CHECK_NOTNULL(cov_non_matches);

        {  // Scope to limit memory usage.
            unsigned int too_short_tracks = 0;
            unsigned int long_enough_tracks = 0;
            constexpr size_t kMinTrackLength = 5;
            size_t number_of_used_tracks = 0;
            Eigen::MatrixXf sumMuMu;
            sumMuMu.setZero(descriptor_size, descriptor_size);

            std::vector<size_t> descriptors_from_tracks;
            descriptors_from_tracks.reserve(500);

            // Centering.
            constexpr int kMaxNumSamples = 50000;
            for (const std::vector<int>& track : tracks) {
                if (track.size() < kMinTrackLength) {
                    ++too_short_tracks;
                    continue;
                }
                if (number_of_used_tracks >= kMaxNumSamples) {
                    LOG(WARNING) << "Truncated descriptors to " << kMaxNumSamples << ".";
                    break;
                }
                ++long_enough_tracks;

                Eigen::Matrix<float, Eigen::Dynamic, 1> mean;
                mean.resize(descriptor_size, Eigen::NoChange);
                mean.setZero();

                for (const size_t& descriptor_idx : track) {
                    descriptors_from_tracks.push_back(descriptor_idx);
                    mean += all_descriptors.block(0, descriptor_idx, descriptor_size, 1);
                }

                mean /= track.size();
                CHECK_LE(mean.maxCoeff(), 1.0);
                CHECK_GE(mean.minCoeff(), 0.0);

                Eigen::MatrixXf mu_sq_current = (mean * mean.transpose()).eval();

                sumMuMu += mu_sq_current * track.size();
                ++number_of_used_tracks;
            }
            
            std::cout<<"number_of_used_tracks: "<<number_of_used_tracks<<std::endl;

            VLOG(3) << "Got " << long_enough_tracks << " tracks out of "
                    << tracks.size() << " (dropped " << too_short_tracks
                    << " tracks because they were too short)";

            CHECK(!descriptors_from_tracks.empty());

            VLOG(3) << "Computing matches covariance from "
                    << descriptors_from_tracks.size()
                    << " matches (descriptor size: " << descriptor_size << ")";

            *sample_size_matches = descriptors_from_tracks.size();

            Eigen::MatrixXf matches;
            matches.resize(descriptor_size, descriptors_from_tracks.size());
            matches.setZero();

            int matched_idx = 0;
            for (const size_t& descriptor_idx : descriptors_from_tracks) {
            matches.block(0, matched_idx, descriptor_size, 1) =
                all_descriptors.block(0, descriptor_idx, descriptor_size, 1);
            ++matched_idx;
            }

            CHECK_GT(descriptors_from_tracks.size(), number_of_used_tracks);

            // Covariance computation for matches.
            cov_matches->noalias() =
                (matches * matches.transpose() - sumMuMu) * 2.0 /
                static_cast<float>(
                    descriptors_from_tracks.size() - number_of_used_tracks);
        }  // Scope to limit memory usage.

        // Use all descriptors to estimate the non-matching covariance.
        *sample_size_non_matches = all_descriptors.cols();
        // Compute sample covariances non matched descriptors.
        ComputeCovariance(all_descriptors, cov_non_matches);

        *cov_non_matches *= 2.0f;
    }

    void ComputeProjectionMatrix(
        const Eigen::MatrixXf& cov_matches, const Eigen::MatrixXf& cov_non_matches,
        Eigen::MatrixXf* A) {
        CHECK_NOTNULL(A);

        const int dimensionality = cov_matches.cols();
        A->resize(dimensionality, dimensionality);
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_matches, Eigen::ComputeFullV);

        CHECK_NE(svd.singularValues().minCoeff(), 0)
            << "Rank deficiency for matrix"
                " of samples detected. Probably too little matches.";

        Eigen::MatrixXf Av =
            svd.singularValues().cwiseSqrt().cwiseInverse().asDiagonal() *
            svd.matrixV().transpose();

        Eigen::JacobiSVD<Eigen::MatrixXf> svd_d(
            Av * cov_non_matches * Av.transpose(), Eigen::ComputeFullV);

        Eigen::MatrixXf singular_values_sqrt_inv =
            svd_d.singularValues().cwiseSqrt().cwiseInverse().asDiagonal();

        Eigen::MatrixXf eye;
        eye.resize(dimensionality, dimensionality);
        eye.setIdentity();

        *A = (eye - singular_values_sqrt_inv) * svd_d.matrixV().transpose() *
            svd.singularValues().cwiseInverse().asDiagonal() *
            svd.matrixV().transpose();
        Eigen::MatrixXf temp_mat = (*A)*cov_non_matches*(*A).transpose();
        Eigen::EigenSolver<Eigen::MatrixXf> es(temp_mat);
        Eigen::MatrixXcf eigen_vec_c = es.eigenvectors();
        Eigen::MatrixXf eigen_vec_r=eigen_vec_c.real();
        Eigen::MatrixXf trimed_A = eigen_vec_r.block(0,0,(*A).cols(),10).transpose()*(*A);
        //std::cout<<es.eigenvectors().row(0)<<std::endl;
        std::cout<<trimed_A.row(0)<<std::endl;
        std::cout<<(*A).col(0).transpose()<<std::endl;
        *A=trimed_A;
    }
    
    template <typename DerivedIn, typename DerivedOut>
    void DescriptorToEigenMatrixDef(
        const Eigen::MatrixBase<DerivedIn>& descriptor,
        const Eigen::MatrixBase<DerivedOut>& matrix_const) {
        EIGEN_STATIC_ASSERT(
            !(Eigen::internal::traits<DerivedOut>::Flags & Eigen::RowMajorBit),
            "This method is only valid for column major matrices");
        CHECK_EQ(descriptor.cols(), 1);

        Eigen::MatrixBase<DerivedOut>& matrix =
            const_cast<Eigen::MatrixBase<DerivedOut>&>(matrix_const);
        const int num_descriptor_bytes = descriptor.rows();
        const int num_descriptor_bits = num_descriptor_bytes * 8;
        CHECK_EQ(matrix.rows(), num_descriptor_bits)
            << "The matrix passed must be preallocated to match the descriptor "
                "length in bits, which is "
            << num_descriptor_bits << ".";
        matrix.setZero();

        CHECK_EQ(num_descriptor_bytes % 16, 0);
        
        // Define a set of macros to NEON and SSE instructions so we can use the same
        // code further down for both platforms.
#ifdef __ARM_NEON__
    #define VECTOR_SET vdupq_n_u8       // Set a vector from a single uint8.
    #define VECTOR_LOAD(x) vld1q_u8(x)  // Set a vector from a mem location.
    #define VECTOR_TYPE uint8x16_t      // The type of the vector element.
    #define VECTOR_AND vandq_u8         // The vector AND instruction.
    #define VECTOR_EXTRACT(x, i) vgetq_lane_u8(x, i)  // Get element from vector.
#else
    #define VECTOR_SET _mm_set1_epi8
    #define VECTOR_LOAD(x) _mm_load_si128(reinterpret_cast<const __m128i*>(x))
    #define VECTOR_TYPE __m128i
    #define VECTOR_AND _mm_and_si128
    // Could use _mm_extract_epi8, but this requires SSE4.1.
    #define VECTOR_EXTRACT(x, i) reinterpret_cast<const char*>(&x)[i]
#endif  // ANDROID

        VECTOR_TYPE mask[8];
        mask[0] = VECTOR_SET((1 << 0));
        mask[1] = VECTOR_SET((1 << 1));
        mask[2] = VECTOR_SET((1 << 2));
        mask[3] = VECTOR_SET((1 << 3));
        mask[4] = VECTOR_SET((1 << 4));
        mask[5] = VECTOR_SET((1 << 5));
        mask[6] = VECTOR_SET((1 << 6));
        mask[7] = VECTOR_SET((1 << 7));

        float* matrix_ref = matrix.derived().data();

        CHECK_EQ(descriptor.derived().cols(), 1);

        const unsigned char* descriptor_data = &descriptor.derived().coeffRef(0, 0);

        for (int pack = 0; pack < num_descriptor_bytes / 16; ++pack) {
            VECTOR_TYPE value = VECTOR_LOAD(descriptor_data + pack * 16);
            const int pack128 = pack << 7;
            for (int i = 0; i < 8; ++i) {  // Checks 16 bits at once with SSE/NEON.
            // Masks the i'th bit of the 16 uint8s.
            VECTOR_TYPE xmm1 = VECTOR_AND(value, mask[i]);
            if (VECTOR_EXTRACT(xmm1, 0))
                matrix_ref[pack128 + i + 0] = 1;
            if (VECTOR_EXTRACT(xmm1, 1))
                matrix_ref[pack128 + i + 8] = 1;
            if (VECTOR_EXTRACT(xmm1, 2))
                matrix_ref[pack128 + i + 16] = 1;
            if (VECTOR_EXTRACT(xmm1, 3))
                matrix_ref[pack128 + i + 24] = 1;
            if (VECTOR_EXTRACT(xmm1, 4))
                matrix_ref[pack128 + i + 32] = 1;
            if (VECTOR_EXTRACT(xmm1, 5))
                matrix_ref[pack128 + i + 40] = 1;
            if (VECTOR_EXTRACT(xmm1, 6))
                matrix_ref[pack128 + i + 48] = 1;
            if (VECTOR_EXTRACT(xmm1, 7))
                matrix_ref[pack128 + i + 56] = 1;
            if (VECTOR_EXTRACT(xmm1, 8))
                matrix_ref[pack128 + i + 64] = 1;
            if (VECTOR_EXTRACT(xmm1, 9))
                matrix_ref[pack128 + i + 72] = 1;
            if (VECTOR_EXTRACT(xmm1, 10))
                matrix_ref[pack128 + i + 80] = 1;
            if (VECTOR_EXTRACT(xmm1, 11))
                matrix_ref[pack128 + i + 88] = 1;
            if (VECTOR_EXTRACT(xmm1, 12))
                matrix_ref[pack128 + i + 96] = 1;
            if (VECTOR_EXTRACT(xmm1, 13))
                matrix_ref[pack128 + i + 104] = 1;
            if (VECTOR_EXTRACT(xmm1, 14))
                matrix_ref[pack128 + i + 112] = 1;
            if (VECTOR_EXTRACT(xmm1, 15))
                matrix_ref[pack128 + i + 120] = 1;
            }
        }
    }

    void ProjectDescriptor(
        const Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& raw_descriptor,
        const Eigen::MatrixXf& projection_matrix, int target_dimensions,
        Eigen::VectorXf& projected_descriptor) {
        Eigen::Matrix<float, Eigen::Dynamic, 1> descriptor;
        const int descriptor_bits = raw_descriptor.size() * 8;
        descriptor.resize(descriptor_bits, Eigen::NoChange);
        DescriptorToEigenMatrixDef(raw_descriptor, descriptor);
        projected_descriptor = projection_matrix.block(0, 0, target_dimensions, descriptor.rows()) * descriptor;
    }
    
    void DescriptorToEigenMatrix( const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& descriptor, Eigen::MatrixXf& matrix_out){
        DescriptorToEigenMatrixDef(descriptor, matrix_out);
    }
}  // namespace descriptor_projection
