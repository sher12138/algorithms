#pragma once
#include <Eigen/Dense>

#include <iostream>
#include <vector>

namespace neodrive {
namespace planning_rl {
class Transformer {
 public:
  static Eigen::VectorXf float_2_vector(std::vector<float>& scales) {
    Eigen::Map<Eigen::VectorXf> ans(scales.data(), scales.size());
    return ans;
  }

  static Eigen::RowVectorXf float_2_row_vector(std::vector<float>& scales) {
    Eigen::Map<Eigen::RowVectorXf> ans(scales.data(), scales.size());
    return ans;
  }

  static Eigen::VectorXf float_2_vector(const std::vector<float>& scales) {
    Eigen::Map<Eigen::VectorXf> ans(scales.data(), scales.size());
    return ans;
  }

  static Eigen::MatrixXf pairs_2_matrix(
      std::vector<std::pair<float, float>> pairs) {
    long rows = pairs.size();
    Eigen::MatrixXf ans(rows, 2);
    for (int i = 0; i < rows; ++i) {
      std::vector<float> tmp = {pairs[i].first, pairs[i].second};
      ans.col(i) = float_2_row_vector(tmp);
    }
    return ans;
  }

  static Eigen::RowVectorXf float_2_row_vector(
      const std::vector<float>& scales) {
    Eigen::Map<Eigen::RowVectorXf> ans(scales.data(), scales.size());
    return ans;
  }

  static float* _1d_flatten(Eigen::RowVectorXf matrix) {
    float* pointer = new float[matrix.size()];
    Eigen::Map<Eigen::MatrixXf>(pointer, matrix.rows(), matrix.cols()) = matrix;
    return pointer;
  }

  static Eigen::MatrixXf points_2_vector(
      const std::vector<std::vector<float>>& matrix) {
    int cols = matrix.size(), rows = matrix[0].size();
    Eigen::MatrixXf ans(cols, rows);
    for (int i = 0; i < cols; ++i) {
      ans.col(i) = float_2_row_vector(matrix[i]);
    }
    return ans;
  }
  static Eigen::MatrixXf pairs_2_matrix(
      const std::vector<std::pair<double, double>>& pairs) {
    long rows = pairs.size();
    Eigen::MatrixXf ans(rows, 2);
    for (int i = 0; i < rows; ++i) {
      std::vector<float> tmp = {static_cast<float>(pairs[i].first),
                                static_cast<float>(pairs[i].second)};
      ans.col(i) = float_2_row_vector(tmp);
    }
    return ans;
  }

  static std::vector<float> __2d_flatten(Eigen::MatrixXf matrix) {
    float* pointer = new float[matrix.size()];
    Eigen::Map<Eigen::MatrixXf>(pointer, matrix.rows(), matrix.cols()) = matrix;
    std::vector<float> ans =
        std::vector<float>(pointer, pointer + matrix.size());
    return ans;
  }

  static Eigen::MatrixXf pairs_2_matrix(
      const std::vector<std::pair<float, float>>& pairs) {
    long rows = pairs.size();
    Eigen::MatrixXf ans(rows, 2);
    for (int i = 0; i < rows; ++i) {
      std::vector<float> tmp = {pairs[i].first, pairs[i].second};
      ans.col(i) = float_2_row_vector(tmp);
    }
    return ans;
  }

  static Eigen::RowVectorXf vector_2_row_vector(
      const std::vector<float>& vector) {
    Eigen::Map<Eigen::RowVectorXf> ans(vector.data(), vector.size());
    return ans;
  }
  static Eigen::RowVectorXf pair_2_row_vector(
      const std::pair<float, float>& pair) {
    float array[2] = {pair.first, pair.second};
    Eigen::Map<Eigen::RowVectorXf> orig(array, 2);
    return orig;
  }
  static Eigen::MatrixXf cumsum(Eigen::MatrixXf x) {
    int cols = x.cols(), rows = x.rows();
    Eigen::MatrixXf ans = Eigen::MatrixXf(rows, cols);
    for (int i = 0; i < rows; ++i) {
      if (i == 0) {
        ans.row(i) = x.row(i);
      } else {
        ans.row(i) = x.row(i) + x.row(i - 1);
      }
    }
    return ans;
  }

  static float* _1d_flatten(const Eigen::RowVectorXf& matrix) {
    float* pointer = new float[matrix.size()];
    Eigen::Map<Eigen::MatrixXf>(pointer, matrix.rows(), matrix.cols()) = matrix;
    return pointer;
  }

  static float* _2d_flatten(const std::vector<Eigen::RowVectorXf>& vectors) {
    float* pointer = new float[vectors.size() * vectors[0].size()];
    int index = 0;
    for (int i = 0; i < vectors.size(); ++i) {
      auto* ans = _1d_flatten(vectors[i]);
      for (int j = 0; j < vectors[i].size(); ++j) {
        pointer[index++] = *(ans + j);
      }
    }
    return pointer;
  }
  static std::vector<float> __2d_flatten(
      const std::vector<Eigen::RowVectorXf>& vectors) {
    int size = vectors.size() * vectors[0].size();
    auto* tmp = _2d_flatten(vectors);
    auto ans = std::vector<float>(tmp, tmp + size);
    return ans;
  }
  static float* _2d_flatten(const Eigen::MatrixXf& matrix) {
    float* pointer = new float[matrix.size()];
    Eigen::Map<Eigen::MatrixXf>(pointer, matrix.rows(), matrix.cols()) = matrix;
    return pointer;
  }

  static std::vector<float> __2d_flatten(const Eigen::MatrixXf& matrix) {
    float* pointer = new float[matrix.size()];
    Eigen::Map<Eigen::MatrixXf>(pointer, matrix.rows(), matrix.cols()) = matrix;
    std::vector<float> ans =
        std::vector<float>(pointer, pointer + matrix.size());
    return ans;
  }

  static float* _3d_flatten(const std::vector<Eigen::MatrixXf>& matrics) {
    float* pointer = new float[matrics.size() * matrics[0].size()];
    int size = matrics[0].size();
    for (int i = 0; i < matrics.size(); ++i) {
      auto* pointer2 = _2d_flatten(matrics[i]);
      for (int j = 0; j < matrics[i].size(); j++) {
        pointer[i * size + j] = *(pointer2 + j);
      }
    }
    return pointer;
  }

  static std::vector<float> __3d_flatten(
      const std::vector<Eigen::MatrixXf>& matrics) {
    int size = matrics.size() * matrics[0].size();
    auto* pointer = _3d_flatten(matrics);
    std::vector<float> ans = std::vector<float>(pointer, pointer + size);
    return ans;
  }

  static Eigen::MatrixXf cumsum(const Eigen::MatrixXf& x) {
    int cols = x.cols(), rows = x.rows();
    Eigen::MatrixXf ans = Eigen::MatrixXf(rows, cols);
    for (int i = 0; i < rows; ++i) {
      if (i == 0) {
        ans.row(i) = x.row(i);
      } else {
        ans.row(i) = x.row(i) + x.row(i - 1);
      }
    }
    return ans;
  }

  static Eigen::MatrixXf expand(const Eigen::RowVectorXf& x, const int& rows) {
    Eigen::MatrixXf ans(rows, x.size());
    for (int i = 0; i < rows; i++) {
      ans.row(i) = x;
    }
    return ans;
  }

  static std::vector<std::pair<float, float>> _2d_matrix_2_vector_pair(
      const Eigen::MatrixXf& matrix) {
    int cols = matrix.cols(), rows = matrix.rows();
    std::vector<std::pair<float, float>> ans;
    for (int i = 0; i < rows; ++i) {
      std::pair<float, float> pair = {matrix.row(i)[0], matrix.row(i)[1]};
      ans.push_back(pair);
    }
    return ans;
  }

  static Eigen::MatrixXf flatten_2_matrix(const std::vector<float>& vector,
                                          const int& rows, const int& cols) {
    /***
     * flatten后的结果转换成为matrix，注意：以列优先进行排列
     */
    Eigen::Map<Eigen::MatrixXf> ans(vector.data(), rows, cols);
    return ans;
  }

};  // namespace neodrive
}  // namespace planning_rl
}  // namespace neodrive
