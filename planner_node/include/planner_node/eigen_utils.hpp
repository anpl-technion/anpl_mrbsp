/* ---------------------------------------------------------------------------
 *
 * Autonomous Navigation and Perception Lab (ANPL),
 * Technion, Israel Institute of Technology,
 * Faculty of Aerospace Engineering,
 * Haifa, Israel, 32000
 * All Rights Reserved
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */
/**
 * @file: eigen_utils.hpp
 * @brief:
 * @author: Andrej Kitanov
 *
 */

#ifndef PLANNER_NODE_EIGEN_UTILS_HPP
#define PLANNER_NODE_EIGEN_UTILS_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>

// This function performs row or column sum of a sparse matrix X and returns a result in the row vector v
template <typename T, typename S> inline void sumAlongDim(
        const Eigen::SparseMatrix<T>& X,
        const int dim,
        Eigen::Matrix<S, Eigen::Dynamic, 1>& v)
{
    // Get size of input
    int m = X.rows();
    int n = X.cols();
    // resize output
    if(dim==1)
    {
        v = Eigen::Matrix<S, Eigen::Dynamic, 1>(n);
    }else
    {
        v = Eigen::Matrix<S, Eigen::Dynamic, 1>(m);
    }

    v.setZero();

    // Iterate over outside
    for(int k=0; k<X.outerSize(); ++k)
    {
        // Iterate over inside
        for(typename Eigen::SparseMatrix<T>::InnerIterator it(X,k); it; ++it)
        {
            if(dim == 1)
            {
                v[it.col()] += it.value();
            }else
            {
                v[it.row()] += it.value();
            }
        }
    }
}

/*void example () {
    SparseMatrix<double> mat(5,5);
    for (int k = 0; k < mat.outerSize(); ++k)
        for (SparseMatrix<double>::InnerIterator it(mat, k); it; ++it) {
            it.value();
            it.row();   // row index
            it.col();   // col index (here it is equal to k)
            it.index(); // inner index, here it is equal to it.row()
        }
}*/



template<typename MatrixType>
void saveMatrixToFile(const char *filename, const MatrixType& m)
{
    ofstream f(filename, ios::binary);
    typename MatrixType::Index rows, cols;
    rows = m.rows();
    cols = m.cols();
    f.write((char *)&rows, sizeof(m.rows()));
    f.write((char *)&cols, sizeof(m.cols()));
    f.write((const char *)(m.data()), sizeof(typename MatrixType::Scalar)*m.rows()*m.cols());
    f.close();
}

template<typename MatrixType>
void loadMatrixFromFile(const char *filename, MatrixType& m)
{
    typename MatrixType::Index rows, cols;
    ifstream f(filename, ios::binary);
    f.read((char *)&rows, sizeof(rows));
    f.read((char *)&cols, sizeof(cols));
    m.resize(rows, cols);
    typename MatrixType::Scalar* p = m.data();
    f.read((char *)p, sizeof(typename MatrixType::Scalar)*rows*cols);
    if (f.bad())
        throw "Error reading matrix";
    f.close();
}

#include <opencv2/core/core.hpp>
template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> &src, cv::Mat &dst) {
    if (!(src.Flags & Eigen::RowMajorBit))
    {
        cv::Mat _src(src.cols(), src.rows(), cv::DataType<_Tp>::type,
                     (void*)src.data(), src.stride() * sizeof(_Tp));
        cv::transpose(_src, dst);
    }
    else
    {
        cv::Mat _src(src.rows(), src.cols(), cv::DataType<_Tp>::type,
                     (void*)src.data(), src.stride() * sizeof(_Tp));
        _src.copyTo(dst);
    }
}

#endif //PLANNER_NODE_EIGEN_UTILS_HPP
