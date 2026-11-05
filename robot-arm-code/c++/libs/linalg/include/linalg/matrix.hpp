#pragma once
#include <cmath>
#include <array>
#include <cassert>
#include <cstddef>

namespace linalg {

template<typename T, std::size_t n_rows, std::size_t n_cols>
class  Matrix
{
private:
    std::array<T, n_rows*n_cols> data_{}; 

public:
    using value_type  = T;
    using iterator = typename std::array<T, n_rows * n_cols>::iterator;
    using const_iterator = typename std::array<T, n_rows * n_cols>::const_iterator;

    constexpr Matrix() = default;
    constexpr Matrix(const std::array<std::array<T, n_cols>, n_rows>& data);
    constexpr T& operator[](std::size_t ind);
    constexpr const T& operator[](std::size_t ind) const;
    constexpr T& operator[](std::size_t row, std::size_t col);
    constexpr const T& operator[](std::size_t row, std::size_t col) const;
    constexpr iterator begin(){ return data_.begin(); }
    constexpr iterator end(){ return data_.end(); }
    constexpr const_iterator begin() const { return data_.begin(); }
    constexpr const_iterator end() const { return data_.end(); }
};

template<typename T, std::size_t length>
using Vector = Matrix<T, length, 1>;

template<typename T, std::size_t n_rows, std::size_t n_cols>
constexpr Matrix<T, n_rows, n_cols>::Matrix(const std::array<std::array<T, n_cols>, n_rows>& data)
{
    for (std::size_t i = 0; i < n_rows; i++) {
        for (std::size_t j = 0; j < n_cols; j++){
            (*this)[i,j] = data[i][j];
        }
    }
}

template<typename T, std::size_t n_rows, std::size_t n_cols>
constexpr T& Matrix<T, n_rows, n_cols>::operator[](std::size_t ind)
{
    assert(ind < n_rows*n_cols);
    return data_[ind];
}

template<typename T, std::size_t n_rows, std::size_t n_cols>
constexpr const T& Matrix<T, n_rows, n_cols>::operator[](std::size_t ind) const
{
    assert(ind < n_rows*n_cols);
    return data_[ind];
}

// C++ is row major
template<typename T, std::size_t n_rows, std::size_t n_cols>
constexpr T& Matrix<T, n_rows, n_cols>::operator[](std::size_t row, std::size_t col)
{
    assert(row < n_rows && col < n_cols); 
    return data_[row*n_cols+col];
}

template<typename T, std::size_t n_rows, std::size_t n_cols>
constexpr const T& Matrix<T, n_rows, n_cols>::operator[](std::size_t row, std::size_t col) const
{
    assert(row < n_rows && col < n_cols);
    return data_[row*n_cols+col];
}

template<typename T, std::size_t n_rows1, std::size_t n_cols1, std::size_t n_cols2>
constexpr Matrix<T, n_rows1, n_cols2> operator*(const Matrix<T,n_rows1,n_cols1>& A, const Matrix<T,n_cols1,n_cols2>& B)
{
    Matrix<T,n_rows1,n_cols2> prod;
    for (std::size_t i=0; i<n_rows1; i++){
        for (std::size_t j=0; j<n_cols1; j++){
            for (std::size_t k=0; k<n_cols2; k++){
                prod[i,k] += A[i,j]*B[j,k];
            }
        }
    }

    return prod;
}

template<typename T, std::size_t n_rows, std::size_t n_cols>
constexpr Matrix<T, n_rows, n_cols> operator-(const Matrix<T,n_rows,n_cols>& A, const Matrix<T,n_rows,n_cols>& B)
{
    Matrix<T,n_rows,n_cols> sub;
    for (std::size_t i=0; i<n_rows; i++){
        for (std::size_t j=0; j<n_cols; j++){
            sub[i,j] = A[i,j]-B[i,j];
        }
    }

    return sub;
}

template<typename T, std::size_t n_rows, std::size_t n_cols>
constexpr Matrix<T, n_rows, n_cols> operator+(const Matrix<T,n_rows,n_cols>& A, const Matrix<T,n_rows,n_cols>& B)
{
    Matrix<T,n_rows,n_cols> add;
    for (std::size_t i=0; i<n_rows; i++){
        for (std::size_t j=0; j<n_cols; j++){
            add[i,j] = A[i,j]+B[i,j];
        }
    }

    return add;
}

template<typename T, std::size_t n_rows, std::size_t n_cols>
constexpr Matrix<T, n_rows, n_cols> operator*(std::type_identity_t<T> a, const Matrix<T, n_rows, n_cols>& M)
{
    Matrix<T, n_rows, n_cols> res;
    for (std::size_t i = 0; i < n_rows * n_cols; i++)
        res[i] = a * M[i];
    return res;
}

template<typename T, std::size_t n_rows, std::size_t n_cols>
constexpr Matrix<T, n_rows, n_cols> operator*(const Matrix<T, n_rows, n_cols>& M, std::type_identity_t<T> a)
{
    return a * M;
}

// Dot product
template<typename T, std::size_t length>
constexpr T dot(const Vector<T, length>& u, const Vector<T, length>& v)
{
    T res = 0;
    for (std::size_t i = 0; i < length; i++)
        res += u[i] * v[i];
    return res;
}

template<typename T, std::size_t length>
T norm2(const Vector<T, length>& vec){
    T norm = 0;
    for (const auto& el : vec){
        norm += el*el;
    }
    return std::sqrt(norm);
}

//Technically not a matrix norm
template<typename T, std::size_t n_rows, std::size_t n_cols>
T norm_el(const Matrix<T,n_rows,n_cols>& M){
    T norm = 0;
    for (int i = 0; i < n_rows*n_cols; i++){
        norm += std::abs(M[i]);
    }
    return norm;
}

//Optimized for cache and vectorization.
template<typename T, std::size_t n_rows, std::size_t n_cols>
T norm_1(const Matrix<T,n_rows,n_cols>& M){
    std::array<T, n_cols> c_norms{};
    for (std::size_t i = 0; i < n_rows; i++){
        for(std::size_t j = 0; j < n_cols; j++){
            c_norms[j] += std::abs(M[i,j]);
        }
    }
    T norm = 0;
    for (std::size_t j = 0; j < n_cols; j++){
        if (c_norms[j] > norm){
            norm = c_norms[j];
        }
    }
    return norm;
}


// Special Matrices

template<typename T, std::size_t size>
constexpr Matrix<T, size, size> identity(){
    Matrix<T,size,size> id{};
    for(std::size_t i = 0; i < size; i++)
        id[i,i] = T(1);
    return id;
}

}