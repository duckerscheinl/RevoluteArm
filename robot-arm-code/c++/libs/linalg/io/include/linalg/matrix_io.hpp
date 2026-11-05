#pragma once
#include <cstddef>
#include <concepts>
#include <iostream>
#include "linalg/matrix.hpp"

namespace linalg {

template<std::floating_point T, std::size_t n_rows, std::size_t n_cols>
std::ostream& operator<<(std::ostream& os, const Matrix<T,n_rows,n_cols>& M){
    for(std::size_t i=0; i<n_rows; i++){
        for(std::size_t j=0; j<n_cols; j++){
            os << (j ? "\t": "") << M[i,j];
        }
        os << "\n";
    }
    return os;
}

}