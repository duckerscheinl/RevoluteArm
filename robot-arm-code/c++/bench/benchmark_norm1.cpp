// Timing benchmark: column-sum 1-norm, row-wise traversal vs column-wise traversal.
//
// Times linalg::norm_1 and linalg::norm_1_naive from linalg/matrix.hpp.
// Matrix stores its elements row-major, so
//   norm_1        walks memory contiguously (row by row),
//   norm_1_naive  jumps by n_cols elements between accesses (column by column).
//
// Requires: both norm functions take the matrix as const Matrix& (a by-value copy of a
// large matrix would overflow the stack), and Matrix has const overloads of operator[].
// For matrices larger than the CPU caches, the strided access should be much slower.
//
// Build in Release mode (-O2, no sanitizers), otherwise the timings are meaningless.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <random>
#include "linalg/matrix.hpp"

using linalg::Matrix;
using linalg::norm_1;

template<typename T, size_t n_rows, size_t n_cols>
T norm_1_naive(Matrix<T,n_rows,n_cols>& M){
    T norm = 0;
    T norm_c = 0;
    for (size_t j = 0; j < n_cols; j++){
        for(size_t i = 0; i < n_rows; i++){
            norm_c += std::abs(M[i,j]);
        }
        if (norm_c > norm){
            norm = norm_c;
        }
        norm_c = 0;
    }
    return norm;
}

// Average time per call in seconds; repeats until at least min_time has passed
template<typename F>
double time_per_call(F&& f, double min_time = 0.3)
{
    using clock = std::chrono::steady_clock;
    f();                                        // warm-up
    long calls = 0;
    auto start = clock::now();
    double elapsed = 0.0;
    do {
        f();
        ++calls;
        elapsed = std::chrono::duration<double>(clock::now() - start).count();
    } while (elapsed < min_time);
    return elapsed / double(calls);
}

static volatile double sink;                    // keeps the compiler from removing the calls

template<typename T, std::size_t N>
void run()
{
    // A large Matrix does not fit on the stack, so it is allocated on the heap
    auto M = std::make_unique<Matrix<T, N, N>>();
    std::mt19937 rng(0);
    std::uniform_real_distribution<T> dist(-1, 1);
    for (std::size_t i = 0; i < N; i++)
        for (std::size_t j = 0; j < N; j++)
            (*M)[i, j] = dist(rng);

    // Both versions must agree before their timings mean anything.
    // The sums are formed in a different order, so allow for rounding differences.
    T a = norm_1(*M), b = norm_1_naive(*M);
    bool agree = std::abs(a - b) <= T(1e-4) * std::max(a, b);

    double t_row = time_per_call([&] { sink = norm_1(*M); });
    double t_col = time_per_call([&] { sink = norm_1_naive(*M); });
    double elems = double(N) * double(N);
    double mbytes = elems * sizeof(T) / (1024.0 * 1024.0);

    std::printf("%6zu x %-6zu %9.2f MB | %8.3f ns/elem  %8.3f ns/elem | %6.2fx %s\n",
                N, N, mbytes, 1e9 * t_row / elems, 1e9 * t_col / elems,
                t_col / t_row, agree ? "" : "  RESULTS DIFFER!");
}

int main()
{
    std::printf("%-15s %12s | %17s %17s | %s\n", "size", "memory", "row-wise", "column-wise", "col/row");
    run<float, 4>();
    run<float, 16>();
    run<float, 64>();
    run<float, 256>();
    run<float, 512>();
    run<float, 1024>();
    run<float, 2048>();
    run<float, 4096>();
    return 0;
}