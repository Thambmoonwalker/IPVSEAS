#include "tracking/hungarian.hpp"

#include <algorithm>
#include <limits>

namespace ipvseas {

std::vector<int> hungarian_assign(const Eigen::MatrixXd& cost, double forbidden) {
    const int rows = static_cast<int>(cost.rows());
    const int cols = static_cast<int>(cost.cols());
    std::vector<int> result(rows, -1);
    if (rows == 0 || cols == 0) return result;

    // Square matrix padded with a large (but finite) cost.
    const int n = std::max(rows, cols);
    const double big = forbidden * 10.0 + 1.0;
    Eigen::MatrixXd a = Eigen::MatrixXd::Constant(n, n, big);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j) a(i, j) = std::min(cost(i, j), big);

    // Classic potentials-based implementation (1-indexed internally).
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
    std::vector<int> p(n + 1, 0), way(n + 1, 0);
    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(n + 1, INF);
        std::vector<bool> used(n + 1, false);
        do {
            used[j0] = true;
            const int i0 = p[j0];
            double delta = INF;
            int j1 = 0;
            for (int j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const double cur = a(i0 - 1, j - 1) - u[i0] - v[j];
                if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                if (minv[j] < delta) { delta = minv[j]; j1 = j; }
            }
            for (int j = 0; j <= n; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else { minv[j] -= delta; }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    for (int j = 1; j <= n; ++j) {
        const int i = p[j] - 1, c = j - 1;
        if (i >= 0 && i < rows && c < cols && cost(i, c) < forbidden) result[i] = c;
    }
    return result;
}

}  // namespace ipvseas
