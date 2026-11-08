// Optimal assignment (Hungarian / Kuhn-Munkres, O(n^3)).
#pragma once

#include <Eigen/Dense>
#include <vector>

namespace ipvseas {

// cost(i, j) = cost of assigning row i to column j. Entries >= forbidden are never assigned.
// Returns, for each row, the assigned column or -1.
std::vector<int> hungarian_assign(const Eigen::MatrixXd& cost, double forbidden);

}  // namespace ipvseas
