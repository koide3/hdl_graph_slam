#ifndef ROBUST_KERNEL_IO_HPP
#define ROBUST_KERNEL_IO_HPP

#include <string>
#include <g2o/core/sparse_optimizer.h>


namespace g2o {

std::string kernel_type(g2o::RobustKernel* kernel);

bool save_robust_kernels(const std::string& filename, g2o::SparseOptimizer* graph);

bool load_robust_kernels(const std::string& filename, g2o::SparseOptimizer* graph);

}

#endif // ROBUST_KERNEL_IO_HPP
