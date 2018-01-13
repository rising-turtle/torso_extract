/*
    Jan. 13 2018, He Zhang, hxzhang1@ualr.edu 

    estimate a linear x' = alpha * x , that minimize the difference between y and x'  

*/

#include "ceres/ceres.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std;

const int kNumObservations = 12*5 + 13*5;
const double data[] = {
    -6, -3.49, -6, -3.49, -6, -3.51, -6, -3.63, -6, -3.55,
    -8, -4.19, -8, -3.05, -8, -4.15, -8, -4.01, -8, -4.26,
    -10, -4.59, -10, -4.64, -10, -4.54, -10, -4.52, -10, -4.58,
    -12, -4.81, -12, -4.57, -12, -4.88, -12, -4.67, -12, -4.56,
    -14, -4.63, -14, -4.67, -14, -4.74, -14, -4.63, -14, -4.66,
    -16, -5.27, -16, -5.13, -16, -5.31, -16, -5.22, -16, -5.1,
    -18, -5.96, -18, -5.82, -18, -6.38, -18, -6.24, -18, -6.61,
    -20, -7.43, -20, -7.28, -20, -7.35, -20, -7.39, -20, -7.55,
    -22, -8.33, -22, -8.36, -22, -8.32, -22, -8.12, -22, -8.08,
    -24, -9.6, -24, -9.29, -24, -9.32, -24, -9.19, -24, -9.3,
    -26, -10.35, -26, -10.22, -26, -10.16, -26, -10.07, -26, -10.27,
    -28, -11.42, -28, -11.29, -28, -11.36, -28, -11.19, -28, -11.17,
    6, 3.74, 6, 3.62, 6, 3.33, 6, 3.15, 6, 3.31,
    8, 4, 8, 3.88, 8, 3.87, 8, 3.87, 8, 3.94,
    10, 4.89, 10, 4.72, 10, 4.22, 10, 4.26, 10, 4.27, 
    12, 5.1, 12, 4.91, 12, 4.43, 12, 4.19, 12, 4.17,
    14, 5.7, 14, 4.88, 14, 4.46, 14, 4.18, 14, 3.97,
    16, 5.9, 16, 5.13, 16, 5.06, 16, 4.84, 16, 4.74,
    18, 6.12, 18, 5.88, 18, 5.42, 18, 5.26, 18, 5.31,
    20, 5.52, 20, 5, 20, 4.55, 20, 4.72, 20, 4.54,
    22, 6, 22, 5.59, 22, 5.27, 22, 5.09, 22, 5.12,
    24, 7.1, 24, 6.63, 24, 6.17, 24, 6.2, 24, 6.42,
    26, 7.8, 26, 7.53, 26, 7.19, 26, 6.96, 26, 7.02,
    28, 8.42, 28, 8.23, 28, 7.92, 28, 7.72, 28, 7.82,
    30, 10.3, 30, 10.03, 30, 9.77, 30, 9.57, 30, 9.55,
};

struct ScaleResidual {
  ScaleResidual(double x, double y)
      : x_(x), y_(y) {}

  template <typename T> bool operator()(const T* const s,
                                        const T* const b,
                                        T* residual) const {
    residual[0] = y_ - s[0] * (x_ + b[0]);
    return true;
  }

 private:
  const double x_;
  const double y_;
};

struct FactorScaleResidual {
  FactorScaleResidual(double x, double y)
      : x_(x), y_(y) {}

  template <typename T> bool operator()(const T* const s,
                                        const T* const b,
					const T* const f,
                                        T* residual) const {
    T tnorm = atan(fabs(x_) - f[0]) + M_PI/2.;
    // residual[0] = y_ - tnorm * s[0] * (x_ + b[0]);
    residual[0] = y_ - tnorm * (x_ + b[0]);
    return true;
  }

 private:
  const double x_;
  const double y_;
};

int main(int argc, char** argv) 
{
  // google::InitGoogleLogging(argv[0]);

  double s = 1.0;
  double b = 1.0;
  double f = 3.0;

  Problem problem;
  for (int i = 0; i < kNumObservations; ++i) {
    // problem.AddResidualBlock(
    //     new AutoDiffCostFunction<ScaleResidual, 1, 1, 1>(
    //        new ScaleResidual(data[2*i+1], data[2 * i])),
    //    NULL,
    //    &s, &b);
    problem.AddResidualBlock(
        new AutoDiffCostFunction<FactorScaleResidual, 1, 1, 1, 1>(
            new FactorScaleResidual(data[2*i+1], data[2 * i])),
        NULL,
        &s, &b, &f);

    // cout<<"i = "<<i<<" x = "<<data[2*i+1]<<" y = "<<data[2*i]<<endl;
  }

  Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial s: " << 3.0 << " b: " << 1.0 << "f: "<<5.0 << "\n";
  std::cout << "Final   s: " << s << " b: " << b << " f: "<<f<< "\n";
  return 0;
}


