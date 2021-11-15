#ifndef SRC_EXTRINSIC_CALIBRATOR_INCLUDE_ROTATION_SOLVER_H_
#define SRC_EXTRINSIC_CALIBRATOR_INCLUDE_ROTATION_SOLVER_H_
#include <openGA.hpp>
#include <iostream>
#include "Eigen/Dense"
#include <sample.h>
using std::string;
using std::cout;
using std::endl;

class RotationSolver {
 public:
  explicit RotationSolver(const std::vector<Sample>& samples_);

  struct MySolution {
    double qx;
    double qy;
    double qz;
    double qw;

    string to_string() const {
      return
          string("{")
              + "qx:" + std::to_string(qx)
              + ", qy:" + std::to_string(qy)
              + ", qz:" + std::to_string(qz)
              + ", qw:" + std::to_string(qw)
              + "}";
    }
  };

  struct MyMiddleCost {
    // This is where the results of simulation
    // is stored but not yet finalized.
    double objective1;
  };

  typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type;
  typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_Type;

  static void init_genes(MySolution &p, const std::function<double(void)> &rnd01);
  static bool eval_solution(const MySolution &p, MyMiddleCost &c);
  static MySolution mutate(const MySolution &X_base,
                           const std::function<double(void)> &rnd01,
                           double shrink_scale);
  static MySolution crossover(const RotationSolver::MySolution &X1,
                              const RotationSolver::MySolution &X2,
                              const std::function<double(void)> &rnd01);

  static double calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X);

  static void SO_report_generation(
      int generation_number,
      const EA::GenerationType<MySolution, MyMiddleCost> &last_generation,
      const MySolution &best_genes);

  Eigen::Quaterniond solve();
};

#endif  // SRC_EXTRINSIC_CALIBRATOR_INCLUDE_ROTATION_SOLVER_H_
