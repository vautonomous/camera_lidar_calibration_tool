#ifndef SRC_EXTRINSIC_CALIBRATOR_INCLUDE_TRANSLATION_SOLVER_H_
#define SRC_EXTRINSIC_CALIBRATOR_INCLUDE_TRANSLATION_SOLVER_H_

#include "openGA.hpp"
#include <Eigen/Dense>
#include <sample.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/sac_model_plane.h>



using std::string;
using std::cout;
using std::endl;

class TranslationSolver
{
 public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  explicit TranslationSolver(const std::vector<Sample>& samples_);

  struct MySolution {
    double dx;
    double dy;
    double dz;

    string to_string() const
    {
      return
          string("{")
              +  "dx:"+std::to_string(dx)
              +", dy:"+std::to_string(dy)
              +", dz:"+std::to_string(dz)
              +"}";
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
  static MySolution crossover(const TranslationSolver::MySolution &X1,
                              const TranslationSolver::MySolution &X2,
                              const std::function<double(void)> &rnd01);

  static double calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X);
  static void SO_report_generation(
      int generation_number,
      const EA::GenerationType<MySolution, MyMiddleCost> &last_generation,
      const MySolution &best_genes);

  Eigen::Vector3d solve();



};

#endif  // SRC_EXTRINSIC_CALIBRATOR_INCLUDE_TRANSLATION_SOLVER_H_
