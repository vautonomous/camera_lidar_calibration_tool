#include "translation_solver.h"

double dx_;
double dy_;
double dz_;

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

std::vector<Eigen::Vector4f> vector_plane_coeff_target;
std::vector<Cloud::Ptr> vector_plane_points_source;

TranslationSolver::TranslationSolver(const std::vector<Sample> &samples_)
{
  for (const auto& sample:samples_)
  {
    if (sample.is_feature_extracted)
    {
      Eigen::Vector4f tmpvec;
      tmpvec(0) = sample.plane_coeff_cam(0);
      tmpvec(1) = sample.plane_coeff_cam(1);
      tmpvec(2) = sample.plane_coeff_cam(2);
      tmpvec(3) = sample.plane_coeff_cam(3);
      vector_plane_coeff_target.push_back(tmpvec);
      vector_plane_points_source.push_back(sample.cloud_plane_points_lidar);
    }
  }

}

void
TranslationSolver::init_genes(MySolution& p,const std::function<double(void)> &rnd01)
{
  // rnd01() gives a random number in 0~1
  p.dx=-5+10*rnd01();
  p.dy=-5+10*rnd01();
  p.dz=-5+10*rnd01();
}

bool
TranslationSolver::eval_solution(
    const MySolution& p,
    MyMiddleCost &c)
{
  const double& dx=p.dx;
  const double& dy=p.dy;
  const double& dz=p.dz;

  Eigen::Affine3d tf_optimized;
  tf_optimized.setIdentity();
  tf_optimized.translation() = Eigen::Vector3d {dx, dy, dz};

  double cost = 0;
  std::vector<double> vector_costs;

  for (size_t i=0; i<vector_plane_coeff_target.size(); i++)
  {
    auto cloud_plane_points = vector_plane_points_source[i];
    auto plane_coeff_target = vector_plane_coeff_target[i];
    Cloud::Ptr cloud_plane_points_transformed(new Cloud);

    pcl::transformPointCloud(*cloud_plane_points, *cloud_plane_points_transformed, tf_optimized);
/*
    TransformHandler::transform_point_cloud(cloud_plane_points,
                                            cloud_plane_points_transformed,
                                            tf_predicted);
*/

    double distance_total = 0;
    for(const auto& point : cloud_plane_points_transformed->points)
    {
      distance_total += pcl::pointToPlaneDistance(point, plane_coeff_target);
    }

    cost = cost + distance_total / cloud_plane_points_transformed->points.size();

    auto single_cost = distance_total /cloud_plane_points_transformed->points.size();
    vector_costs.push_back(single_cost);
  }

  double mean_cost = cost/vector_plane_coeff_target.size();

  c.objective1=mean_cost;
  return true; // solution is accepted
}


TranslationSolver::MySolution
TranslationSolver::mutate(
    const MySolution& X_base,
    const std::function<double(void)> &rnd01,
    double shrink_scale)
{
  MySolution X_new;
  const double mu = 0.2*shrink_scale; // mutation radius (adjustable)
  bool in_range;
  do{
    in_range=true;
    X_new=X_base;
    X_new.dx+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.dx>=-5 && X_new.dx<5);
    X_new.dy+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.dy>=-5 && X_new.dy<5);
    X_new.dz+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.dz>=-5 && X_new.dz<5);
  } while(!in_range);
  return X_new;
}


TranslationSolver::MySolution TranslationSolver::crossover(
    const MySolution& X1,
    const MySolution& X2,
    const std::function<double(void)> &rnd01)
{
  MySolution X_new;
  double r;
  r=rnd01();
  X_new.dx=r*X1.dx+(1.0-r)*X2.dx;
  r=rnd01();
  X_new.dy=r*X1.dy+(1.0-r)*X2.dy;
  r=rnd01();
  X_new.dz=r*X1.dz+(1.0-r)*X2.dz;
  return X_new;
}

double
TranslationSolver::calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X)
{
  // finalize the cost
  double final_cost=0.0;
  final_cost+=X.middle_costs.objective1;
  return final_cost;
}


void TranslationSolver::SO_report_generation(
    int generation_number,
    const EA::GenerationType<MySolution,MyMiddleCost> &last_generation,
    const MySolution& best_genes)
{
  std::cout
      <<"Generation ["<<generation_number<<"], "
      <<"Best="<<last_generation.best_total_cost<<", "
      <<"Average="<<last_generation.average_cost<<", "
      <<"Best genes=("<<best_genes.to_string()<<")"<<", "
      <<"Exe_time="<<last_generation.exe_time
      <<std::endl;

  dx_ = best_genes.dx;
  dy_ = best_genes.dy;
  dz_ = best_genes.dz;

}
Eigen::Vector3d TranslationSolver::solve() {
  EA::Chronometer timer;
  timer.tic();

  GA_Type ga_obj;
  ga_obj.problem_mode=EA::GA_MODE::SOGA;
  ga_obj.multi_threading=true;
  ga_obj.verbose=false;
  ga_obj.population=5000;
  ga_obj.generation_max=10000;
  ga_obj.calculate_SO_total_fitness=calculate_SO_total_fitness;
  ga_obj.init_genes=init_genes;
  ga_obj.eval_solution=eval_solution;
  ga_obj.mutate=mutate;
  ga_obj.crossover=crossover;
  ga_obj.SO_report_generation=SO_report_generation;
  ga_obj.crossover_fraction=0.7;
  ga_obj.mutation_rate=0.2;
  ga_obj.best_stall_max=10;
  ga_obj.elite_count=10;
  ga_obj.solve();

  Eigen::Vector3d translation_optimized = {dx_, dy_, dz_};
  std::cout << "Optimized translation dx dy dz:" << dx_ <<
  " " << dy_ << " " << dz_ << std::endl;

  return translation_optimized;
}
