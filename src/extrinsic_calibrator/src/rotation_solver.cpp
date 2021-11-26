#include "rotation_solver.h"

double qx;
double qy;
double qz;
double qw;
std::vector<Sample> samples;

RotationSolver::RotationSolver(const std::vector<Sample> &samples_) {
samples = samples_;
}


void
RotationSolver::init_genes(MySolution& p,const std::function<double(void)> &rnd01)
{
  // rnd01() gives a random number in 0~1
  p.qx=-1+2*rnd01();
  p.qy=-1+2*rnd01();
  p.qz=-1+2*rnd01();
  p.qw=-1+2*rnd01();
}

bool
RotationSolver::eval_solution(
    const MySolution& p,
    MyMiddleCost &c)
{
  const double& qx=p.qx;
  const double& qy=p.qy;
  const double& qz=p.qz;
  const double& qw=p.qw;

  Eigen::Quaterniond q_predicted = {qw, qx, qy, qz};
  double cost = 0;
  int counter = 0;
  for (const auto& sample:samples)
  {
    if (sample.is_feature_extracted)
    {
      Eigen::Vector3d normal_transformed = q_predicted.toRotationMatrix()*sample.normal_vector_lid;
      Eigen::Vector3d diff = normal_transformed-sample.normal_vector_cam;
      double abs_x = abs(diff(0));
      double abs_y = abs(diff(1));
      double abs_z = abs(diff(2));
      cost = cost + pow(abs_x,2) + pow(abs_y,2) + pow(abs_z,2);
      counter++;
    }
  }

  c.objective1=cost/counter;
  return true; // solution is accepted
}

RotationSolver::MySolution
RotationSolver::mutate(
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
    X_new.qx+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.qx>=-1 && X_new.qx<1);
    X_new.qy+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.qy>=-1 && X_new.qy<1);
    X_new.qz+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.qz>=-1 && X_new.qz<1);
    X_new.qw+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.qw>=-1 && X_new.qw<1);
  } while(!in_range);
  return X_new;
}

RotationSolver::MySolution RotationSolver::crossover(
    const MySolution& X1,
    const MySolution& X2,
    const std::function<double(void)> &rnd01)
{
  MySolution X_new;
  double r;
  r=rnd01();
  X_new.qx=r*X1.qx+(1.0-r)*X2.qx;
  r=rnd01();
  X_new.qy=r*X1.qy+(1.0-r)*X2.qy;
  r=rnd01();
  X_new.qz=r*X1.qz+(1.0-r)*X2.qz;
  r=rnd01();
  X_new.qw=r*X1.qw+(1.0-r)*X2.qw;
  return X_new;
}

double
RotationSolver::calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X)
{
  // finalize the cost
  double final_cost=0.0;
  final_cost+=X.middle_costs.objective1;
  return final_cost;
}


void RotationSolver::SO_report_generation(
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

  qx = best_genes.qx;
  qy = best_genes.qy;
  qz = best_genes.qz;
  qw = best_genes.qw;
}


Eigen::Quaterniond
RotationSolver::solve()
{
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

  Eigen::Quaterniond q_optimized = {qw, qx, qy, qz};

  std::cout << "qw qx qy qz: " << qw << ", " << qx << ", " << qy << ", " << qz << std::endl;
  std::cout << "Rotation matrix:\n" << q_optimized.toRotationMatrix() << std::endl;

  return q_optimized;
}
