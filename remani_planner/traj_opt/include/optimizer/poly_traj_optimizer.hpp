#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <fstream>
#include <sys/time.h>
#include <ctime>
#include <chrono>
#include <limits>
#include <math.h>
#include "optimizer/lbfgs.hpp"
#include "plan_env/grid_map.h"
#include "traj_utils/plan_container.hpp"
#include "traj_utils/poly_traj_utils.hpp"
#include "path_searching/kino_astar.h"
#include "path_searching/rrt.h"
#include "mm_config/mm_config.hpp"

namespace remani_planner
{

  class ConstrainPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    
    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      points.resize(8, size_set);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {
  public:
    int mobile_base_dof_, manipulator_dof_, traj_dim_;
    double safe_margin_;
    double self_safe_margin_;
    double safe_margin_mani_;
    double hard_collision_margin_;
    double hard_collision_margin_mani_;
    double ground_safe_margin_;
    double ground_safe_dis_;
    
    double map_resolution_;
    bool firs_plot_;

    KinoAstar::Ptr kino_a_star_;
    RrtPlanning::Ptr rrt_plan_;
    
  private:
    std::shared_ptr<MMConfig> mm_config_;
    std::vector<poly_traj::MinSnapOpt<8>> SnapOpt_container_;
    std::vector<int> piece_num_container_;
    std::vector<ConstrainPoints> cps_container_;
    std::vector<int> singul_container_;
    std::vector<Eigen::MatrixXd> iniState_container_;
    std::vector<Eigen::MatrixXd> finState_container_;
    std::vector<double> gearVel_container_;
    int traj_num_;
    double non_singul_v_ = 0.01;
    bool opt_gear_ = true;

    std::shared_ptr<GridMap> grid_map_;
    poly_traj::MinSnapOpt<8> SnapOpt_;
    ConstrainPoints cps_;
    
    int times_cal_coll_;
    double time_cal_matrix_;
    int times_cal_matrix_;
    Eigen::VectorXd time_cal_dyna_;
    Eigen::VectorXi times_cal_dyna_;

    Eigen::VectorXi times_query_esdf_;
    Eigen::VectorXd time_check_self_;
    Eigen::VectorXi times_check_self_;
    Eigen::VectorXd time_coll_;
    Eigen::VectorXi times_coll_;
    Eigen::VectorXd time_coll_other_;
    Eigen::VectorXi times_coll_other_;
    bool obs_viol_, car_fea_viol_, mani_fea_viol_;
    Eigen::Matrix4d T_q_0_;

    int cps_num_prePiece_; // number of distinctive constrain points each piece
    int variable_num_;     // optimization variables
    int piece_num_;        // poly traj piece numbers
    int iter_num_;         // iteration of the solver
    double max_solve_time_;
    double max_solve_time_dynamic_;
    double wei_time_dynamic_;
    double wei_temporal_anchor_dynamic_;
    int lbfgs_memory_;
    int lbfgs_max_iterations_;
    int lbfgs_max_iterations_dynamic_;
    bool dynamic_mode_;
    bool optimization_timed_out_;
    std::chrono::steady_clock::time_point optimize_start_time_;

    std::vector<Eigen::Vector3d> car_pts_buffer_;
    std::vector<Eigen::Vector2d> car_pts_yaw_grad_buffer_;
    std::vector<Eigen::Matrix4d> joint_transform_buffer_;
    std::vector<Eigen::Matrix4d> joint_gradient_buffer_;
    std::vector<Eigen::Matrix4d> transform_gradient_buffer_;
    std::vector<Eigen::Matrix4d> self_gradient_buffer_;
    std::vector<Eigen::Matrix4d> self_chain_gradient_buffer_;
    std::vector<Eigen::Vector3d> self_car_pts_buffer_;
    std::vector<Eigen::Vector3d> recovery_car_pts_buffer_;
    Eigen::VectorXd scratch_grad_;
    double min_recovery_margin_;
    double min_recovery_time_slack_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    enum SAFETY_FAILURE_TYPE
    {
      SAFE,
      COLLISION_FAILURE,
      RECOVERY_MARGIN_FAILURE,
      RECOVERY_TIME_FAILURE,
      FEASIBILITY_FAILURE
    } last_safety_failure_;

    /* optimization parameters */
    double wei_obs_;                         // obstacle weight
    double wei_feas_;                        // feasibility weight
    double wei_time_;                        // time weight
    double wei_temporal_anchor_;             // quadratic time-allocation prior
    double wei_mani_obs_;                    // manipulator obstacle weight
    double wei_mani_self_;                   // manipulator self collision weight
    double wei_mani_feas_;                   // manipulator feasibility weight
    double wei_recoverability_;
    double wei_recovery_time_;
    double stop_deceleration_;
    double control_dt_;
    double recovery_tolerance_;
    double arm_extension_weight_;
    double recovery_horizon_;
    double recovery_time_tolerance_;
    double temporal_repair_factor_;
    int temporal_repair_attempts_;
    double last_safety_failure_time_;
    Eigen::VectorXd recovery_joint_pose_;
    double trajectory_start_time_ = 0.0;
    bool use_dynamic_prediction_ = true;
    std::vector<Eigen::VectorXd> temporal_seed_container_;

    int dense_sample_resolution_;
    
    double max_vel_, max_acc_, max_wheel_omega_, max_wheel_alpha_; // dynamic limits
    double mobile_base_wheel_base_;
    double mobile_base_length_, mobile_base_width_, mobile_base_height_, mobile_base_wheel_radius_;
    double mobile_base_check_radius_;
    Eigen::VectorXd min_joint_pos_, max_joint_pos_;
    double max_joint_vel_, max_joint_acc_;
    double manipulator_thickness_;
    std::vector<Eigen::Matrix4Xd> manipulator_link_pts_;
    Eigen::VectorXd manipulator_config_;

    Eigen::Matrix2d B_h_;
    Eigen::MatrixXd C_h_;

    ros::Publisher traj_pt_pub_, traj_init_pt_pub_;
    ros::Publisher front_end_mm_mesh_vis_pub_, back_end_mm_mesh_vis_pub_;
  public:

    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    /* set variables */
    void setParam(ros::NodeHandle &nh, const std::shared_ptr<GridMap> &map, const std::shared_ptr<MMConfig> &mm_config);
    void clear_resize_Cps_container(int container_size);
    void setControlPoints(const int trajid, const Eigen::MatrixXd &points);

    /* helper functions */
    inline ConstrainPoints getControlPoints() { return cps_; }
    inline const ConstrainPoints *getControlPointsPtr(void) { return &cps_; }
    inline const poly_traj::MinSnapOpt<8> *getMinJerkOptPtr(void) { return &SnapOpt_; }
    inline const std::vector<poly_traj::MinSnapOpt<8>> *getMinSnapOptContainerPtr(void) { return &SnapOpt_container_; }
    inline int get_cps_num_prePiece_(){return cps_num_prePiece_;};
    inline bool frontEndMeshVisible() {return front_end_mm_mesh_vis_pub_.getNumSubscribers() > 0;};
    inline double getMinRecoveryMargin() const {return min_recovery_margin_;};
    inline double getMinRecoveryTimeSlack() const {return min_recovery_time_slack_;};
    bool checkCollision(const SingulTrajData &traj, double t, int &coll_type);
    bool checkRecoverability(const SingulTrajData &traj, double t, double &margin, double &time_slack);

    /* main planning API */
    bool OptimizeTrajectory_lbfgs(const std::vector<Eigen::MatrixXd> &iniStates_container, 
                                  const std::vector<Eigen::MatrixXd> &finState_container,
                                  const std::vector<Eigen::MatrixXd> &innerPts_container, 
                                  const std::vector<Eigen::VectorXd> &initT_container, 
                                  const std::vector<int> &singul_container,
                                  std::vector<Eigen::MatrixXd> &optCps_container, 
                                  std::vector<Eigen::MatrixXd> &optWps_container, 
                                  std::vector<Eigen::VectorXd> &optT_container,
                                  std::vector<Eigen::VectorXd> &optEECps_container,
                                  double trajectory_start_time);
                                            
    int astarWithMinTraj(const Eigen::MatrixXd &iniState,
                          const Eigen::MatrixXd &finState,
                          const double start_yaw,
                          const int _start_singul,
                          const bool start_gripper,
                          const double end_yaw,
                          const bool end_gripper,
                          const Eigen::Vector2d init_ctrl,
                          const int continous_failures_count,
                          std::vector<std::vector<Eigen::VectorXd>> &simple_path_container,  // 前进/倒车为一段, 每段N个点x, y, mani_angle
                          std::vector<vector<double>> &yaw_list_container,
                          std::vector<poly_traj::MinSnapOpt<8>> &frontendMJ_container,
                          std::vector<int> &singul_container);
    void displayFrontEndMesh(std::vector<Eigen::VectorXd> &simple_path_full, vector<double> &yaw_list);
    void displayBackEndMesh(const SingulTrajData &traj_data, bool init, bool gripper_close);
  
  
  private:
    /* callbacks by the L-BFGS optimizer */
    // static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionCallback(void *instance,
                                       const Eigen::VectorXd &x,
                                       Eigen::VectorXd &g);

    static int earlyExitCallback(void *instance, const Eigen::VectorXd &x,
                                    const Eigen::VectorXd &g,
                                    const double fx,
                                    const double step,
                                    const int k,
                                    const int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const int trajid,
                          const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);
    bool smoothedL1(const double &x, const double &mu,double &f,double &df);
    bool smoothedMax3(const double &x, double &f, double &df);
    bool smoothedLog(const double &x, const double &mu, double &l, double &grad);
    double armExtension(const Eigen::VectorXd &pos,
                        double &d_extension_d_joint1,
                        double &d_extension_d_joint2) const;
    double jointRecoveryTime(const Eigen::VectorXd &pos,
                             int &critical_joint,
                             double &d_time_d_joint) const;
    double recoveryTimeSlack(const Eigen::VectorXd &pos,
                             const Eigen::VectorXd &vel) const;

    /* gradient and cost evaluation functions */    
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(const int trajid, EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAJGradCost2CT(const int trajid, EIGENVEC &gdT,
                            Eigen::VectorXd &costs, const int &K,
                            double trajectory_start_time);

    bool obstacleGradCostforMM(const int i_dp,
                                const Eigen::VectorXd &pos,
                                const Eigen::VectorXd &vel,
                                const int trajid,
                                Eigen::VectorXd &gradp,
                                Eigen::VectorXd &gradv,
                                double &costp,
                                double &cost_mani,
                                double &cost_self,
                                double query_time);

    bool feasibilityGradCostCar(const Eigen::Vector2d &vel, const Eigen::Vector2d &acc, const Eigen::Vector2d &jer, const int trajid,
                                Eigen::Vector2d &gradv_2d, Eigen::Vector2d &grada_2d, Eigen::Vector2d &gradj_2d,
                                double &cost_mm_feasible, bool dense_sample);
    
    bool feasibilityGradCostJoint(const Eigen::VectorXd &pos, const Eigen::VectorXd &vel, const Eigen::VectorXd &acc, 
                                  Eigen::VectorXd &gradp, Eigen::VectorXd &gradv, Eigen::VectorXd &grada,
                                  double &cost_joint_feasible);
    bool IsTrajSafe(const SingulTrajData &traj_data, double check_dt = 0.01);
    bool IsNotFeasibie(const SingulTrajData &traj_data, double t);

  public:
    typedef unique_ptr<PolyTrajOptimizer> Ptr;

  };

} // namespace remani_planner
#endif
