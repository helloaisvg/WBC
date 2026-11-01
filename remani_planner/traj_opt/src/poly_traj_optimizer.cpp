#include "optimizer/poly_traj_optimizer.hpp"

namespace remani_planner
{
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh, const std::shared_ptr<GridMap> &map, const std::shared_ptr<MMConfig> &mm_config){
    grid_map_ = map;
    map_resolution_ = grid_map_->getResolution();

    mm_config_ = mm_config;
    max_vel_ = mm_config_->getBaseMaxVel();
    max_acc_ = mm_config_->getBaseMaxAcc();
    manipulator_config_ = mm_config_->getManiConfig();
    T_q_0_ = mm_config_->getTq0();
    manipulator_link_pts_ = mm_config_->getLinkPoint();

    nh.param("optimization/constrain_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_base_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    nh.param("optimization/weight_time_dynamic",
             wei_time_dynamic_, wei_time_);
    nh.param("optimization/weight_temporal_anchor",
             wei_temporal_anchor_, 0.0);
    nh.param("optimization/weight_temporal_anchor_dynamic",
             wei_temporal_anchor_dynamic_, wei_temporal_anchor_);
    wei_mani_obs_ = wei_obs_ / 5.0;
    nh.param("optimization/weight_manipulator_self", wei_mani_self_, -1.0);
    nh.param("optimization/weight_manipulator_feasibility", wei_mani_feas_, -1.0);

    nh.param("optimization/dense_sample_resolution", dense_sample_resolution_, -1);
    nh.param("optimization/max_solve_time", max_solve_time_, 0.30);
    nh.param("optimization/max_solve_time_dynamic",
             max_solve_time_dynamic_, max_solve_time_);
    nh.param("optimization/max_iterations", lbfgs_max_iterations_, 300);
    nh.param("optimization/max_iterations_dynamic",
             lbfgs_max_iterations_dynamic_, lbfgs_max_iterations_);
    nh.param("optimization/lbfgs_memory", lbfgs_memory_, 64);
    nh.param("optimization/weight_recoverability", wei_recoverability_, 20000.0);
    nh.param("optimization/weight_recovery_time", wei_recovery_time_, 5000.0);
    nh.param("optimization/stop_deceleration", stop_deceleration_, 1.4);
    nh.param("optimization/control_dt", control_dt_, 0.01);
    nh.param("optimization/recovery_tolerance", recovery_tolerance_, 0.07);
    nh.param("optimization/arm_extension_weight", arm_extension_weight_, 0.25);
    nh.param("optimization/recovery_horizon", recovery_horizon_, 1.5);
    nh.param("optimization/recovery_time_tolerance", recovery_time_tolerance_, 0.10);
    nh.param("optimization/temporal_repair_factor", temporal_repair_factor_, 0.75);
    nh.param("optimization/temporal_repair_attempts", temporal_repair_attempts_, 3);
    nh.param("optimization/use_dynamic_prediction",
             use_dynamic_prediction_, true);

    nh.param("optimization/safe_margin", safe_margin_, 0.1);
    nh.param("optimization/safe_margin_mani", safe_margin_mani_, 0.1);
    nh.param("optimization/self_safe_margin", self_safe_margin_, 0.1);
    nh.param("optimization/hard_collision_margin", hard_collision_margin_, 0.0);
    nh.param("optimization/hard_collision_margin_mani",
             hard_collision_margin_mani_, 0.0);
    nh.param("optimization/ground_safe_dis", ground_safe_dis_, 0.1);
    nh.param("optimization/ground_safe_margin", ground_safe_margin_, 0.1);
    nh.param("optimization/mobile_base_opt_gear", opt_gear_, true);

    nh.param("mm/mobile_base_dof", mobile_base_dof_, -1);
    nh.param("mm/mobile_base_non_singul_vel", non_singul_v_, -1.0);
    nh.param("mm/mobile_base_max_wheel_omega", max_wheel_omega_, -1.0);
    nh.param("mm/mobile_base_max_wheel_alpha", max_wheel_alpha_, -1.0);
    nh.param("mm/mobile_base_wheel_base", mobile_base_wheel_base_, -1.0);
    nh.param("mm/mobile_base_wheel_radius", mobile_base_wheel_radius_, -1.0);
    nh.param("mm/mobile_base_length", mobile_base_length_, -1.0);
    nh.param("mm/mobile_base_width", mobile_base_width_, -1.0);
    nh.param("mm/mobile_base_height", mobile_base_height_, -1.0);
    nh.param("mm/mobile_base_check_radius", mobile_base_check_radius_, -1.0);

    nh.param("mm/manipulator_dof", manipulator_dof_, -1);
    nh.param("mm/manipulator_thickness", manipulator_thickness_, -1.0);
    
    std::vector<double> joint_pos_limit;
    nh.getParam("mm/manipulator_min_pos", joint_pos_limit);
    min_joint_pos_.resize(joint_pos_limit.size());
    for(unsigned int i = 0; i < joint_pos_limit.size(); i++){
        min_joint_pos_(i) = joint_pos_limit[i];
    }

    joint_pos_limit.clear();
    nh.getParam("mm/manipulator_max_pos", joint_pos_limit);
    max_joint_pos_.resize(joint_pos_limit.size());
    for(unsigned int i = 0; i < joint_pos_limit.size(); i++){
        max_joint_pos_(i) = joint_pos_limit[i];
    }
    std::vector<double> recovery_joint_pose;
    if(nh.getParam("mm/manipulator_recovery_pos", recovery_joint_pose) &&
       (int)recovery_joint_pose.size() == manipulator_dof_){
        recovery_joint_pose_.resize(manipulator_dof_);
        for(int i = 0; i < manipulator_dof_; ++i){
            recovery_joint_pose_(i) = recovery_joint_pose[i];
        }
    }else{
        recovery_joint_pose_ = 0.5 * (min_joint_pos_ + max_joint_pos_);
    }
    nh.param("mm/manipulator_max_vel", max_joint_vel_, -1.0);
    nh.param("mm/manipulator_max_acc", max_joint_acc_, -1.0);

    firs_plot_ = true;

    B_h_ << 0.0, -1.0,
            1.0,  0.0;
    C_h_ = Eigen::MatrixXd::Zero(3, 4);
    C_h_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    kino_a_star_.reset(new KinoAstar);
    kino_a_star_->setParam(nh, map, mm_config);
    // kino_a_star_->init();

    traj_dim_ = mobile_base_dof_ + manipulator_dof_;
    car_pts_buffer_.reserve(128);
    car_pts_yaw_grad_buffer_.reserve(128);
    joint_transform_buffer_.reserve(manipulator_dof_);
    joint_gradient_buffer_.reserve(manipulator_dof_);
    transform_gradient_buffer_.resize(2 + manipulator_dof_);
    self_gradient_buffer_.resize(manipulator_dof_);
    self_chain_gradient_buffer_.reserve(manipulator_dof_);
    self_car_pts_buffer_.reserve(128);
    recovery_car_pts_buffer_.reserve(128);
    scratch_grad_.resize(traj_dim_);
    min_recovery_margin_ = std::numeric_limits<double>::infinity();
    min_recovery_time_slack_ = std::numeric_limits<double>::infinity();
    traj_pt_pub_ = nh.advertise<visualization_msgs::Marker>("traj_pt_vis", 10);
    traj_init_pt_pub_ = nh.advertise<visualization_msgs::Marker>("traj_init_pt_vis", 10);
    front_end_mm_mesh_vis_pub_ =  nh.advertise<visualization_msgs::MarkerArray>("front_end_mm_mesh_vis", 50);
    back_end_mm_mesh_vis_pub_ =  nh.advertise<visualization_msgs::MarkerArray>("back_end_mm_mesh_vis", 50);
  }

  /* main planning API */
  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs(const std::vector<Eigen::MatrixXd> &iniState_container, 
                                                    const std::vector<Eigen::MatrixXd> &finState_container,
                                                    const std::vector<Eigen::MatrixXd> &initInnerPts_container, 
                                                    const std::vector<Eigen::VectorXd> &initT_container, 
                                                    const std::vector<int> &singul_container,
                                                    std::vector<Eigen::MatrixXd> &optCps_container, 
                                                    std::vector<Eigen::MatrixXd> &optWps_container, 
                                                    std::vector<Eigen::VectorXd> &optT_container,
                                                    std::vector<Eigen::VectorXd> &optEECps_container,
                                                    double trajectory_start_time){
    trajectory_start_time_ = trajectory_start_time;
    const double configured_solve_time = max_solve_time_;
    const double configured_time_weight = wei_time_;
    const double configured_temporal_anchor = wei_temporal_anchor_;
    const int configured_iterations = lbfgs_max_iterations_;
    dynamic_mode_ = grid_map_->getDynamicObstacleCount() > 0;
    if(dynamic_mode_){
      max_solve_time_ = max_solve_time_dynamic_;
      wei_time_ = wei_time_dynamic_;
      wei_temporal_anchor_ = wei_temporal_anchor_dynamic_;
      lbfgs_max_iterations_ = lbfgs_max_iterations_dynamic_;
    }
    traj_num_ = initInnerPts_container.size();
    iniState_container_ = iniState_container;
    finState_container_ = finState_container;
    singul_container_ = singul_container;
    variable_num_ = 0;
    SnapOpt_container_.clear();
    piece_num_container_.clear();
    SnapOpt_container_.resize(traj_num_);
    piece_num_container_.resize(traj_num_);
    if (traj_num_ != (int)initT_container.size()){
      ROS_ERROR("traj_num_ != initT_container.size()");
      return false;
    }

    int piece_num;
    int piece_num_all = 0;
    for(int i = 0; i < traj_num_; i++){
      //check
      if(initInnerPts_container[i].cols()==0){
        ROS_ERROR("There is only a piece?");
        return false;
      }
      piece_num = initInnerPts_container[i].cols() + 1;
      piece_num_container_[i] = piece_num;
      piece_num_all += piece_num;

      //reset the start end max_vel_
      if(iniState_container_[i].col(1).head(2).norm() >= max_vel_){
        iniState_container_[i].col(1).head(2) = iniState_container_[i].col(1).head(2).normalized() * (max_vel_ - 1.0e-2);
      }
      if(finState_container_[i].col(1).head(2).norm() >= max_vel_){
        finState_container_[i].col(1).head(2) = finState_container_[i].col(1).head(2).normalized() * (max_vel_ - 1.0e-2);
      }

      if(iniState_container_[i].col(2).head(2).norm() >= max_acc_){
        iniState_container_[i].col(2).head(2) = iniState_container_[i].col(2).head(2).normalized() * (max_acc_ - 1.0e-2);
      }
      if(finState_container_[i].col(2).head(2).norm() >= max_acc_){
        finState_container_[i].col(2).head(2) = finState_container_[i].col(2).head(2).normalized() * (max_acc_ - 1.0e-2);
      }
      
      SnapOpt_container_[i].reset(piece_num);
      variable_num_ += traj_dim_ * (piece_num - 1); // inner pt
      variable_num_ += piece_num; // time allocation
    }

    std::vector<Eigen::VectorXd> seeded_T_container = initT_container;
    double seeded_duration = 0.0;
    for(const auto &times : seeded_T_container){
      seeded_duration += times.sum();
    }
    SingulTrajData seeded_traj_data;
    bool seeded_traj_safe = false;
    if(dynamic_mode_){
      for(int i = 0; i < traj_num_; ++i){
        SnapOpt_container_[i].generate(
            initInnerPts_container[i], seeded_T_container[i],
            iniState_container_[i], finState_container_[i]);
      }

      auto rebuild_seeded_trajectory = [&]() {
        double traj_start_time = trajectory_start_time_;
        seeded_traj_data.clearSingulTraj();
        for(unsigned int i = 0; i < singul_container.size(); ++i){
          seeded_traj_data.addSingulTraj(
              SnapOpt_container_[i].getTraj(singul_container[i]),
              traj_start_time);
          traj_start_time = seeded_traj_data.singul_traj.back().end_time;
        }
      };

      rebuild_seeded_trajectory();
      seeded_traj_safe = IsTrajSafe(seeded_traj_data);
      if(!seeded_traj_safe &&
         (last_safety_failure_ == RECOVERY_MARGIN_FAILURE ||
          last_safety_failure_ == RECOVERY_TIME_FAILURE ||
          last_safety_failure_ == FEASIBILITY_FAILURE)){
        for(int attempt = 0; attempt < temporal_repair_attempts_; ++attempt){
          double local_time = last_safety_failure_time_;
          int traj_idx = 0;
          while(traj_idx < traj_num_ &&
                local_time > seeded_traj_data.singul_traj[traj_idx].duration){
            local_time -= seeded_traj_data.singul_traj[traj_idx].duration;
            traj_idx++;
          }
          if(traj_idx >= traj_num_){
            break;
          }

          int piece_idx =
              SnapOpt_container_[traj_idx]
                  .getTraj(singul_container_[traj_idx])
                  .locatePieceIdx(local_time);
          for(int piece = piece_idx;
              piece < seeded_T_container[traj_idx].size(); ++piece){
            seeded_T_container[traj_idx](piece) /= temporal_repair_factor_;
          }
          SnapOpt_container_[traj_idx].regenerateWithTimes(
              seeded_T_container[traj_idx]);
          rebuild_seeded_trajectory();
          seeded_traj_safe = IsTrajSafe(seeded_traj_data);
          if(seeded_traj_safe ||
             (last_safety_failure_ != RECOVERY_MARGIN_FAILURE &&
              last_safety_failure_ != RECOVERY_TIME_FAILURE &&
              last_safety_failure_ != FEASIBILITY_FAILURE)){
            break;
          }
        }
      }
    }
    temporal_seed_container_ = seeded_T_container;
    
    variable_num_ += traj_dim_ * (traj_num_ - 1); // gear pos
    variable_num_ += 1 * (traj_num_ - 1); // gear angle

    Eigen::VectorXd x;
    x.resize(variable_num_);
    int offset = 0;
    // inner pt
    for(int i = 0; i < traj_num_; i++){
      memcpy(x.data() + offset, initInnerPts_container[i].data(), initInnerPts_container[i].size() * sizeof(x[0]));
      offset += initInnerPts_container[i].size();
    }
    // time allocation
    Eigen::VectorXd initT;
    int offset_temp = 0;
    initT.resize(piece_num_all);
    for(int i = 0; i < traj_num_; i++){
      memcpy(x.data() + offset, seeded_T_container[i].data(),
             seeded_T_container[i].size() * sizeof(x[0]));
      memcpy(initT.data() + offset_temp, seeded_T_container[i].data(),
             seeded_T_container[i].size() * sizeof(initT[0]));
      offset += seeded_T_container[i].size();
      offset_temp += seeded_T_container[i].size();
    }
    Eigen::Map<Eigen::VectorXd> Vt(x.data() + offset - piece_num_all, piece_num_all);
    RealT2VirtualT(initT, Vt);
    // gear pos
    for(int i = 0; i < traj_num_ - 1; i++){
      memcpy(x.data() + offset, finState_container_[i].col(0).data(), traj_dim_ * sizeof(x[0]));
      offset += traj_dim_;
    }
    // gear angle
    Eigen::Map<Eigen::VectorXd> angles(x.data() + offset, traj_num_ - 1);
    for(int i = 0; i < traj_num_ - 1; i++){
      Eigen::Vector2d gearv = finState_container_[i].col(1).head(2);
      angles[i] = std::atan2(gearv[1], gearv[0]);
    }
    
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = lbfgs_memory_;
    lbfgs_params.g_epsilon = 1.0e-4;
    lbfgs_params.delta = 1e-3;
    lbfgs_params.max_iterations = lbfgs_max_iterations_;

    double final_cost;
    
    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;
    optimization_timed_out_ = false;
    optimize_start_time_ = std::chrono::steady_clock::now();

    /* ---------- optimize ---------- */
    // ros::Time t1 = ros::Time::now();
    int result = lbfgs::lbfgs_optimize(
        x,
        final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        lbfgs_params);
    max_solve_time_ = configured_solve_time;
    wei_time_ = configured_time_weight;
    wei_temporal_anchor_ = configured_temporal_anchor;
    lbfgs_max_iterations_ = configured_iterations;

    // printf("\033[32miter = %d, time = %5.3f(ms), \n\033[0m", iter_num_, (ros::Time::now() - t1).toSec() * 1000.0);
    if(result == lbfgs::LBFGS_CANCELED && optimization_timed_out_){
      ROS_INFO("Optimization reached the %.3f s wall-clock budget after %d iterations.",
               max_solve_time_, iter_num_);
    }else if((result != lbfgs::LBFGS_CONVERGENCE)&&(result != lbfgs::LBFGS_STOP)&&(result != lbfgs::LBFGSERR_MAXIMUMLINESEARCH)){
      ROS_ERROR("The optimization result is : %s", lbfgs::lbfgs_strerror(result));      
    }else if(result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
      ROS_WARN("The optimization result is : %s", lbfgs::lbfgs_strerror(result));
    }else{
      ROS_INFO("The optimization result is : %s", lbfgs::lbfgs_strerror(result));      
    }

    SingulTrajData singul_traj_data;
    auto rebuild_safety_trajectory = [&]() {
      double traj_start_time = trajectory_start_time_;
      singul_traj_data.clearSingulTraj();
      for(unsigned int i = 0; i < singul_container.size(); ++i){
        singul_traj_data.addSingulTraj(SnapOpt_container_[i].getTraj(singul_container[i]), traj_start_time);
        traj_start_time = singul_traj_data.singul_traj.back().end_time;
      }
    };

    rebuild_safety_trajectory();
    bool good_traj = IsTrajSafe(singul_traj_data);
    if(!good_traj &&
       (last_safety_failure_ == RECOVERY_MARGIN_FAILURE ||
        last_safety_failure_ == RECOVERY_TIME_FAILURE ||
        last_safety_failure_ == FEASIBILITY_FAILURE)){
      int repair_count = 0;
      for(int attempt = 0; attempt < temporal_repair_attempts_; ++attempt){
        double local_time = last_safety_failure_time_;
        int traj_idx = 0;
        while(traj_idx < traj_num_ &&
              local_time > singul_traj_data.singul_traj[traj_idx].duration){
          local_time -= singul_traj_data.singul_traj[traj_idx].duration;
          traj_idx++;
        }
        if(traj_idx >= traj_num_){
          break;
        }

        int piece_idx =
            SnapOpt_container_[traj_idx]
                .getTraj(singul_container_[traj_idx])
                .locatePieceIdx(local_time);
        Eigen::VectorXd repaired_times =
            SnapOpt_container_[traj_idx].get_T1();
        for(int piece = piece_idx; piece < repaired_times.size(); ++piece){
          repaired_times(piece) /= temporal_repair_factor_;
        }
        SnapOpt_container_[traj_idx].regenerateWithTimes(repaired_times);
        repair_count++;
        rebuild_safety_trajectory();
        good_traj = IsTrajSafe(singul_traj_data);
        if(good_traj){
          ROS_INFO("Temporal safety repair localized %d polynomial time allocations; duration %.3f s.",
                   repair_count, singul_traj_data.duration);
          break;
        }
        if(last_safety_failure_ != RECOVERY_MARGIN_FAILURE &&
           last_safety_failure_ != RECOVERY_TIME_FAILURE &&
           last_safety_failure_ != FEASIBILITY_FAILURE){
          break;
        }
      }
    }
    if(!good_traj && seeded_traj_safe){
      for(int i = 0; i < traj_num_; ++i){
        SnapOpt_container_[i].generate(
            initInnerPts_container[i], seeded_T_container[i],
            iniState_container_[i], finState_container_[i]);
        cps_container_[i].points = initInnerPts_container[i];
      }
      rebuild_safety_trajectory();
      good_traj = IsTrajSafe(singul_traj_data);
      if(good_traj){
        ROS_INFO("Selected safety-seeded temporal candidate; duration %.3f s.",
                 singul_traj_data.duration);
      }
    }
    if(result == lbfgs::LBFGSERR_INVALID_FUNCVAL && !good_traj){
      return false;
    }

    double final_duration = 0.0;
    for(int i = 0; i < traj_num_; ++i){
      final_duration += SnapOpt_container_[i].get_T1().sum();
    }
    if(dynamic_mode_ && good_traj && seeded_duration > 0.0 &&
       final_duration > 1.2 * seeded_duration){
      std::vector<Eigen::VectorXd> accepted_times(traj_num_);
      std::vector<Eigen::VectorXd> original_times(traj_num_);
      for(int traj = 0; traj < traj_num_; ++traj){
        accepted_times[traj] = SnapOpt_container_[traj].get_T1();
        original_times[traj] = accepted_times[traj];
      }

      for(int pass = 0; pass < 3; ++pass){
        bool changed = false;
        for(int traj = 0; traj < traj_num_; ++traj){
          for(int piece = 0; piece < accepted_times[traj].size(); ++piece){
            Eigen::VectorXd trial_times = accepted_times[traj];
            trial_times(piece) = std::min(
                trial_times(piece), seeded_T_container[traj](piece));
            if(accepted_times[traj](piece) - trial_times(piece) < 0.05){
              continue;
            }

            SnapOpt_container_[traj].regenerateWithTimes(trial_times);
            rebuild_safety_trajectory();
            if(IsTrajSafe(singul_traj_data, 0.05)){
              accepted_times[traj] = trial_times;
              changed = true;
            }else{
              double safe_piece_time = accepted_times[traj](piece);
              double fast_piece_time = trial_times(piece);
              for(int refinement = 0; refinement < 3; ++refinement){
                const double candidate_piece_time =
                    0.5 * (safe_piece_time + fast_piece_time);
                trial_times = accepted_times[traj];
                trial_times(piece) = candidate_piece_time;
                SnapOpt_container_[traj].regenerateWithTimes(trial_times);
                rebuild_safety_trajectory();
                if(IsTrajSafe(singul_traj_data, 0.05)){
                  accepted_times[traj] = trial_times;
                  safe_piece_time = candidate_piece_time;
                  changed = true;
                }else{
                  fast_piece_time = candidate_piece_time;
                }
              }
            }
          }
        }
        if(!changed){
          break;
        }
      }

      double interpolation = 1.0;
      double fast_interpolation = 0.0;
      std::vector<Eigen::VectorXd> best_times = accepted_times;
      for(int refinement = 0; refinement < 4; ++refinement){
        const double candidate_interpolation =
            0.5 * (interpolation + fast_interpolation);
        for(int traj = 0; traj < traj_num_; ++traj){
          Eigen::VectorXd trial_times =
              seeded_T_container[traj] +
              candidate_interpolation *
                  (accepted_times[traj] - seeded_T_container[traj]);
          for(int piece = 0; piece < trial_times.size(); ++piece){
            trial_times(piece) =
                std::min(trial_times(piece), accepted_times[traj](piece));
          }
          SnapOpt_container_[traj].regenerateWithTimes(trial_times);
        }
        rebuild_safety_trajectory();
        if(IsTrajSafe(singul_traj_data, 0.05)){
          interpolation = candidate_interpolation;
          best_times = accepted_times;
          for(int traj = 0; traj < traj_num_; ++traj){
            best_times[traj] =
                seeded_T_container[traj] +
                interpolation * (accepted_times[traj] -
                                 seeded_T_container[traj]);
            for(int piece = 0; piece < best_times[traj].size(); ++piece){
              best_times[traj](piece) =
                  std::min(best_times[traj](piece),
                           accepted_times[traj](piece));
            }
          }
        }else{
          fast_interpolation = candidate_interpolation;
        }
      }
      accepted_times = best_times;

      for(int traj = 0; traj < traj_num_; ++traj){
        SnapOpt_container_[traj].regenerateWithTimes(accepted_times[traj]);
      }
      rebuild_safety_trajectory();
      good_traj = IsTrajSafe(singul_traj_data);
      if(good_traj){
        final_duration = 0.0;
        for(int traj = 0; traj < traj_num_; ++traj){
          final_duration += accepted_times[traj].sum();
        }
        ROS_INFO("Piecewise temporal compression reduced duration to %.3f s.",
                 final_duration);
      }else{
        for(int traj = 0; traj < traj_num_; ++traj){
          SnapOpt_container_[traj].regenerateWithTimes(original_times[traj]);
        }
        rebuild_safety_trajectory();
        good_traj = IsTrajSafe(singul_traj_data);
        final_duration = 0.0;
        for(int traj = 0; traj < traj_num_; ++traj){
          final_duration += original_times[traj].sum();
        }
      }
    }
    ROS_INFO("Optimizer time allocation: seed %.3f s, optimized %.3f s.",
             seeded_duration, final_duration);

    optCps_container.clear();
    optWps_container.clear();
    optT_container.clear();
    optEECps_container.clear();
    for(int i = 0; i < traj_num_; i++){
      optCps_container.push_back(cps_container_[i].points);
      optWps_container.push_back(SnapOpt_container_[i].getInitConstrainPoints(1));
      optT_container.push_back(SnapOpt_container_[i].get_T1());
    }

    return good_traj;
  }

  void PolyTrajOptimizer::displayBackEndMesh(const SingulTrajData &traj_data, bool init, bool gripper_close){
    if(back_end_mm_mesh_vis_pub_.getNumSubscribers() == 0){
      return;
    }

    Eigen::Vector3d car_state;
    Eigen::VectorXd joint_state;
    joint_state.resize(manipulator_dof_);

    visualization_msgs::MarkerArray marker_array, marker_array_all, marker_array_i;
    visualization_msgs::Marker marker_delete_all;
    marker_delete_all.action = visualization_msgs::Marker::DELETEALL;
    marker_array_all.markers.push_back(marker_delete_all);

    Eigen::VectorXd pos, vel;
    int i = 0;
    double yaw;
    double t_step = 0.5 / max_vel_;
    double duration = traj_data.duration;
    for(double t = 0; t < duration - 1e-3; t += t_step, ++i){
      pos = traj_data.getPos(t);
      vel = traj_data.getVel(t);
      yaw = traj_data.getCarAngle(t);

      car_state.head(2) = pos.head(2);
      car_state(2) = yaw;
      joint_state = pos.tail(manipulator_dof_);
      mm_config_->getMMMarkerArray(marker_array, "vis_mm_back_end", i, 0.17, car_state, joint_state, gripper_close);
      marker_array_all.markers.insert(marker_array_all.markers.end(), marker_array.markers.begin(), marker_array.markers.end());
    }
    back_end_mm_mesh_vis_pub_.publish(marker_array_all);
  }

  bool PolyTrajOptimizer::smoothedL1(const double &x, const double &mu, double &f, double &df){
    if (x < 0.0)
    {
      f=0;
      df=0;
      return false;
    }
    else if (x > mu)
    {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    }
    else
    {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);

      return true;
    }
  }

  bool PolyTrajOptimizer::smoothedMax3(const double &x, double &f, double &df){
    if (x < 0.0)
    {
      f=0;
      df=0;
      return false;
    }else{
      f = x * x * x;
      df = 3.0 * x * x;
      return true;
    }
  }

  bool PolyTrajOptimizer::smoothedLog(const double &x, const double &mu, double &l, double &grad){
    if (x <= -mu){
      l = 0.0;
      grad = 0.0;
      return false;
    }else if (x <= 0){
      double e1 = x + mu;
      double e1_2 = e1 * e1;
      double mu_4 = 1.0 / (mu * mu * mu * mu);
      l = 0.5 * e1_2 * e1 * (mu - x) * mu_4;
      grad = e1_2 * (mu - 2 * x) * mu_4;
      return true;
    }else if(x <= mu){
      double e2 = x - mu;
      double e2_2 = e2 * e2;
      double mu_4 = 1.0 / (mu * mu * mu * mu);
      l = 0.5 * (x + mu) * e2_2 * e2 * mu_4 + 1.0;
      grad = e2_2 * (mu + 2 * x) * mu_4;
      return true;
    }else{
      l = 1.0;
      grad = 0.0;
      return true;
    }
  }

  double PolyTrajOptimizer::armExtension(const Eigen::VectorXd &pos,
                                         double &d_extension_d_joint1,
                                         double &d_extension_d_joint2) const {
    d_extension_d_joint1 = 0.0;
    d_extension_d_joint2 = 0.0;
    if(manipulator_dof_ < 2 || manipulator_config_.size() < 2){
      return 0.0;
    }

    const double l1 = std::fabs(manipulator_config_(0));
    const double l2 = std::fabs(manipulator_config_(1));
    const double reach_range = l1 + l2 - std::fabs(l1 - l2);
    if(reach_range < 1.0e-6){
      return 0.0;
    }

    const double q1 = pos(mobile_base_dof_);
    const double q2 = pos(mobile_base_dof_ + 1);
    const double reach = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
    const double raw_extension = (reach - std::fabs(l1 - l2)) / reach_range;
    if(raw_extension <= 0.0 || raw_extension >= 1.0){
      return std::max(0.0, std::min(1.0, raw_extension));
    }

    d_extension_d_joint1 =
        (-l1 * std::sin(q1) - l2 * std::sin(q1 + q2)) / reach_range;
    d_extension_d_joint2 = -l2 * std::sin(q1 + q2) / reach_range;
    return raw_extension;
  }

  double PolyTrajOptimizer::jointRecoveryTime(const Eigen::VectorXd &pos,
                                              int &critical_joint,
                                              double &d_time_d_joint) const {
    critical_joint = -1;
    d_time_d_joint = 0.0;
    double recovery_time = 0.0;
    for(int i = 0; i < manipulator_dof_; ++i){
      const int joint_idx = mobile_base_dof_ + i;
      const double delta_pos = pos(joint_idx) - recovery_joint_pose_(i);
      const double delta_abs = std::fabs(delta_pos);
      if(delta_abs < 1.0e-6){
        continue;
      }

      double joint_time;
      double joint_gradient;
      const double direction = delta_pos > 0.0 ? 1.0 : -1.0;
      const double vel_limit = std::max(1.0e-4, max_joint_vel_);
      const double acc_limit = std::max(1.0e-4, max_joint_acc_);
      const double accel_distance = vel_limit * vel_limit / acc_limit;
      if(delta_abs <= accel_distance){
        joint_time = 2.0 * std::sqrt(delta_abs / acc_limit);
        joint_gradient = direction / std::sqrt(acc_limit * delta_abs);
      }else{
        joint_time = delta_abs / vel_limit + vel_limit / acc_limit;
        joint_gradient = direction / vel_limit;
      }

      if(joint_time > recovery_time){
        recovery_time = joint_time;
        critical_joint = i;
        d_time_d_joint = joint_gradient;
      }
    }
    return recovery_time;
  }

  double PolyTrajOptimizer::recoveryTimeSlack(const Eigen::VectorXd &pos,
                                              const Eigen::VectorXd &vel) const {
    int critical_joint;
    double d_time_d_joint;
    const double arm_time = jointRecoveryTime(pos, critical_joint, d_time_d_joint);
    const double speed = vel.head(2).norm();
    const double stop_time = speed / std::max(1.0e-4, stop_deceleration_) + control_dt_;
    return recovery_horizon_ - stop_time - arm_time;
  }

  bool PolyTrajOptimizer::IsNotFeasibie(const SingulTrajData &traj_data, double t){
    Eigen::VectorXd vel, acc, jer;
    double pen;
    double feas_tol_percent_ = 0.05;

    vel = traj_data.getVel(t).head(2);
    acc = traj_data.getAcc(t).head(2);
    jer = traj_data.getJer(t).head(2);
    int singul = traj_data.getSingul(t);

    double aTv = acc.transpose() * vel;
    double aTBv = acc.transpose() * B_h_ * vel;
    double jTBv = jer.transpose() * B_h_ * vel;
    double v_norm = vel.norm();
    double vTv_inv = 1.0 / vel.squaredNorm(); // avoid siguality vel = 0
    double vTv_inv2 = vTv_inv * vTv_inv;

    // Check minimum speed
    // pen =  non_singul_v_ *  non_singul_v_ * (1.0 - feas_tol_percent_) * (1.0 - feas_tol_percent_) - vel.squaredNorm();
    // if(pen > 0.0){
    //   ROS_WARN("%f min vel is not feasible at relative time %f! opt failed", vel.norm(), t);
    //   return true;
    // }

    double omega = aTBv * vTv_inv;
    // Check maximum rotation speed of the left wheel
    double wheel_omega_left = (2.0 * singul * v_norm - mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_omega_left * wheel_omega_left - max_wheel_omega_ * max_wheel_omega_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      ROS_WARN("%f max left wheel omega is not feasible at relative time %f! opt failed", wheel_omega_left, t );
      return true;
    }

    // Check maximum rotation speed of the right wheel
    double wheel_omega_right = (2.0 * singul * v_norm + mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_omega_right * wheel_omega_right - max_wheel_omega_ * max_wheel_omega_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      ROS_WARN("%f max right wheel omega is not feasible at relative time %f! opt failed", wheel_omega_right, t );
      return true;
    }

    // Check maximum rotational acceleration of the left wheel
    double alpha = jTBv * vTv_inv - 2.0 * aTBv * aTv * vTv_inv2;
    double wheel_alpha_left = (2.0 * singul * aTv / v_norm - mobile_base_wheel_base_ * alpha) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_alpha_left * wheel_alpha_left - max_wheel_alpha_ * max_wheel_alpha_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      ROS_WARN("%f max left wheel alpha is not feasible at relative time %f! opt failed", wheel_alpha_left, t );
      return true;
    }

    // Check maximum acceleration of the right wheel
    double wheel_alpha_right = (2.0 * singul * aTv / v_norm + mobile_base_wheel_base_ * alpha) / (2 * mobile_base_wheel_radius_);
    pen =  wheel_alpha_right * wheel_alpha_right - max_wheel_alpha_ * max_wheel_alpha_ * (1.0 + feas_tol_percent_) * (1.0 + feas_tol_percent_);
    if(pen > 0.0){
      ROS_WARN("%f max right wheel alpha is not feasible at relative time %f! opt failed", wheel_alpha_right, t );
      return true;
    }

    // Manipulator
    vel = traj_data.getVel(t).tail(manipulator_dof_);
    acc = traj_data.getAcc(t).tail(manipulator_dof_);
    if(vel.lpNorm<Eigen::Infinity>() - max_joint_vel_ * (1.0 + feas_tol_percent_) > 0){
      ROS_WARN("mani vel %f is not feasible at relative time %f! opt failed", vel.lpNorm<Eigen::Infinity>(), t);
      return true;
    }

    if(acc.lpNorm<Eigen::Infinity>() - max_joint_acc_ * (1.0 + feas_tol_percent_) > 0){
      ROS_WARN("mani acc %f is not feasible at relative time %f! opt failed", acc.lpNorm<Eigen::Infinity>(), t);
      return true;
    }

    return false;
  }

  bool PolyTrajOptimizer::IsTrajSafe(const SingulTrajData &traj_data,
                                     double check_dt){
    double T_all = traj_data.duration;
    int i_end = floor(T_all / check_dt); // check all
    double t = std::min(0.05, check_dt);  //skip the start point
    // double t = 0.00;  //do not skip the start point
    int coll_type;
    min_recovery_margin_ = std::numeric_limits<double>::infinity();
    min_recovery_time_slack_ = std::numeric_limits<double>::infinity();
    last_safety_failure_ = SAFE;
    last_safety_failure_time_ = -1.0;
    for (int i = 0; i < i_end; ++i){
      if(checkCollision(traj_data, t, coll_type)){
        if(coll_type == 0){
          ROS_WARN("car collision at time %f!", t);
        }else if (coll_type == 1){
          ROS_WARN("mani collision at time %f!", t);
        }else if (coll_type == 2){
          ROS_WARN("car-mani collision at time %f!", t);
        }else if (coll_type == 3){
          ROS_WARN("mani-mani collision at time %f!", t);
        }
        // ROS_WARN("mm (%d) collision at time %f! opt failed", coll_type, t);
        last_safety_failure_ = COLLISION_FAILURE;
        last_safety_failure_time_ = t;
        return false;
      }

      double recovery_margin = 0.0;
      double recovery_time_slack = 0.0;
      const bool recovery_ok = checkRecoverability(traj_data, t, recovery_margin, recovery_time_slack);
      min_recovery_margin_ = std::min(min_recovery_margin_, recovery_margin);
      min_recovery_time_slack_ = std::min(min_recovery_time_slack_, recovery_time_slack);
      if(!recovery_ok){
        if(recovery_margin < -recovery_tolerance_){
          ROS_WARN("Recoverability envelope violated at time %f: margin %.4f m", t, recovery_margin);
          last_safety_failure_ = RECOVERY_MARGIN_FAILURE;
        }else{
          ROS_WARN("Whole-body recovery time exceeded at time %f: slack %.4f s", t, recovery_time_slack);
          last_safety_failure_ = RECOVERY_TIME_FAILURE;
        }
        last_safety_failure_time_ = t;
        return false;
      }

      if(IsNotFeasibie(traj_data, t)){
        last_safety_failure_ = FEASIBILITY_FAILURE;
        last_safety_failure_time_ = t;
        return false;
      }
      
      t += check_dt;
    }
    return true;
  }

  bool PolyTrajOptimizer::checkCollision(const SingulTrajData &traj, double t, int &coll_type)
  {
    Eigen::VectorXd pos = traj.getPos(t);
    Eigen::VectorXd vel = traj.getVel(t);
    double yaw = traj.getCarAngle(t);
    const double world_time = traj.start_time + t;
    return mm_config_->checkcollision(Eigen::Vector3d(pos(0), pos(1), yaw),
                                      pos.tail(manipulator_dof_), false,
                                      coll_type, world_time,
                                      hard_collision_margin_,
                                      hard_collision_margin_mani_);
  }

  bool PolyTrajOptimizer::checkRecoverability(const SingulTrajData &traj, double t,
                                              double &margin, double &time_slack)
  {
    const Eigen::VectorXd pos = traj.getPos(t);
    const Eigen::VectorXd vel = traj.getVel(t);
    const Eigen::Vector3d car_state(pos(0), pos(1), traj.getCarAngle(t));
    mm_config_->getCarPts(car_state, recovery_car_pts_buffer_);
    double min_clearance = std::numeric_limits<double>::infinity();
    const double world_time = traj.start_time + t;
    for(const auto &point : recovery_car_pts_buffer_){
      min_clearance = std::min(
          min_clearance, grid_map_->getDistanceAtTime(point, world_time));
    }

    double unused_d_extension_1 = 0.0;
    double unused_d_extension_2 = 0.0;
    const double extension = armExtension(pos, unused_d_extension_1, unused_d_extension_2);
    const double speed = vel.head(2).norm();
    const double stop_distance =
        (0.5 * speed * speed / stop_deceleration_ + speed * control_dt_) *
        (1.0 + arm_extension_weight_ * extension);
    margin = min_clearance - mobile_base_check_radius_ - safe_margin_ - stop_distance;
    time_slack = recoveryTimeSlack(pos, vel);
    return margin >= -recovery_tolerance_ &&
           time_slack >= -recovery_time_tolerance_;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *instance,
                                       const Eigen::VectorXd &x,
                                       Eigen::VectorXd &g)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(instance);
    double total_smoo_cost = 0, total_time_cost = 0;
    Eigen::VectorXd obs_feas_costs(6), total_obs_feas_costs(6); // car-obs mani-obs mm-mm car_feasible mani_feasible quratic_variance
    obs_feas_costs.setZero();
    total_obs_feas_costs.setZero();
    int offset = 0;
    
    // inner point
    std::vector<Eigen::Map<const Eigen::MatrixXd>> P_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradP_container;
    double segment_start_time = opt->trajectory_start_time_;
    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, opt->traj_dim_, opt->piece_num_container_[trajid] - 1);
      Eigen::Map<Eigen::MatrixXd>gradP(g.data() + offset, opt->traj_dim_, opt->piece_num_container_[trajid] - 1);
      offset += opt->traj_dim_ * (opt->piece_num_container_[trajid] - 1);
      gradP.setZero();
      P_container.push_back(P);
      gradP_container.push_back(gradP);
    }

    std::vector<Eigen::VectorXd> T_container; // real time
    std::vector<Eigen::VectorXd> gradT_container;
    std::vector<Eigen::Map<const Eigen::VectorXd>> t_container; // virtual time
    std::vector<Eigen::Map<Eigen::VectorXd>> gradt_container;
    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      int piece_num = opt->piece_num_container_[trajid];
      Eigen::Map<const Eigen::VectorXd> t(x.data() + offset, piece_num);
      Eigen::Map<Eigen::VectorXd> gradt(g.data() + offset, piece_num); 
      gradt.setZero();
      t_container.push_back(t);
      gradt_container.push_back(gradt);
      
      Eigen::VectorXd T(piece_num);
      opt->VirtualT2RealT(t, T);
      Eigen::VectorXd gradT(piece_num); 
      gradT.setZero();
      T_container.push_back(T);
      gradT_container.push_back(gradT);

      offset += piece_num;
    }

    // Ini/Fin Gear Pos
    std::vector<Eigen::Map<const Eigen::MatrixXd>> Gear_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradGear_container;
    for(int trajid = 0; trajid < opt->traj_num_ - 1; trajid++){
      Eigen::Map<const Eigen::MatrixXd> Gear(x.data() + offset, opt->traj_dim_, 1);
      Eigen::Map<Eigen::MatrixXd>gradGear(g.data() + offset, opt->traj_dim_, 1);
      offset += opt->traj_dim_;
      gradGear.setZero();
      // std::cout << "Gear: " << Gear.transpose() << "\n";
      Gear_container.push_back(Gear);
      gradGear_container.push_back(gradGear);
    }

    // gear angle
    Eigen::Map<const Eigen::VectorXd> Angles(x.data() + offset, opt->traj_num_ - 1);
    Eigen::Map<Eigen::VectorXd>gradAngles(g.data() + offset, opt->traj_num_ - 1);
    gradAngles.setZero();

    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      Eigen::MatrixXd IniS, FinS;
      IniS = opt->iniState_container_[trajid];
      FinS = opt->finState_container_[trajid];

      if(trajid > 0){
        double theta = Angles[trajid - 1];
        IniS.col(0) = Gear_container[trajid - 1];
        IniS.col(1).head(2) = Eigen::Vector2d(-opt->non_singul_v_ * cos(theta), -opt->non_singul_v_ * sin(theta)); 
      }
      if(trajid < opt->traj_num_ - 1){
        double theta = Angles[trajid];
        FinS.col(0) = Gear_container[trajid];
        FinS.col(1).head(2) = Eigen::Vector2d(opt->non_singul_v_ * cos(theta), opt->non_singul_v_ * sin(theta));
      }
      
      opt->SnapOpt_container_[trajid].generate(P_container[trajid], T_container[trajid], IniS, FinS);
      double smoo_cost = 0;
      opt->initAndGetSmoothnessGradCost2PT(trajid, gradT_container[trajid], smoo_cost); // Smoothness cost
      obs_feas_costs.setZero();
      opt->addPVAJGradCost2CT(trajid, gradT_container[trajid],
                              obs_feas_costs,
                              opt->cps_num_prePiece_,
                              segment_start_time); // Time int cost
      segment_start_time += T_container[trajid].sum();
      
      //Get gradT gradC
      total_smoo_cost += smoo_cost;
      total_obs_feas_costs += obs_feas_costs;
    }

    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      double time_cost = 0.0;
      Eigen::MatrixXd gradIni, gradFin;
      //waypoint
      opt->SnapOpt_container_[trajid].getGrad2TP(gradT_container[trajid], gradP_container[trajid], gradIni, gradFin);
      if(opt->opt_gear_){
        if(trajid > 0){
          double theta = Angles[trajid - 1];
          gradGear_container[trajid - 1] += gradIni.col(0);
          gradAngles[trajid - 1] += gradIni.topRows(2).col(1).transpose() * Eigen::Vector2d(opt->non_singul_v_ * sin(theta), -opt->non_singul_v_*cos(theta));
        }
        if(trajid < opt->traj_num_ - 1){
          double theta = Angles[trajid];
          gradGear_container[trajid] += gradFin.col(0);
          gradAngles[trajid] += gradFin.topRows(2).col(1).transpose() * Eigen::Vector2d(-opt->non_singul_v_ * sin(theta), opt->non_singul_v_*cos(theta));
        }
      }
      opt->VirtualTGradCost(trajid, T_container[trajid],
                            t_container[trajid], gradT_container[trajid],
                            gradt_container[trajid], time_cost);
      total_time_cost += time_cost;
    }
    double costall = total_smoo_cost + total_obs_feas_costs.sum() + total_time_cost;
    opt->iter_num_ += 1;

    return costall;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data,const Eigen::VectorXd &x,
                                    const Eigen::VectorXd &g,
                                    const double fx,
                                    const double step,
                                    const int k,
                                    const int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    if (opt->max_solve_time_ > 0.0) {
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - opt->optimize_start_time_).count();
      if (elapsed >= opt->max_solve_time_) {
        opt->optimization_timed_out_ = true;
        return 1;
      }
    }

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const int trajid, const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    double anchor_cost = 0.0;
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      const double anchor_error =
          RT(i) - temporal_seed_container_[trajid](i);
      anchor_cost += 0.5 * wei_temporal_anchor_ *
                     anchor_error * anchor_error;
      gdVT(i) = (gdRT(i) + wei_time_ +
                 wei_temporal_anchor_ * anchor_error) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_ + anchor_cost;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(const int trajid, EIGENVEC &gdT, double &cost)
  {
    SnapOpt_container_[trajid].initGradCost(gdT, cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAJGradCost2CT(
      const int trajid, EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K,
      double trajectory_start_time){
    costs.setZero();
    int N = gdT.size();
    Eigen::VectorXd pos(traj_dim_), pos_next(traj_dim_), vel(traj_dim_), acc(traj_dim_), jer(traj_dim_), snap(traj_dim_);
    Eigen::VectorXd dense_pos(traj_dim_), dense_vel(traj_dim_), dense_acc(traj_dim_), dense_jer(traj_dim_), dense_sna(traj_dim_);
    Eigen::VectorXd gradp(traj_dim_), gradv(traj_dim_), grada(traj_dim_), gradj(traj_dim_), gradp_traj_piece(traj_dim_);
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4; 
    Eigen::Matrix<double, 8, 1> Densebeta0, Densebeta1, Densebeta2, Densebeta3, Densebeta4;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha;
    Eigen::MatrixXd gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    Eigen::MatrixXd gradViolaVc_dense, gradViolaAc_dense, gradViolaJc_dense;
    gradViolaPc.resize(8, traj_dim_);
    gradViolaVc.resize(8, traj_dim_);
    gradViolaAc.resize(8, traj_dim_);
    gradViolaJc.resize(8, traj_dim_);
    gradViolaVc_dense.resize(8, traj_dim_);
    gradViolaAc_dense.resize(8, traj_dim_);
    gradViolaJc_dense.resize(8, traj_dim_);
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt, gradViolaVt_dense, gradViolaAt_dense, gradViolaJt_dense;

    double omg, dense_omg;
    int i_dp = 0;
    double current_piece_start = trajectory_start_time;

    for (int i = 0; i < N; ++i){
      
      Eigen::MatrixXd c;
      c.resize(8, traj_dim_);
      c = SnapOpt_container_[trajid].get_b().block(i * 8, 0, 8, traj_dim_);
      
      step = SnapOpt_container_[trajid].get_T1()(i) / K;
      const double piece_start_time = current_piece_start;
      current_piece_start += SnapOpt_container_[trajid].get_T1()(i);
      s1 = 0.0;
      gradp_traj_piece.setZero();

      
      for (int j = 0; j <= K; ++j){
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s3 * s3;
        s7 = s6 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5, beta0(6) = s6, beta0(7) = s7;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4, beta1(6) = 6.0 * s5, beta1(7) = 7.0 * s6;
        beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3, beta2(6) = 30.0 * s4, beta2(7) = 42.0 * s5;
        beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2, beta3(6) = 120.0 * s3, beta3(7) = 210.0 * s4;
        beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1, beta4(6) = 360.0 * s2, beta4(7) = 840.0 * s3;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        snap = c.transpose() * beta4;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;
        cps_container_[trajid].points.col(i_dp) = pos;
        // collision
        gradp.setZero();
        gradv.setZero();
        double cost_car = 0;
        double cost_mani = 0;
        double cost_self = 0;
        const double query_time =
            use_dynamic_prediction_ ? piece_start_time + j * step
                                    : ros::Time::now().toSec();
        
        if(obstacleGradCostforMM(i_dp, pos, vel, trajid, gradp, gradv,
                                 cost_car, cost_mani, cost_self,
                                 query_time))
        {
          gradp_traj_piece += gradp;
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaPc;
          gdT(i) += omg * ((cost_car + cost_mani + cost_self) / K + step * gradViolaPt);

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaVc;
          gdT(i) += omg * (step * gradViolaVt);

          costs(0) += omg * step * cost_car;
          costs(1) += omg * step * cost_mani;
          costs(2) += omg * step * cost_self;
        }
        
        // car feasibility
        Eigen::Vector2d gradv_2d, grada_2d, gradj_2d;
        gradv_2d.setZero(); grada_2d.setZero(); gradj_2d.setZero();
        double cost_mm_feasible = 0.0;
        if(feasibilityGradCostCar(vel.head(2), acc.head(2), jer.head(2), trajid,
                                  gradv_2d, grada_2d, gradj_2d,
                                  cost_mm_feasible, false)){
          gradv.setZero(); grada.setZero(); gradj.setZero();
          gradv.head(2) = gradv_2d; grada.head(2) = grada_2d; gradj.head(2) = gradj_2d;

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaVc;
          gdT(i) += omg * (cost_mm_feasible / K + step * gradViolaVt);

          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaAc;
          gdT(i) += omg * (step * gradViolaAt);

          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * snap;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaJc;
          gdT(i) += omg * (step * gradViolaJt);

          costs(3) += omg * step * cost_mm_feasible;
        }
        
        // dense sampling
        double min_dense_vel = 0.1; 
        double pen_min_vel = min_dense_vel * min_dense_vel - vel.head(2).squaredNorm();
        double pen_min_vel_log, grad_pen_min_vel_log;
        if(smoothedLog(pen_min_vel, 0.001, pen_min_vel_log, grad_pen_min_vel_log)){
          double special_step = step / dense_sample_resolution_;
          double special_s1 = s1;
          int disQuantity;
          double dense_alpha;
          Eigen::VectorXd gradv_dense(traj_dim_), grada_dense(traj_dim_), gradj_dense(traj_dim_);
          if(j == 0){
            disQuantity = dense_sample_resolution_ / 2;
            dense_alpha = 1.0 / K * j - 1.0 / K / dense_sample_resolution_;
          }else if(j == K){
            special_s1 = special_s1 - step / 2.0;
            disQuantity = dense_sample_resolution_ / 2;
            dense_alpha = 1.0 / K * j - 0.5 / K - 1.0 / K / dense_sample_resolution_;
          }else{
            special_s1 = special_s1 - step / 2.0;
            disQuantity = dense_sample_resolution_;
            dense_alpha = 1.0 / K * j - 0.5 / K - 1.0 / K / dense_sample_resolution_;
          }

          for(int l = 0; l <= disQuantity; l++){
            s2 = special_s1 * special_s1;
            s3 = s2 * special_s1;
            s4 = s2 * s2;
            s5 = s4 * special_s1;
            s6 = s3 * s3;
            s7 = s4 * s3;

            Densebeta0 << 1.0, special_s1, s2, s3, s4, s5, s6, s7;
            Densebeta1 << 0.0, 1.0, 2.0 * special_s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
            Densebeta2 << 0.0, 0.0, 2.0, 6.0 * special_s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
            Densebeta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * special_s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
            Densebeta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * special_s1, 360.0 * s2, 840 * s3;
            dense_alpha += 1.0 / K / dense_sample_resolution_;

            special_s1 += special_step;

            dense_pos = c.transpose() * Densebeta0;
            dense_vel = c.transpose() * Densebeta1;
            dense_acc = c.transpose() * Densebeta2;
            dense_jer = c.transpose() * Densebeta3;
            dense_sna = c.transpose() * Densebeta4;
            
            dense_omg = (l == 0 || l == disQuantity) ? 0.5 : 1.0;

            gradv_2d.setZero(); grada_2d.setZero(); gradj_2d.setZero();
            gradv_dense.setZero(); grada_dense.setZero(); gradj_dense.setZero(); gradv.setZero();
            double cost_mm_feasible = 0.0;
            if(feasibilityGradCostCar(dense_vel.head(2), dense_acc.head(2), dense_jer.head(2), trajid,
                                      gradv_2d, grada_2d, gradj_2d,
                                      cost_mm_feasible, true)){
              double cost_dense_log = pen_min_vel_log * cost_mm_feasible;
              
              gradv.head(2)       = -2 * grad_pen_min_vel_log * cost_mm_feasible * vel.head(2);
              gradv_dense.head(2) = pen_min_vel_log * gradv_2d;
              grada_dense.head(2) = pen_min_vel_log * grada_2d;
              gradj_dense.head(2) = pen_min_vel_log * gradj_2d;

              gradViolaVc = beta1 * gradv.transpose();
              gradViolaVt = alpha * gradv.transpose() * acc;
              gradViolaVc_dense = Densebeta1  * gradv_dense.transpose();
              gradViolaVt_dense = dense_alpha * gradv_dense.transpose() * dense_acc;
              gradViolaAc_dense = Densebeta2  * grada_dense.transpose();
              gradViolaAt_dense = dense_alpha * grada_dense.transpose() * dense_jer;
              gradViolaJc_dense = Densebeta3  * gradj_dense.transpose();
              gradViolaJt_dense = dense_alpha * gradj_dense.transpose() * dense_sna;

              SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += dense_omg * special_step * (gradViolaVc + gradViolaVc_dense + gradViolaAc_dense + gradViolaJc_dense);
              gdT(i) += dense_omg * (cost_dense_log / K / dense_sample_resolution_ + special_step * (gradViolaVt + gradViolaVt_dense + gradViolaAt_dense + gradViolaJt_dense));
              costs(3) += dense_omg * special_step * cost_dense_log;
            }
          }
        }
        // joint feasibility
        gradp.setZero(); gradv.setZero(); grada.setZero();
        double cost_joint_feasible = 0.0;
        if(feasibilityGradCostJoint(pos, vel, acc, gradp, gradv, grada, cost_joint_feasible)){
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaPc;
          gdT(i) += omg * (cost_joint_feasible / K + step * gradViolaPt);

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaVc;
          gdT(i) += omg * (step * gradViolaVt);

          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaAc;
          gdT(i) += omg * (step * gradViolaAt);

          costs(4) += omg * step * cost_joint_feasible;
        }
        s1 += step;
        if (j != K || (j == K && i == N - 1)){
          ++i_dp;
        }
      }
    }
  }

  bool PolyTrajOptimizer::obstacleGradCostforMM(const int i_dp,
                                            const Eigen::VectorXd &pos,
                                            const Eigen::VectorXd &vel,
                                            const int trajid,
                                            Eigen::VectorXd &gradp,
                                            Eigen::VectorXd &gradv,
                                            double &costp,
                                            double &costp_mani,
                                            double &costp_self,
                                            double query_time){
    // if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
    //   return false;
    bool ret = false;

    gradp.setZero();
    gradv.setZero();
    costp = 0;
    costp_mani = 0;
    costp_self = 0;

    double dist;
    double dist_err;
    double dist_err_2, dist_err_3;
    Eigen::Vector4d dist_grad4 = Eigen::Vector4d::Zero();
    Eigen::Vector4d dzd_p = Eigen::Vector4d::Zero();
    dzd_p(2) = 1;
    scratch_grad_.setZero();

    Eigen::Vector2d dYawdV = mm_config_->caldYawdV(vel.head(2));
    mm_config_->getCarPtsGradNew(pos.head(2), vel.head(2), singul_container_[trajid], Eigen::Vector3d(0, 0, 0),
                                 car_pts_buffer_, car_pts_yaw_grad_buffer_);
    double min_clearance = std::numeric_limits<double>::infinity();
    Eigen::Vector2d min_clearance_grad = Eigen::Vector2d::Zero();
    for(unsigned int i = 0; i < car_pts_buffer_.size(); ++i){
      Eigen::Vector3d dist_grad;
      grid_map_->evaluateEDTWithGradAtTime(car_pts_buffer_[i], query_time,
                                           dist, dist_grad);
      if(dist < min_clearance){
        min_clearance = dist;
        min_clearance_grad = dist_grad.head(2);
      }
      dist_err = mobile_base_check_radius_ + safe_margin_ - dist;
      if (dist_err > 0){
        dist_err_2 = dist_err * dist_err;
        dist_err_3 = dist_err_2 * dist_err;
        ret = true;
        costp += wei_obs_ * dist_err_3;
        gradp.head(mobile_base_dof_) -= wei_obs_ * 3.0 * dist_err_2 * dist_grad.head(2);
        gradv.head(mobile_base_dof_) -= wei_obs_ * 3.0 * dist_err_2 * dist_grad.head(2).transpose() * car_pts_yaw_grad_buffer_[i] * dYawdV;
      }
    }

    const Eigen::Vector2d base_vel = vel.head(2);
    const double speed = base_vel.norm();
    double d_extension_d_joint1 = 0.0;
    double d_extension_d_joint2 = 0.0;
    const double extension = armExtension(pos, d_extension_d_joint1, d_extension_d_joint2);
    const double base_stop_distance = 0.5 * speed * speed / stop_deceleration_ + speed * control_dt_;
    const double stop_distance = base_stop_distance * (1.0 + arm_extension_weight_ * extension);
    const double recovery_err = stop_distance + mobile_base_check_radius_ + safe_margin_ - min_clearance;
    if(recovery_err > 0.0){
      const double recovery_err_2 = recovery_err * recovery_err;
      const double recovery_err_3 = recovery_err_2 * recovery_err;
      ret = true;
      costp += wei_recoverability_ * recovery_err_3;
      gradp.head(2) -= wei_recoverability_ * 3.0 * recovery_err_2 * min_clearance_grad;
      if(speed > 1.0e-4){
        const double stop_gradient = speed / stop_deceleration_ + control_dt_;
        gradv.head(2) += wei_recoverability_ * 3.0 * recovery_err_2 *
                         (1.0 + arm_extension_weight_ * extension) * stop_gradient * base_vel / speed;
      }
      if(manipulator_dof_ >= 2){
        const double extension_gradient = wei_recoverability_ * 3.0 * recovery_err_2 *
                                          arm_extension_weight_ * base_stop_distance;
        gradp(mobile_base_dof_) += extension_gradient * d_extension_d_joint1;
        gradp(mobile_base_dof_ + 1) += extension_gradient * d_extension_d_joint2;
      }
    }

    int critical_joint = -1;
    double d_time_d_joint = 0.0;
    const double arm_recovery_time = jointRecoveryTime(pos, critical_joint, d_time_d_joint);
    const double stop_time = speed / std::max(1.0e-4, stop_deceleration_) + control_dt_;
    const double recovery_time_slack = recovery_horizon_ - stop_time - arm_recovery_time;
    if(recovery_time_slack < 0.0){
      const double recovery_time_err = -recovery_time_slack;
      ret = true;
      costp += wei_recovery_time_ * recovery_time_err * recovery_time_err * recovery_time_err;
      if(speed > 1.0e-4){
        gradv.head(2) += wei_recovery_time_ * 3.0 * recovery_time_err * recovery_time_err *
                         base_vel / (speed * std::max(1.0e-4, stop_deceleration_));
      }
      if(critical_joint >= 0){
        gradp(mobile_base_dof_ + critical_joint) +=
            wei_recovery_time_ * 3.0 * recovery_time_err * recovery_time_err * d_time_d_joint;
      }
    }

    if(manipulator_dof_ < 1) return ret;
    Eigen::Matrix4d T_w_q = Eigen::Matrix4d::Identity(), T_w_q_grad_x, T_w_q_grad_y;
    Eigen::Matrix2d R = mm_config_->calR(vel.head(2), singul_container_[trajid]);
    T_w_q.block(0, 0, 2, 2) = R;
    T_w_q.block(0, 3, 2, 1) = pos.head(2);
    T_w_q_grad_x.setZero();
    T_w_q_grad_x(0, 3) = 1;
    T_w_q_grad_y.setZero();
    T_w_q_grad_y(1, 3) = 1;
    // check mani link with obs
    mm_config_->getJointTrans(pos.tail(manipulator_dof_), joint_transform_buffer_, joint_gradient_buffer_);

    Eigen::Matrix4d T_now = T_w_q * T_q_0_;
    Eigen::Matrix4d T_caldv = T_q_0_; // nouse
    double yaw = atan2(singul_container_[trajid] * vel(1), singul_container_[trajid] * vel(0));
    Eigen::Matrix2d dRdYaw;
    dRdYaw << -sin(yaw), -cos(yaw),
              cos(yaw), -sin(yaw);
    Eigen::Matrix4d dTdYaw;
    dTdYaw.setZero();
    dTdYaw.block(0, 0, 2, 2) = dRdYaw;
    dTdYaw = dTdYaw * T_q_0_;
    transform_gradient_buffer_[0] = T_w_q_grad_x * T_q_0_;
    transform_gradient_buffer_[1] = T_w_q_grad_y * T_q_0_;

    Eigen::Matrix4d dRldv;
    Eigen::Vector3d pt_on_link;
    double ground_err = 0.0;
    double wei_ground_arg = 5.0;

    double costp_mani_nume = 0;
    for(int i = 0; i < manipulator_dof_; ++i){
      for(int j = 0; j < i + 2; ++j){
        transform_gradient_buffer_[j] = transform_gradient_buffer_[j] * joint_transform_buffer_[i];
      }
      T_caldv = T_caldv * joint_transform_buffer_[i];
      transform_gradient_buffer_[i + 2] = T_now * joint_gradient_buffer_[i];
      T_now = T_now * joint_transform_buffer_[i];
      dTdYaw = dTdYaw * joint_transform_buffer_[i];
      
      // get ESDF value & grad
      int pts_size = manipulator_link_pts_[i].cols();
      double factor = 1.0 / (double)pts_size;
      factor = 1.0;
      for(int j = 0; j < pts_size; ++j){
        pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
        Eigen::Vector3d dist_grad;
        grid_map_->evaluateEDTWithGradAtTime(pt_on_link, query_time,
                                             dist, dist_grad);
        dist_err = manipulator_thickness_ + safe_margin_mani_ - dist;
        if(dist_err > 0){
          ret = true;
          dist_grad4.segment(0, 3) = dist_grad;
          costp_mani += wei_mani_obs_ * factor * pow(dist_err, 3);
          costp_mani_nume += wei_mani_obs_ * factor * pow(dist_err, 3);
          scratch_grad_.setZero();
          for(int k = 0; k < i + 3; ++k){
            scratch_grad_(k) -= wei_mani_obs_ * factor * 3.0 * pow(dist_err, 2) * dist_grad4.transpose() * (transform_gradient_buffer_[k] * manipulator_link_pts_[i].col(j));
          }
          gradp += scratch_grad_;

          scratch_grad_.setZero();
          double dDistdYaw = dist_grad4.transpose() * (dTdYaw * manipulator_link_pts_[i].col(j));
          Eigen::Vector2d gradv_temp = wei_mani_obs_ * factor * 3.0 * pow(dist_err, 2) * dDistdYaw * dYawdV;
          scratch_grad_.head(2) -= gradv_temp;
          gradv += scratch_grad_;
        }
        ground_err = manipulator_thickness_ + ground_safe_dis_ + ground_safe_margin_ + map_resolution_ - pt_on_link(2);
        if(ground_err > 0){
          ground_err = manipulator_thickness_ + ground_safe_dis_ + ground_safe_margin_ - pt_on_link(2);
          double f_temp = 0, df_ground = 0;
          if(smoothedL1(ground_err, 0.0005, f_temp, df_ground)){
            costp_mani += wei_mani_obs_ * wei_ground_arg * factor * f_temp;
            scratch_grad_.setZero();
            for(int k = 0; k < i + 3; ++k){
              if(ground_err > 0){
                scratch_grad_(k) -= wei_mani_obs_ * wei_ground_arg * factor * df_ground * dzd_p.transpose() * (transform_gradient_buffer_[k] * manipulator_link_pts_[i].col(j));
              }
            }
            gradp += scratch_grad_;
          }
        }
      }
    }

    // check mani link with robot
    T_now = T_q_0_ * joint_transform_buffer_[0];
    self_gradient_buffer_[0] = T_q_0_ * joint_gradient_buffer_[0];
    mm_config_->getCarPts(Eigen::Vector3d::Zero(), self_car_pts_buffer_, Eigen::Vector3d::Zero());
    double factor2 = 1.0;
    for(int i = 1; i < manipulator_dof_; ++i){ // car will not collide with link 0
      for(int j = 0; j < i; ++j){
        self_gradient_buffer_[j] = self_gradient_buffer_[j] * joint_transform_buffer_[i];
      }
      self_gradient_buffer_[i] = T_now * joint_gradient_buffer_[i];
      T_now *= joint_transform_buffer_[i];
      int pts_size = manipulator_link_pts_[i].cols();
      double factor = 1.0;
      double wei_temp = wei_mani_self_ * factor * factor2;
      for(int j = 0; j < pts_size; ++j){
        // with car
        pt_on_link = (T_now * manipulator_link_pts_[i].col(j)).head(3);
        for(unsigned int m = 0; m < self_car_pts_buffer_.size(); ++m){
          dist = (self_car_pts_buffer_[m] - pt_on_link).norm(); // coarse distance
          dist_err = manipulator_thickness_ + mobile_base_check_radius_ + self_safe_margin_ - dist; //refine distance
          if(dist_err > 0){
            dist_err_2 = dist_err * dist_err;
            dist_err_3 = dist_err_2 * dist_err;
            dist_grad4.segment(0, 3) = (pt_on_link - self_car_pts_buffer_[m]) / dist;
            ret = true;
            costp_self += wei_temp * dist_err_3;
            scratch_grad_.setZero();
            for(int k = 0; k < i + 1; ++k){
              scratch_grad_(mobile_base_dof_ + k) -= 3.0 * wei_temp * dist_err_2 * (self_gradient_buffer_[k] * manipulator_link_pts_[i].col(j)).transpose() * dist_grad4;
            }
            gradp += scratch_grad_;
          }
        }

        // manipulator self-collision
        if(i < 2) continue;
        Eigen::Matrix4d T_now_self_mani = joint_transform_buffer_[i];
        self_chain_gradient_buffer_.clear();
        self_chain_gradient_buffer_.push_back(joint_gradient_buffer_[i]);
        // FIXME obj on ee may collide with arm i - 1
        for(int m = i - 2; m >= 0; --m){
          for(unsigned int n = 0; n < self_chain_gradient_buffer_.size(); ++n){
            self_chain_gradient_buffer_[n] = joint_transform_buffer_[m + 1] * self_chain_gradient_buffer_[n];
          }
          self_chain_gradient_buffer_.push_back(joint_gradient_buffer_[m + 1] * T_now_self_mani);
          T_now_self_mani = joint_transform_buffer_[m + 1] * T_now_self_mani;
          Eigen::Vector3d pt_on_link_to_check_m, pt_on_link_m;
          pt_on_link_m = (T_now_self_mani * manipulator_link_pts_[i].col(j)).head(3);
          int pts_size_temp = manipulator_link_pts_[m].cols();
          double factor = 1.0 / (double)pts_size_temp;
          for(int n = 0; n < pts_size_temp; ++n){
            pt_on_link_to_check_m = (manipulator_link_pts_[m].col(n)).head(3);
            dist = (pt_on_link_m - pt_on_link_to_check_m).norm(); // coarse distance
            dist_err = 2 * manipulator_thickness_ + self_safe_margin_ - dist; //refine distance
            if(dist_err > 0){
              dist_err_2 = dist_err * dist_err;
              dist_err_3 = dist_err_2 * dist_err;
              dist_grad4.segment(0, 3) = (pt_on_link_m - pt_on_link_to_check_m) / dist;
              ret = true;
              costp_self += wei_mani_self_ * factor * dist_err_3;
              scratch_grad_.setZero();
              int temp_grad_list_size = self_chain_gradient_buffer_.size() - 1;
              for(int k = temp_grad_list_size; k >= 0; --k){
                scratch_grad_(mobile_base_dof_ + m + k + 1) -= 3 * wei_mani_self_ * factor * dist_err_2 * (self_chain_gradient_buffer_[temp_grad_list_size - k] * manipulator_link_pts_[i].col(j)).transpose() * dist_grad4;
              }
              gradp += scratch_grad_;
            }
          }
        }
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostCar(const Eigen::Vector2d &vel, const Eigen::Vector2d &acc, const Eigen::Vector2d &jer, const int trajid,
                                                 Eigen::Vector2d &gradv_2d, Eigen::Vector2d &grada_2d, Eigen::Vector2d &gradj_2d,
                                                 double &cost_mm_feasible, bool dense_sample){
    bool ret = false;
    double pen, f, df;
    cost_mm_feasible = 0;

    const double vTv = vel.transpose() * vel;
    const double aTv = acc.transpose() * vel;
    const double aTBv = acc.transpose() * B_h_ * vel;
    const double jTBv = jer.transpose() * B_h_ * vel;
    const double v_norm = vel.norm();
    const double vTv_inv = 1.0 / vTv; // avoid siguality vel = 0
    const double vTv_inv2 = vTv_inv * vTv_inv;

    
    // Check minimum speed
    if(dense_sample){ 
      pen = non_singul_v_ * non_singul_v_ - vel.squaredNorm();
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        cost_mm_feasible += wei_feas_ * 1e6 * f;
        gradv_2d += -wei_feas_ * 1e6 * df * 2.0 * vel;
        ret = true;
      }
    }

    // Linear velocity
    pen = vel.squaredNorm() - max_vel_ * max_vel_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * vel;
      ret = true;
    }

    double omega = aTBv * vTv_inv;
    double wheel_omega_left = (2.0 * singul_container_[trajid] * v_norm - mobile_base_wheel_base_ * omega) / (2.0 * mobile_base_wheel_radius_);
    Eigen::Vector2d dOmegadV = B_h_.transpose() * acc / vTv - (aTBv * vel + vel.transpose() * B_h_.transpose() * acc * vel) / vTv / vTv;
    Eigen::Vector2d dOmegadA = vTv_inv * (B_h_ * vel);
    // Left wheel rotation speed
    pen = wheel_omega_left * wheel_omega_left - max_wheel_omega_ * max_wheel_omega_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_omega_left * (2.0 * singul_container_[trajid] / v_norm * vel - mobile_base_wheel_base_ * dOmegadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += -wei_feas_ * df * 2.0 * wheel_omega_left * mobile_base_wheel_base_ * dOmegadA / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    // Right wheel rotation speed
    double wheel_omega_right = (2.0 * singul_container_[trajid] * v_norm + mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);
    pen = wheel_omega_right * wheel_omega_right - max_wheel_omega_ * max_wheel_omega_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_omega_right * (2.0 * singul_container_[trajid] / v_norm * vel + mobile_base_wheel_base_ * dOmegadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_omega_right * mobile_base_wheel_base_ * dOmegadA / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    double alpha = jTBv * vTv_inv - 2.0 * aTBv * aTv * vTv_inv2;
    Eigen::Vector2d dAlphadV = (B_h_.transpose() * jer * vTv - 2.0 * jTBv * vel) * vTv_inv2
                              - 2.0 * (aTBv * acc + aTv * B_h_.transpose() * acc) * vTv_inv2
                              + 8.0 * vTv_inv2 * vTv_inv * aTBv * aTv * vel;
    Eigen::Vector2d dAlphadA = -2.0 * vTv_inv2 * (aTv * B_h_ * vel + aTBv * vel);
    Eigen::Vector2d dAlphadJ = B_h_ * vel * vTv_inv;

    // Left wheel rotational acceleration
    double wheel_alpha_left = (2.0 * singul_container_[trajid] * aTv / v_norm - mobile_base_wheel_base_ * alpha) / (2.0 * mobile_base_wheel_radius_);
    pen = wheel_alpha_left * wheel_alpha_left - max_wheel_alpha_ * max_wheel_alpha_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_alpha_left * (2.0 * singul_container_[trajid] * (v_norm * acc - aTv * vel / v_norm) * vTv_inv - mobile_base_wheel_base_ * dAlphadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_alpha_left * (2.0 * singul_container_[trajid] * vel / v_norm - mobile_base_wheel_base_ * dAlphadA) / (2.0 * mobile_base_wheel_radius_);
      gradj_2d += -wei_feas_ * df * 2.0 * wheel_alpha_left * mobile_base_wheel_base_ * dAlphadJ / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    // Right wheel rotational acceleration
    double wheel_alpha_right = (2.0 * singul_container_[trajid] * aTv / v_norm + mobile_base_wheel_base_ * alpha) / (2.0 * mobile_base_wheel_radius_);
    pen =  wheel_alpha_right * wheel_alpha_right - max_wheel_alpha_ * max_wheel_alpha_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_alpha_right * (2.0 * singul_container_[trajid] * (v_norm * acc - aTv * vel / v_norm) * vTv_inv + mobile_base_wheel_base_ * dAlphadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_alpha_right * (2.0 * singul_container_[trajid] * vel / v_norm + mobile_base_wheel_base_ * dAlphadA) / (2.0 * mobile_base_wheel_radius_);
      gradj_2d += wei_feas_ * df * 2.0 * wheel_alpha_right * mobile_base_wheel_base_ * dAlphadJ / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }
    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostJoint(const Eigen::VectorXd &pos, const Eigen::VectorXd &vel, const Eigen::VectorXd &acc, 
                                                    Eigen::VectorXd &gradp, Eigen::VectorXd &gradv, Eigen::VectorXd &grada,
                                                    double &cost_joint_feasible){
    bool ret = false;
    double pen, f, df;

    // pos limit
    for(int i = 0; i < manipulator_dof_; ++i){
      pen = pos(mobile_base_dof_ + i) - max_joint_pos_[i];
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        cost_joint_feasible += wei_mani_feas_ * f;
        gradp(mobile_base_dof_ + i) += wei_mani_feas_ * df;
        ret = true;
      }

      pen = min_joint_pos_[i] - pos(mobile_base_dof_ + i);
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        cost_joint_feasible += wei_mani_feas_ * f;
        gradp(mobile_base_dof_ + i) += wei_mani_feas_ * df * (-1.0);
        ret = true;
      }
    }

    // vel limit
    for(int i = 0; i < manipulator_dof_; ++i){
      pen = vel(mobile_base_dof_ + i) * vel(mobile_base_dof_ + i) - max_joint_vel_ * max_joint_vel_;
      f = 0, df = 0;
      if(smoothedL1(pen,0.005,f,df)){
        gradv(mobile_base_dof_ + i) += wei_mani_feas_ * df * 2.0 * vel(mobile_base_dof_ + i);
        cost_joint_feasible += wei_mani_feas_ * f;
        ret = true;
      }
    }

    // acc limit
    for(int i = 0; i < manipulator_dof_; ++i){
      pen = acc(mobile_base_dof_ + i) * acc(mobile_base_dof_ + i) - max_joint_acc_ * max_joint_acc_;
      f = 0, df = 0;
      if(smoothedL1(pen,0.005,f,df)){
        grada(mobile_base_dof_ + i) += wei_mani_feas_ * df * 2.0 * acc(mobile_base_dof_ + i);
        cost_joint_feasible += wei_mani_feas_ * f;
        ret = true;
      }
    }

    return ret;
  }

  int PolyTrajOptimizer::astarWithMinTraj(const Eigen::MatrixXd &iniState,
                                           const Eigen::MatrixXd &finState,
                                           const double start_yaw,
                                           const int _start_singul,
                                           const bool start_gripper,
                                           const double end_yaw,
                                           const bool end_gripper,
                                           const Eigen::Vector2d init_ctrl,
                                           const int continous_failures_count,
                                           std::vector<std::vector<Eigen::VectorXd>> &simple_path_container,  // 前进/倒车为一段, 每段N个点x, y, mani_angle
                                           std::vector<std::vector<double>> &yaw_list_container,
                                           std::vector<poly_traj::MinSnapOpt<8>> &frontendMJ_container,
                                           std::vector<int> &singul_container){
    Eigen::VectorXd start_pos = iniState.col(0);
    Eigen::VectorXd end_pos = finState.col(0);
    Eigen::VectorXd start_vel = iniState.col(1);
    Eigen::VectorXd end_vel = finState.col(1);
    int start_singul = _start_singul;
    std::vector<Eigen::VectorXd> t_list_container; t_list_container.clear();
    vector<Eigen::VectorXd> simple_path; simple_path.clear();
    std::vector<double> yaw_list; yaw_list.clear();
    Eigen::VectorXd t_list;
    
    simple_path_container.clear();
    yaw_list_container.clear();
    frontendMJ_container.clear();
    singul_container.clear();

    /* astar search and get the simple path*/
    int status = kino_a_star_->KinoAstarSearchAndGetSimplePath(start_pos, start_vel, start_yaw, start_singul, start_gripper,
                                                              end_pos, end_vel, end_yaw, end_gripper, init_ctrl,
                                                              continous_failures_count,
                                                              simple_path_container, yaw_list_container, 
                                                              singul_container, t_list_container);
  
    if(status == KinoAstar::NO_PATH || status == KinoAstar::START_COLLISION || status == KinoAstar::GOAL_COLLISION){
      return status;
    }
    Eigen::MatrixXd innerPts;
    Eigen::MatrixXd headState, tailState;
    headState.resize(traj_dim_, 4);
    tailState.resize(traj_dim_, 4);
    frontendMJ_container.resize(simple_path_container.size());
    for(unsigned int i = 0; i < simple_path_container.size(); ++i){

      int piece_num = simple_path_container[i].size() - 1;
      if (piece_num > 1){
        innerPts.resize(traj_dim_, piece_num - 1);
        for (int j = 0; j < piece_num - 1; ++j){
          innerPts.col(j) = (simple_path_container[i])[j + 1]; // front end
        }
      }
      else{
        // ROS_WARN("simple path is too short");
        piece_num = 2;
        innerPts.resize(traj_dim_, 1);
        innerPts.col(0) = ((simple_path_container[i])[0] + (simple_path_container[i])[1]) / 2;
        simple_path_container[i].insert(simple_path_container[i].begin() + 1, innerPts.col(0));
        t_list.resize(2);
        t_list.setConstant((t_list_container[i])[0] / 2);
        t_list_container[i] = t_list;
      }

      t_list = t_list_container[i];
      // t_list = t_list * 1.2;
      Eigen::VectorXd angle1, angle2;
      double err;
      // double max_t = t_list[0];
      double t_acc = max_joint_vel_ / max_joint_acc_;
      double dist_acc = max_joint_acc_ * t_acc * t_acc;
      // std::cout << "astarWithMinTraj 8 " << i << "\n";
      for (int j = 0; j < piece_num; ++j){
        angle1 = (simple_path_container[i])[j].tail(manipulator_dof_);
        angle2 = (simple_path_container[i])[j + 1].tail(manipulator_dof_);
        double max_t = t_list[j];
        for(int k = 0; k < manipulator_dof_; ++k){
          if(angle1(k) - angle2(k) >= 0)
            err = angle1(k) - angle2(k);
          else
            err = angle2(k) - angle1(k);
          double t_acc_vel = 0;
          if(j == 0 || j == piece_num - 1){
            if(err <= dist_acc){
              t_acc_vel = sqrt(err / max_joint_acc_);
            }else{
              t_acc_vel = (err - dist_acc) / max_joint_vel_ + 2 * t_acc;
            }
          }else{
            t_acc_vel = err / max_joint_vel_;
          }
          max_t = max(t_acc_vel, max_t);
        }
        t_list[j] = max_t;
      }
      t_list[0] *= 1.5;
      t_list[piece_num - 1] *= 1.5;

      if(i > 0){
        headState.setZero();
        headState.col(0) = (simple_path_container[i])[0];
        headState.col(1).head(2) = singul_container[i] * non_singul_v_ * Eigen::Vector2d(cos(yaw_list_container[i][0]), sin(yaw_list_container[i][0]));
      }else{
        headState = iniState;
        if(_start_singul == 0){
          headState.col(1).head(2) = singul_container[i] * non_singul_v_ * Eigen::Vector2d(cos(yaw_list_container[i][0]), sin(yaw_list_container[i][0]));
        }
      }
      if(i < simple_path_container.size() - 1 || status == KinoAstar::REACH_HORIZON){
        tailState.setZero();
        tailState.col(0) = (simple_path_container[i]).back();
        tailState.col(1).head(2) = singul_container[i] * non_singul_v_ * Eigen::Vector2d(cos(yaw_list_container[i].back()), sin(yaw_list_container[i].back()));
      }else{
        tailState = finState; // end vel acc jer dir is foreward
        tailState.col(1).head(2) = singul_container[i] * tailState.col(1).head(2);
        tailState.col(2).head(2) = singul_container[i] * tailState.col(2).head(2);
        tailState.col(3).head(2) = singul_container[i] * tailState.col(3).head(2);
      }
      frontendMJ_container[i].reset(headState, tailState, piece_num);
      frontendMJ_container[i].generate(innerPts, t_list_container[i]);
    }
    return status;
  }

  void PolyTrajOptimizer::displayFrontEndMesh(std::vector<Eigen::VectorXd> &simple_path_full, vector<double> &yaw_list){
    if(front_end_mm_mesh_vis_pub_.getNumSubscribers() == 0){
      return;
    }

    Eigen::Vector3d car_state;
    Eigen::VectorXd joint_state;
    joint_state.resize(manipulator_dof_);

    visualization_msgs::MarkerArray marker_array, marker_array_all;
    visualization_msgs::Marker marker_delete_all;
    marker_delete_all.action = visualization_msgs::Marker::DELETEALL;
    marker_array_all.markers.push_back(marker_delete_all);
    
    for(unsigned int i = 0; i < simple_path_full.size(); i+=1){
      car_state.head(2) = simple_path_full[i].head(2);
      car_state(2) = yaw_list[i];
      joint_state = simple_path_full[i].tail(manipulator_dof_);
      mm_config_->getMMMarkerArray(marker_array, "vis_mm_front_end", i, 0.07, car_state, joint_state, true); // TODO gripper state
      marker_array_all.markers.insert(marker_array_all.markers.end(), marker_array.markers.begin(), marker_array.markers.end());
    }

    front_end_mm_mesh_vis_pub_.publish(marker_array_all);

  }

  void PolyTrajOptimizer::clear_resize_Cps_container(int container_size){
    cps_container_.clear();
    cps_container_.resize(container_size);
  }

  void PolyTrajOptimizer::setControlPoints(const int trajid, const Eigen::MatrixXd &points){
    cps_container_[trajid].resize_cp(points.cols());
    cps_container_[trajid].points = points;
  }

}
