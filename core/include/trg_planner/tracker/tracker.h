#ifndef CPP_TRG_PLANNER_CORE_TRG_PLANNER_INCLUDE_TRACKER_TRACKER_H_
#define CPP_TRG_PLANNER_CORE_TRG_PLANNER_INCLUDE_TRACKER_TRACKER_H_

#include <algorithm>

#include "trg_planner/utils/common.h"

// Omnidirectional 路径跟踪器 (方案A: yaw 跟随路径切向, w = k_yaw*yaw_err + v*curvature)
class OmniPathTracker {
 public:
  struct Params {
    float lookahead_base      = 0.6f;
    float lookahead_k_v       = 0.5f;
    float max_speed           = 1.0f;
    float max_vx              = 1.0f;
    float max_vy              = 1.0f;
    float max_w               = 2.5f;
    float a_lat_max           = 1.0f;
    float a_lin_max           = 1.5f;
    float k_px                = 1.0f;
    float k_py                = 2.0f;
    float k_yaw               = 2.0f;
    float k_dyaw              = 0.0f;
    float goal_tolerance_pos  = 0.05f;
    float goal_tolerance_yaw  = 0.05f;
    float stop_slowdown_dist  = 0.8f;
    float finished_wait_time  = 0.2f;
    bool  enable_curvature_ff = true;
  } param_;

  struct Cmd {
    float vx = 0, vy = 0, w = 0;
  } cmd_;

  OmniPathTracker() = default;
  explicit OmniPathTracker(const Params& p) : param_(p) {}
  void setParams(const Params& p) { param_ = p; }

  void setPath(const std::vector<Eigen::Vector3f>& path3d) {
    std::scoped_lock lk(mtx_);
    path2d_.clear();
    path2d_.reserve(path3d.size());
    for (const auto& p : path3d) path2d_.push_back({p.x(), p.y()});
    progress_idx_       = 0;
    finished_           = false;
    finish_stamp_valid_ = false;
  }
  void clearPath() {
    std::scoped_lock lk(mtx_);
    path2d_.clear();
    progress_idx_       = 0;
    finished_           = true;
    finish_stamp_valid_ = false;
  }

  // 删除旧接口, 新接口: 位置(Eigen::Vector3f), yaw(float), 线速度(Eigen::Vector3f),
  // z轴角速度(float), dt(float)
  Cmd computeCommand(const Eigen::Vector3f& pos,
                     float                  yaw,
                     const Eigen::Vector3f& lin_vel,
                     float                  wz,
                     float                  dt) {
    std::scoped_lock lk(mtx_);
    pos_x_   = pos.x();
    pos_y_   = pos.y();
    yaw_     = yaw;
    vx_body_ = lin_vel.x();
    vy_body_ = lin_vel.y();
    wz_body_ = wz;
    if (path2d_.size() < 2) {
      cmd_      = {};
      finished_ = true;
      return cmd_;
    }
    advanceProgress();
    float dx_end       = path2d_.back().x - pos_x_;
    float dy_end       = path2d_.back().y - pos_y_;
    float d_rem        = std::sqrt(dx_end * dx_end + dy_end * dy_end);
    float v_goal_scale = 1.f;
    if (d_rem < param_.stop_slowdown_dist)
      v_goal_scale = std::clamp(d_rem / std::max(0.01f, param_.stop_slowdown_dist), 0.f, 1.f);
    Tangent tan    = currentTangent();
    float   kappa  = localCurvature();
    float  v_kappa = std::sqrt(std::max(0.f, param_.a_lat_max / std::max(std::fabs(kappa), 1e-4f)));
    float  v_des   = std::min({param_.max_speed, v_kappa, param_.max_speed * v_goal_scale});
    float  v_body_norm = std::sqrt(vx_body_ * vx_body_ + vy_body_ * vy_body_);
    float  Ld          = param_.lookahead_base + param_.lookahead_k_v * v_body_norm;
    Point2 P_la        = findLookahead(Ld);
    float  cy          = std::cos(yaw_);
    float  sy          = std::sin(yaw_);
    float  vff_wx      = v_des * tan.x;
    float  vff_wy      = v_des * tan.y;
    float  v_ff_bx     = cy * vff_wx + sy * vff_wy;
    float  v_ff_by     = -sy * vff_wx + cy * vff_wy;
    float  ex_w        = P_la.x - pos_x_;
    float  ey_w        = P_la.y - pos_y_;
    float  ex_b        = cy * ex_w + sy * ey_w;
    float  ey_b        = -sy * ex_w + cy * ey_w;
    float  vx          = v_ff_bx + param_.k_px * ex_b;
    float  vy          = v_ff_by + param_.k_py * ey_b;
    float  yaw_d       = std::atan2(tan.y, tan.x);
    float  yaw_err     = std::atan2(std::sin(yaw_d - yaw_), std::cos(yaw_d - yaw_));
    float  w_ff        = param_.enable_curvature_ff ? v_des * kappa : 0.f;
    float  yaw_err_dot = w_ff - wz_body_;  // 期望角速度-实际角速度
    float  w           = w_ff + param_.k_yaw * yaw_err + param_.k_dyaw * yaw_err_dot;
    // 加速度限幅
    if (dt > 1e-4f) {
      float dvx     = vx - cmd_.vx;
      float dvy     = vy - cmd_.vy;
      float dv_norm = std::sqrt(dvx * dvx + dvy * dvy);
      float max_dv  = param_.a_lin_max * dt;
      if (dv_norm > max_dv) {
        float s = max_dv / dv_norm;
        dvx *= s;
        dvy *= s;
        vx = cmd_.vx + dvx;
        vy = cmd_.vy + dvy;
      }
    }
    vx = std::clamp(vx, -param_.max_vx, param_.max_vx);
    vy = std::clamp(vy, -param_.max_vy, param_.max_vy);
    w  = std::clamp(w, -param_.max_w, param_.max_w);
    if (d_rem < param_.goal_tolerance_pos) {
      if (!finish_stamp_valid_) {
        finish_stamp_       = std::chrono::steady_clock::now();
        finish_stamp_valid_ = true;
      }
      float dt_finish = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - finish_stamp_)
                            .count() /
                        1000.f;
      if (dt_finish > param_.finished_wait_time && std::fabs(yaw_err) < param_.goal_tolerance_yaw) {
        vx        = 0;
        vy        = 0;
        w         = 0;
        finished_ = true;
      }
    } else {
      finish_stamp_valid_ = false;
      finished_           = false;
    }
    cmd_.vx = vx;
    cmd_.vy = vy;
    cmd_.w  = w;
    return cmd_;
  }

  bool   isFinished() const { return finished_; }
  size_t progressIndex() const { return progress_idx_; }

 private:
  struct Point2 {
    float x;
    float y;
  };
  struct Tangent {
    float x;
    float y;
  };

  void advanceProgress() {
    if (progress_idx_ >= path2d_.size() - 1) return;
    bool advanced = true;
    int  safety   = 0;
    while (advanced && progress_idx_ < path2d_.size() - 1 && safety < 20) {
      safety++;
      advanced    = false;
      Point2 A    = path2d_[progress_idx_];
      Point2 B    = path2d_[progress_idx_ + 1];
      float  ABx  = B.x - A.x;
      float  ABy  = B.y - A.y;
      float  len2 = ABx * ABx + ABy * ABy;
      if (len2 < 1e-6f) {
        progress_idx_++;
        advanced = true;
        continue;
      }
      float APx = pos_x_ - A.x;
      float APy = pos_y_ - A.y;
      float t   = (APx * ABx + APy * ABy) / len2;
      if (t > 1.f) {
        progress_idx_++;
        advanced = true;
      }
    }
  }

  Point2 findLookahead(float Ld) const {
    if (progress_idx_ >= path2d_.size() - 1) return path2d_.back();
    float acc = 0.f;
    for (size_t i = progress_idx_; i + 1 < path2d_.size(); ++i) {
      Point2 p0 = path2d_[i];
      Point2 p1 = path2d_[i + 1];
      float  dx = p1.x - p0.x;
      float  dy = p1.y - p0.y;
      float  ds = std::sqrt(dx * dx + dy * dy);
      if (acc + ds >= Ld) {
        float r = (Ld - acc) / std::max(1e-6f, ds);
        if (r < 0) r = 0;
        if (r > 1) r = 1;
        return {p0.x + r * dx, p0.y + r * dy};
      }
      acc += ds;
    }
    return path2d_.back();
  }

  float localCurvature() const {
    if (path2d_.size() < 3) return 0.f;
    size_t i = progress_idx_;
    if (i == 0) i = 1;
    if (i >= path2d_.size() - 1) i = path2d_.size() - 2;
    const Point2& p0    = path2d_[i - 1];
    const Point2& p1    = path2d_[i];
    const Point2& p2    = path2d_[i + 1];
    float         a_x   = p1.x - p0.x;
    float         a_y   = p1.y - p0.y;
    float         b_x   = p2.x - p1.x;
    float         b_y   = p2.y - p1.y;
    float         la    = std::sqrt(a_x * a_x + a_y * a_y);
    float         lb    = std::sqrt(b_x * b_x + b_y * b_y);
    float         cross = a_x * b_y - a_y * b_x;
    float         denom = la * lb * (la + lb);
    if (denom < 1e-6f) return 0.f;
    return 2.f * cross / denom;
  }

  Tangent currentTangent() const {
    if (progress_idx_ >= path2d_.size() - 1) {
      if (path2d_.size() >= 2) {
        const auto& pN = path2d_.back();
        const auto& pP = path2d_[path2d_.size() - 2];
        float       dx = pN.x - pP.x;
        float       dy = pN.y - pP.y;
        float       n  = std::sqrt(dx * dx + dy * dy);
        if (n < 1e-6f) return {1.f, 0.f};
        return {dx / n, dy / n};
      }
      return {1.f, 0.f};
    }
    const auto& A  = path2d_[progress_idx_];
    const auto& B  = path2d_[progress_idx_ + 1];
    float       dx = B.x - A.x;
    float       dy = B.y - A.y;
    float       n  = std::sqrt(dx * dx + dy * dy);
    if (n < 1e-6f) return {1.f, 0.f};
    return {dx / n, dy / n};
  }

  std::vector<Point2> path2d_;
  size_t              progress_idx_ = 0;
  bool                finished_     = true;
  float               prev_yaw_err_ = 0.f;

  float pos_x_ = 0.f, pos_y_ = 0.f, yaw_ = 0.f;  // 里程计
  float vx_body_ = 0.f, vy_body_ = 0.f;          // 当前机体系速度
  float wz_body_ = 0.f;                          // z轴角速度

  mutable std::mutex                    mtx_;
  bool                                  finish_stamp_valid_ = false;
  std::chrono::steady_clock::time_point finish_stamp_;
};

#endif  // CPP_TRG_PLANNER_CORE_TRG_PLANNER_INCLUDE_TRACKER_TRACKER_H_
