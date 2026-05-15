#pragma once
// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 英雄发射机构独立实现，负责摩擦轮、拨弹盘控制与热量约束发射逻辑
constructor_args:
  - task_stack_depth: 1536
  - launcher_param:
      trig_gear_ratio: 19.2032
      num_trig_tooth: 6
      speed_sync: false
  - cmd: '@&cmd'
  - fric_setpoint_speed_0: 3900.0
  - fric_setpoint_speed_1: 2700.0
  - pid_trig_angle:
      k: 1.0
      p: 2000.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 2000.0
      cycle: true
  - pid_trig_speed:
      k: 1.0
      p: 0.0013
      i: 0.0
      d: 0.0
      i_limit: 1.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_0:
      k: 1.0
      p: 0.0003
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_1:
      k: 1.0
      p: 0.0003
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_2:
      k: 1.0
      p: 0.0003
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_3:
      k: 1.0
      p: 0.0003
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - fric_motor_0: '@&motor_fric_front_left'
  - fric_motor_1: '@&motor_fric_front_right'
  - fric_motor_2: '@&motor_fric_back_left'
  - fric_motor_3: '@&motor_fric_back_right'
  - motor_trig_: '@&motor_trig'
  - referee: '@&ref'
  - thread_priority: LibXR::Thread::Priority::MEDIUM
template_args: []
required_hardware:
  - motor
depends:
  - qdu-future/RMMotor
  - qdu-future/CMD
  - xrobot-org/Referee
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cstdint>

#include "CMD.hpp"
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#define UI_HERO_LAUNCHER_LAYER 2

/**
 * @brief 英雄发射机构独立实现
 * @details 负责摩擦轮、拨弹盘控制与热量约束发射逻辑。
 *          包含完整的线程管理和事件系统，可独立使用。
 */
class HeroLauncher {
 public:
  static constexpr int FRIC_NUM = 4;
  static constexpr float TRIG_ZERO_ANGLE_OFFSET = 0.65f;
  static constexpr float TRIG_LOADING_ANGLE_STEP =
      static_cast<float>(LibXR::TWO_PI) / 1000.0f;

  enum class TrigMode : uint8_t {
    RELAX = 0,
    SAFE,
    SINGLE,
    CONTINUE,
  };

  enum class LauncherEvent : uint8_t {
    SET_FRICMODE_RELAX,
    SET_FRICMODE_SAFE,
    SET_FRICMODE_READY,
  };
  struct HeatControl {
    float heat;          /* 现在热量水平 */
    float last_heat;     /* 之前的热量水平 */
    float heat_limit;    /* 热量上限 */
    float speed_limit;   /* 弹丸初速上限 */
    float cooling_rate;  /* 冷却速率 */
    float heat_increase; /* 每发热量增加值 */

    uint8_t cooling_acc;  // 冷却增益

    uint32_t available_shot; /* 热量范围内还可以发射的数量 */
  };

  struct LauncherParam {
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
    bool speed_sync;
  };
  /**
   * @brief 构造 HeroLauncher
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param task_stack_depth 线程栈深
   * @param launcher_param 发射器参数
   * @param cmd CMD 模块指针
   * @param fric_setpoint_speed_0 第一级摩擦轮目标转速
   * @param fric_setpoint_speed_1 第二级摩擦轮目标转速
   * @param pid_fric_speed_0 摩擦轮0 PID参数
   * @param pid_fric_speed_1 摩擦轮1 PID参数
   * @param pid_fric_speed_2 摩擦轮2 PID参数
   * @param pid_fric_speed_3 摩擦轮3 PID参数
   * @param fric_motor_0 摩擦轮0电机指针
   * @param fric_motor_1 摩擦轮1电机指针
   * @param fric_motor_2 摩擦轮2电机指针
   * @param fric_motor_3 摩擦轮3电机指针
   * @param motor_trig_ 拨弹电机指针
   * @param thread_priority 线程优先级
   */
  HeroLauncher(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      uint32_t task_stack_depth, LauncherParam launcher_param, CMD* cmd,
      float fric_setpoint_speed_0, float fric_setpoint_speed_1,
      LibXR::PID<float>::Param pid_trig_angle,
      LibXR::PID<float>::Param pid_trig_speed,
      LibXR::PID<float>::Param pid_fric_speed_0,
      LibXR::PID<float>::Param pid_fric_speed_1,
      LibXR::PID<float>::Param pid_fric_speed_2,
      LibXR::PID<float>::Param pid_fric_speed_3, RMMotor* fric_motor_0,
      RMMotor* fric_motor_1, RMMotor* fric_motor_2, RMMotor* fric_motor_3,
      RMMotor* motor_trig, Referee* ref,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM)
      : cmd_(cmd),
        speed_sync_(launcher_param.speed_sync),
        trig_angle_pid_(pid_trig_angle),
        trig_speed_pid_(pid_trig_speed),
        fric_speed_pid_({{LibXR::PID<float>(pid_fric_speed_0),
                          LibXR::PID<float>(pid_fric_speed_1),
                          LibXR::PID<float>(pid_fric_speed_2),
                          LibXR::PID<float>(pid_fric_speed_3)}}),
        trig_gear_ratio_(launcher_param.trig_gear_ratio),
        num_trig_tooth_(launcher_param.num_trig_tooth),
        ref_(ref) {
    motor_trig_ = motor_trig;

    fric_motor_[0] = fric_motor_0;
    fric_motor_[1] = fric_motor_1;
    fric_motor_[2] = fric_motor_2;
    fric_motor_[3] = fric_motor_3;

    param_fric_target_speed_[0] = fric_setpoint_speed_0;
    param_fric_target_speed_[1] = fric_setpoint_speed_1;

    last_wakeup_ = LibXR::Timebase::GetMicroseconds();

    thread_.Create(this, ThreadFunc, "HeroLauncherThread", task_stack_depth,
                   thread_priority);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, HeroLauncher* self, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->LostCtrl();
          self->mutex_.Unlock();
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, HeroLauncher* self, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->SetMode(
              static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX));
          self->mutex_.Unlock();
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);

    auto event_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, HeroLauncher* self, uint32_t event_id) {
          UNUSED(in_isr);
          self->mutex_.Lock();
          self->SetMode(event_id);
          self->mutex_.Unlock();
        },
        this);
    event_.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX),
                    event_callback);
    event_.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE),
                    event_callback);
    event_.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY),
                    event_callback);

    void (*DrawUi)(HeroLauncher*) = [](HeroLauncher* hero) { hero->DrawUI(); };
    ui_timer_handle_ = LibXR::Timer::CreateTask(DrawUi, this, 47);
    LibXR::Timer::Add(ui_timer_handle_);
    LibXR::Timer::Start(ui_timer_handle_);
  }

  /**
   * @brief 线程主函数
   */
  static void ThreadFunc(HeroLauncher* self) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> cmd_sub("launcher_cmd");
    LibXR::Topic::ASyncSubscriber<Referee::LauncherPack> launcher_ref(
        "launcher_ref");
    cmd_sub.StartWaiting();
    launcher_ref.StartWaiting();

    self->last_wakeup_time_ = LibXR::Timebase::GetMilliseconds();
    self->last_online_time_ = LibXR::Timebase::GetMicroseconds();

    while (true) {
      LibXR::Thread::Sleep(2);

      auto now = LibXR::Timebase::GetMicroseconds();
      self->dt_ = (now - self->last_online_time_).ToSecondf();
      self->last_online_time_ = now;

      if (cmd_sub.Available()) {
        self->launcher_cmd_ = cmd_sub.GetData();
        cmd_sub.StartWaiting();
      }
      if (launcher_ref.Available()) {
        self->ref_data_.rs.shooter_cooling_value =
            launcher_ref.GetData().rs.shooter_cooling_value;
        self->ref_data_.rs.shooter_heat_limit =
            launcher_ref.GetData().rs.shooter_heat_limit;
        launcher_ref.StartWaiting();
      }

      self->mutex_.Lock();
      self->Update();
      self->Solve();
      self->mutex_.Unlock();
      self->Control();
    }
  }

  LibXR::Event& GetEvent() { return event_; }

  void OnMonitor() {}

 private:
  CMD* cmd_;

  TrigMode trig_mode_ = TrigMode::SAFE;

  HeatControl heat_ctrl_;

  bool first_loading_ = true;

  bool speed_sync_ = false;

  bool soft_start_finish_ = false;

  float dt_ = 0.0f;

  LibXR::MillisecondTimestamp now_ = 0;

  LibXR::MicrosecondTimestamp last_wakeup_;

  LibXR::MillisecondTimestamp last_change_angle_time_ = 0;

  LibXR::MillisecondTimestamp start_loading_time_ = 0;

  RMMotor* fric_motor_[FRIC_NUM];

  RMMotor* motor_trig_;

  LibXR::ErrorCode motor_state_;

  float trig_setpoint_angle_ = 0.0f;
  float trig_setpoint_speed_ = 0.0f;

  float trig_zero_angle_ = 0.0f;
  float trig_angle_ = 0.0f;
  float trig_output_ = 0.0f;

  float param_fric_target_speed_[2] = {0.0f, 0.0f};
  float fric_target_rpm_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  LibXR::PID<float> trig_angle_pid_;
  LibXR::PID<float> trig_speed_pid_;

  std::array<LibXR::PID<float>, FRIC_NUM> fric_speed_pid_;

  LibXR::PID<float> fric_sync_pid_front_{LibXR::PID<float>::Param{
      1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, false}};  // 前摩擦轮同步PID (0-1)
  LibXR::PID<float> fric_sync_pid_back_{LibXR::PID<float>::Param{
      1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, false}};  // 后摩擦轮同步PID (2-3)

  float trig_gear_ratio_;
  uint8_t num_trig_tooth_;

  bool fire_flag_ = false;    // 发射命令标志位
  uint8_t fired_ = 0;         // 已发射弹丸
  bool enable_fire_ = false;  // 拨弹盘旋转命令发出标志位
  bool mark_launch_ = false;  // 拨弹发射完成标志位

  bool stuck = false;  // 拨弹卡弹标志位

  LibXR::MillisecondTimestamp start_fire_time_ = 0;
  LibXR::MillisecondTimestamp finish_fire_time_ = 0;
  uint32_t real_launch_delay_ = 0.0f;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;

  uint16_t delay_time_ = 0;

  LauncherEvent launcher_state_ = LauncherEvent::SET_FRICMODE_RELAX;

  Motor::Feedback param_motor_fric_[FRIC_NUM];

  Motor::Feedback param_trig_;

  Motor::MotorCmd cmd_fric_[FRIC_NUM] = {
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0},
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0},
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0},
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0}};
  Motor::MotorCmd cmd_trig_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 19.2032f,
                      .velocity = 0};

  // 添加线程和事件相关成员
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;
  LibXR::Event event_;

  Referee* ref_;

  uint8_t ui_step_ = 0;  // UI绘制步骤
  uint8_t ui_tick_ = 0;  // UI绘制计数器

  LibXR::MillisecondTimestamp last_wakeup_time_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  LibXR::Timer::TimerHandle
      ui_timer_handle_;  // 定时器句柄用于定期更新UI（2Hz）
  /*----------工具函数--------------------------------*/
  /**ui
   * @brief 更新电机反馈和状态量不同
   */
  void Update() {
    this->last_wakeup_ = LibXR::Timebase::GetMicroseconds();

    const float LAST_TRIG_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_trig_.abs_angle);
    for (int i = 0; i < FRIC_NUM; i++) {
      fric_motor_[i]->Update();
      param_motor_fric_[i] = fric_motor_[i]->GetFeedback();
    }
    motor_state_ = motor_trig_->Update();
    param_trig_ = motor_trig_->GetFeedback();
    const float DELTA_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_trig_.abs_angle) - LAST_TRIG_MOTOR_ANGLE;
    this->trig_angle_ += DELTA_MOTOR_ANGLE / trig_gear_ratio_;
  }

  /**
   * @brief 状态机与热量计算
   * @details 更新热量限制，更新拨弹状态机。
   */
  void Solve() {
    SoftStart();
    HeatLimit();
    UpdateTrigMode();
    UpdateFricTarget();
  }

  /**
   * @brief 控制输出
   * @details 拨弹控制、发弹检测和摩擦轮PID输出。
   */
  void Control() {
    if (motor_state_ == LibXR::ErrorCode::OK) {
      if (first_loading_) {
        FirstLoadingControl();
      } else {
        NormalFireControl();
      }
      real_launch_delay_ =
          (finish_fire_time_ - start_fire_time_).ToMillisecond();
      FricPidControl();
      TrigPidControl();
    } else {
      Reset();
    }
  }

  /**
   * @brief 设置发射器模式
   * @param mode 事件ID，对应 LauncherEvent
   */
  void SetMode(uint32_t mode) {
    launcher_state_ = static_cast<LauncherEvent>(mode);
  }

  /**
   * @brief 失控处理
   */
  void LostCtrl() {
    // 重置所有发射相关的状态变量到初始模式
    launcher_state_ = LauncherEvent::SET_FRICMODE_RELAX;
    trig_mode_ = TrigMode::RELAX;

    // 重置发射控制标志
    fire_flag_ = false;
    enable_fire_ = false;
    mark_launch_ = false;
    first_loading_ = true;
    press_continue_ = false;

    // 重置计数器
    fired_ = 0;
    delay_time_ = 0;

    // 重置时间戳
    fire_press_time_ = 0;
    start_fire_time_ = 0;
    finish_fire_time_ = 0;
    start_loading_time_ = 0;
    last_change_angle_time_ = 0;

    // 重置角度相关变量
    trig_zero_angle_ = 0.0f;
    trig_angle_ = 0.0f;
    trig_setpoint_angle_ = 0.0f;

    trig_output_ = 0.0f;

    // 重置速度目标值
    fric_target_rpm_[0] = 0.0f;
    fric_target_rpm_[1] = 0.0f;
    fric_target_rpm_[2] = 0.0f;
    fric_target_rpm_[3] = 0.0f;

    // 重置发射命令
    launcher_cmd_.isfire = false;
    last_fire_notify_ = false;

    // 重置延迟计算
    real_launch_delay_ = 0.0f;

    soft_start_finish_ = false;
  }

  /**
   * @brief 重置发射器状态
   * @details 将所有发射相关的状态变量、标志位、时间戳和角度值重置到初始状态。
   */
  void Reset() {
    first_loading_ = true;
    fire_flag_ = false;
    enable_fire_ = false;
    mark_launch_ = false;
    press_continue_ = false;

    fired_ = 0;

    start_fire_time_ = 0;
    finish_fire_time_ = 0;

    launcher_cmd_.isfire = false;
    last_fire_notify_ = false;

    trig_angle_ = 0.0f;
    trig_zero_angle_ = 0.0f;
    trig_setpoint_angle_ = 0.0f;

    delay_time_ = 0;

    soft_start_finish_ = false;

    for (int i = 0; i < FRIC_NUM; i++) {
      fric_motor_[i]->Relax();
    }
    motor_trig_->Relax();
  }

  /**
   * @brief 更新拨弹盘模式
   */
  void UpdateTrigMode() {
    LibXR::MillisecondTimestamp now_time = LibXR::Timebase::GetMilliseconds();

    if (launcher_state_ != LauncherEvent::SET_FRICMODE_RELAX) {
      if (launcher_cmd_.isfire && !last_fire_notify_) {
        fire_press_time_ = now_time;
        press_continue_ = false;
        trig_mode_ = TrigMode::SINGLE;
      } else if (launcher_cmd_.isfire && last_fire_notify_) {
        if (!press_continue_ && (now_time - fire_press_time_ > 200)) {
          press_continue_ = true;
        }
        if (press_continue_) {
          trig_mode_ = TrigMode::CONTINUE;
        }
      } else {
        trig_mode_ = TrigMode::SAFE;
        press_continue_ = false;
      }
    } else {
      trig_mode_ = TrigMode::RELAX;
    }

    last_fire_notify_ = launcher_cmd_.isfire;
  }

  /**
   * @brief 根据模式设置摩擦轮目标转速
   */
  void UpdateFricTarget() {
    switch (launcher_state_) {
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
        fric_target_rpm_[0] = 0.0f;
        fric_target_rpm_[1] = 0.0f;
        fric_target_rpm_[2] = 0.0f;
        fric_target_rpm_[3] = 0.0f;
        soft_start_finish_ = false;
        Reset();
        break;
      case LauncherEvent::SET_FRICMODE_READY:
        fric_target_rpm_[0] = param_fric_target_speed_[0];
        fric_target_rpm_[1] = param_fric_target_speed_[0];
        fric_target_rpm_[2] = param_fric_target_speed_[1];
        fric_target_rpm_[3] = param_fric_target_speed_[1];
        break;
      default:
        break;
    }
  }

  /**
   * @brief 首次发弹标定控制
   */
  void FirstLoadingControl() {
    if (trig_mode_ == TrigMode::SINGLE || trig_mode_ == TrigMode::CONTINUE) {
      fire_flag_ = true;
    }
    if (fire_flag_) {
      if (start_loading_time_ == 0) {
        start_loading_time_ = LibXR::Timebase::GetMilliseconds();
      }

      trig_setpoint_angle_ -= TRIG_LOADING_ANGLE_STEP;
      last_change_angle_time_ = LibXR::Timebase::GetMilliseconds();

      delay_time_++;
    }

    if (soft_start_finish_) {
      if (std::abs(param_motor_fric_[2].torque) > 0.05) {  // 发弹检测
        trig_zero_angle_ = trig_angle_;                    // 获取电机当前位置
        trig_setpoint_angle_ = trig_angle_ - TRIG_ZERO_ANGLE_OFFSET;  // 偏移量

        fire_flag_ = false;
        first_loading_ = false;
        fired_++;

        mark_launch_ = true;

        stuck = false;
      }
    }
  }

  /**
   * @brief 常规发弹逻辑
   */
  void NormalFireControl() {
    if (trig_mode_ == TrigMode::SINGLE) {
      mark_launch_ = false;
      if (!enable_fire_) {
        if (heat_ctrl_.available_shot) {
          trig_setpoint_angle_ -= static_cast<float>(LibXR::TWO_PI) /
                                  static_cast<float>(num_trig_tooth_);

          enable_fire_ = true;
          mark_launch_ = false;
          start_fire_time_ = LibXR::Timebase::GetMilliseconds();

          trig_mode_ = TrigMode::SAFE;
        }
      }
    }

    // 添加发射超时检测（超过100毫秒未检测到发弹则重置状态）
    if (start_fire_time_ > 0 && (now_ - start_fire_time_ > 100) &&
        !mark_launch_) {
      fire_flag_ = false;
      enable_fire_ = false;
      start_fire_time_ = now_;
    }

    if (!mark_launch_) {  // 发弹状态检测
      if (std::abs(param_motor_fric_[2].torque) > 0.05) {
        fire_flag_ = false;

        fired_++;

        mark_launch_ = true;
        enable_fire_ = false;
        finish_fire_time_ = LibXR::Timebase::GetMilliseconds();
      }
    }
  }

  /**
   * @brief 摩擦轮PID控制输出
   */
  void FricPidControl() {
    if (speed_sync_) {
      float fric_outputs[FRIC_NUM];
      for (int i = 0; i < FRIC_NUM; i++) {
        fric_outputs[i] = fric_speed_pid_[i].Calculate(
            fric_target_rpm_[i], param_motor_fric_[i].velocity, dt_);
      }
      // 前摩擦轮同步控制 (0-1)
      float front_speed_diff =
          param_motor_fric_[0].velocity - param_motor_fric_[1].velocity;
      float front_sync_output =
          fric_sync_pid_front_.Calculate(0.0f, front_speed_diff, dt_);

      // 后摩擦轮同步控制 (2-3)
      float back_speed_diff =
          param_motor_fric_[2].velocity - param_motor_fric_[3].velocity;
      float back_sync_output =
          fric_sync_pid_back_.Calculate(0.0f, back_speed_diff, dt_);

      // 将同步输出叠加到各摩擦轮
      cmd_fric_[0].velocity =
          fric_outputs[0] + front_sync_output;  // 前左 + 同步输出
      cmd_fric_[1].velocity =
          fric_outputs[1] - front_sync_output;  // 前右 - 同步输出
      cmd_fric_[2].velocity =
          fric_outputs[2] + back_sync_output;  // 后左 + 同步输出
      cmd_fric_[3].velocity =
          fric_outputs[3] - back_sync_output;  // 后右 - 同步输出
    } else {
      for (int i = 0; i < FRIC_NUM; i++) {
        cmd_fric_[i].velocity = fric_speed_pid_[i].Calculate(
            fric_target_rpm_[i], param_motor_fric_[i].velocity, dt_);
      }
    }

    for (int i = 0; i < FRIC_NUM; i++) {
      fric_motor_[i]->Control(cmd_fric_[i]);
    }
  }

  /**
   * @brief 拨弹PID控制输出
   */
  void TrigPidControl() {
    trig_setpoint_speed_ =
        trig_angle_pid_.Calculate(trig_setpoint_angle_, trig_angle_, dt_);

    trig_output_ = trig_speed_pid_.Calculate(trig_setpoint_speed_,
                                             param_trig_.velocity, dt_);
    switch (trig_mode_) {
      case TrigMode::RELAX:
        cmd_trig_.velocity = 0;
        break;
      case TrigMode::SAFE:
      case TrigMode::SINGLE:
      case TrigMode::CONTINUE:
        cmd_trig_.velocity = trig_output_;
        break;
      default:
        break;
    }
    motor_trig_->Control(cmd_trig_);
  }

  /**
   * @brief 摩擦轮软启动控制
   * @details 在摩擦轮启动初期限制 PID 输出，防止电流冲击；
   *          当实际转速达到设定值后解除输出限制。
   */
  void SoftStart() {
    if (!soft_start_finish_) {
      for (LibXR::PID<float>& i : fric_speed_pid_) {
        i.SetOutLimit(0.1f);
      }

      if (fric_motor_[0]->GetFeedback().velocity >
          param_fric_target_speed_[0]) {
        soft_start_finish_ = true;
      }
    } else {
      for (LibXR::PID<float>& i : fric_speed_pid_) {
        i.SetOutLimit(1.0f);
      }
    }
  }

  /**
   * @brief 热量限制计算
   */
  void HeatLimit() {
    heat_ctrl_.heat_limit = ref_data_.rs.shooter_heat_limit;
    heat_ctrl_.heat_increase = 100.0f;
    heat_ctrl_.cooling_rate = ref_data_.rs.shooter_cooling_value;
    if (fired_ >= 1) {
      heat_ctrl_.heat += heat_ctrl_.heat_increase;
      fired_ = 0;
    }
    heat_ctrl_.heat -=
        heat_ctrl_.cooling_rate / (1 / dt_);  // 每个控制周期的冷却恢复
    heat_ctrl_.heat = std::max(heat_ctrl_.heat, 0.0f);
    float available_float =
        (this->heat_ctrl_.heat_limit - this->heat_ctrl_.heat) /
        this->heat_ctrl_.heat_increase;
    heat_ctrl_.available_shot = static_cast<uint32_t>(available_float);
  }

  CMD::LauncherCMD launcher_cmd_;  // NOLINT
  Referee::LauncherPack ref_data_{};

  void DrawUI() {
    uint16_t robot_id = ref_->GetRobotID();
    uint16_t client_id = ref_->GetClientID(robot_id);

    // 首次绘制使用ADD，后续使用MODIFY
    Referee::UIFigureOp ADD_OP = Referee::UIFigureOp::UI_OP_MODIFY;
    if (this->ui_tick_ % 5 == 0) {
      ADD_OP = Referee::UIFigureOp::UI_OP_ADD;
    }

    // 摩擦轮颜色：根据转速判断状态（>5000 RPM为青色，否则为橙色）
    auto fric_color_0 = (fabsf(param_motor_fric_[0].velocity) > 3500.0f)
                            ? Referee::UIColor::UI_COLOR_CYAN
                            : Referee::UIColor::UI_COLOR_ORANGE;
    auto fric_color_1 = (fabsf(param_motor_fric_[1].velocity) > 3500.0f)
                            ? Referee::UIColor::UI_COLOR_CYAN
                            : Referee::UIColor::UI_COLOR_ORANGE;
    auto fric_color_2 = (fabsf(param_motor_fric_[2].velocity) > 2200.0f)
                            ? Referee::UIColor::UI_COLOR_CYAN
                            : Referee::UIColor::UI_COLOR_ORANGE;
    auto fric_color_3 = (fabsf(param_motor_fric_[3].velocity) > 2200.0f)
                            ? Referee::UIColor::UI_COLOR_CYAN
                            : Referee::UIColor::UI_COLOR_ORANGE;

    // 拨弹盘位置计算
    float trig_pos =
        LibXR::CycleValue<float>(-this->trig_angle_) / 6.2832f * 360.0f;
    uint16_t arc_mid = static_cast<uint16_t>(trig_pos);
    uint16_t arc_start = static_cast<uint16_t>((arc_mid + 330) % 360);
    uint16_t arc_end = static_cast<uint16_t>((arc_mid + 30) % 360);
    if (arc_start >= arc_end) arc_end += 360;

    // 拨弹盘颜色：根据模式判断（校准模式为橙色，其他为青色）
    auto trig_color = first_loading_ ? Referee::UIColor::UI_COLOR_ORANGE
                                     : Referee::UIColor::UI_COLOR_CYAN;

    switch (ui_step_) {
      case 0: {
        // 绘制前摩擦轮状态
        Referee::UIFigure2 fig_front{};
        ref_->FillCircle(fig_front.interaction_figure[0], "lfl", ADD_OP,
                         UI_HERO_LAUNCHER_LAYER, fric_color_0, 4, 1600, 820,
                         25);
        ref_->FillCircle(fig_front.interaction_figure[1], "lfr", ADD_OP,
                         UI_HERO_LAUNCHER_LAYER, fric_color_1, 4, 1680, 820,
                         25);
        ref_->SendUIFigure2(robot_id, client_id, fig_front);
        this->ui_tick_ += 1;
        break;
      }
      case 1: {
        // 绘制后摩擦轮状态
        Referee::UIFigure2 fig_back{};
        ref_->FillCircle(fig_back.interaction_figure[0], "lbl", ADD_OP,
                         UI_HERO_LAUNCHER_LAYER, fric_color_2, 4, 1600, 750,
                         25);
        ref_->FillCircle(fig_back.interaction_figure[1], "lbr", ADD_OP,
                         UI_HERO_LAUNCHER_LAYER, fric_color_3, 4, 1680, 750,
                         25);
        ref_->SendUIFigure2(robot_id, client_id, fig_back);
        break;
      }
      case 2: {
        // 绘制拨弹盘位置
        Referee::UIFigure fig_trig{};
        ref_->FillArc(fig_trig, "lta", ADD_OP, UI_HERO_LAUNCHER_LAYER,
                      trig_color, 3, 1640, 640, arc_start, arc_end, 80, 80);
        ref_->SendUIFigure(robot_id, client_id, fig_trig);
        break;
      }
      case 3: {
        // 竖直画线 - 主瞄准线
        Referee::UIFigure fig_line{};
        ref_->FillLine(fig_line, "ctrln", ADD_OP, UI_HERO_LAUNCHER_LAYER,
                       Referee::UIColor::UI_COLOR_WHITE, 2, 960, 150, 960, 900);
        ref_->SendUIFigure(robot_id, client_id, fig_line);
        break;
      }
      case 4: {
        // 水平瞄准线 - 下方第一条

        Referee::UIFigure fig_bot2{};
        ref_->FillLine(fig_bot2, "hlb1", ADD_OP, UI_HERO_LAUNCHER_LAYER,
                       Referee::UIColor::UI_COLOR_WHITE, 2, 940, 300, 980, 300);
        ref_->SendUIFigure(robot_id, client_id, fig_bot2);

        break;
      }
      default:
        break;
    }
    this->ui_step_ = (this->ui_step_ + 1) % 5;  // 更新为7个case (0-6)
  }
};
