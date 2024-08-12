/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MulticopterRateControl.hpp" // 包含 MulticopterRateControl 类的头文件

#include <drivers/drv_hrt.h> // 包含高分辨率定时器的驱动程序头文件
#include <circuit_breaker/circuit_breaker.h> // 包含电路断路器的头文件
#include <mathlib/math/Limits.hpp> // 包含数学库中的限制函数
#include <mathlib/math/Functions.hpp> // 包含数学库中的函数
#include <px4_platform_common/events.h> // 包含 PX4 平台通用事件的头文件

using namespace matrix; // 使用 matrix 命名空间
using namespace time_literals; // 使用时间字面量命名空间
using math::radians; // 使用 math 库中的 radians 函数

// MulticopterRateControl 类的构造函数，接受一个布尔参数 vtol
MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr), // 初始化 ModuleParams 基类
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl), // 初始化 WorkItem 基类
	// 根据 vtol 参数选择合适的 ORB 话题进行发布
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")) // 分配性能计数器
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING; // 设置车辆类型为旋翼机

	parameters_updated(); // 更新参数
	_controller_status_pub.advertise(); // 广播控制器状态
}

// MulticopterRateControl 类的析构函数
MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf); // 释放性能计数器
}

// 初始化函数
bool
MulticopterRateControl::init()
{
	// 注册车辆角速度的回调函数
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed"); // 如果注册失败，打印错误信息
		return false; // 返回 false 表示初始化失败
	}

	return true; // 返回 true 表示初始化成功
}

// 更新参数函数
void
MulticopterRateControl::parameters_updated()
{
	// 速率控制参数
	// 控制器增益 K 用于将并联形式 (P + I/s + sD) 转换为理想形式 (K * [1 + 1/sTi + sTd])
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	// 设置速率控制器的增益
	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	// 设置积分器限制
	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	// 设置前馈增益
	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));

	// 手动速率控制特技模式下的速率限制
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) { // 如果需要退出
		_vehicle_angular_velocity_sub.unregisterCallback(); // 注销车辆角速度的回调函数
		exit_and_cleanup(); // 退出并清理
		return; // 返回
	}

	perf_begin(_loop_perf); // 开始性能计数

	// 检查参数是否已更改
	if (_parameter_update_sub.updated()) {
		// 清除更新标志
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update); // 复制参数更新

		updateParams(); // 更新参数
		parameters_updated(); // 参数更新后的处理
	}

	/* 在陀螺仪数据变化时运行控制器 */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) { // 更新车辆角速度数据

		const hrt_abstime now = angular_velocity.timestamp_sample; // 获取当前时间戳

		// 防止时间间隔过小 (< 0.125ms) 或过大 (> 20ms)
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now; // 更新上次运行时间

		const Vector3f rates{angular_velocity.xyz}; // 获取角速度
		const Vector3f angular_accel{angular_velocity.xyz_derivative}; // 获取角加速度

		/* 检查其他话题的更新 */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode); // 更新车辆控制模式

		if (_vehicle_land_detected_sub.updated()) { // 如果检测到车辆着陆状态更新
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) { // 复制着陆检测数据
				_landed = vehicle_land_detected.landed; // 更新着陆状态
				_maybe_landed = vehicle_land_detected.maybe_landed; // 更新可能着陆状态
			}
		}

		_vehicle_status_sub.update(&_vehicle_status); // 更新车辆状态

		// 使用速率设定点话题
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// 从遥控器生成速率设定点
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) { // 更新手动控制设定点
				// 手动速率控制 - 特技模式
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max); // 计算速率设定点
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f; // 计算推力设定点
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f; // 设置推力设定点的 x 和 y 分量为 0

				// 发布速率设定点
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint); // 发布速率设定点
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) { // 更新速率设定点
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) { // 复制速率设定点数据
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body); // 更新推力设定点
			}
		}

		// 运行速率控制器
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// 如果未武装或车辆类型不是旋翼机，则重置积分器
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// 从控制分配反馈中更新饱和状态
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) { // 更新控制分配器状态
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) { // 如果未达到扭矩设定点
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true; // 正饱和

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true; // 负饱和
						}
					}
				}

				// TODO: 直接发送未分配的值以更好地防止积分器饱和
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative); // 设置饱和状态
			}

			// 运行速率控制器
			const Vector3f att_control = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

			// 发布速率控制器状态
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status); // 发布控制器状态

			// 发布推力和扭矩设定点
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.f;

			// 如果启用了电池状态缩放，则按电池状态缩放设定点
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) { // 更新电池状态
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale; // 更新电池状态缩放比例
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint); // 发布推力设定点

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint); // 发布扭矩设定点

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt); // 更新执行器控制状态

		}
	}

	perf_end(_loop_perf); // 结束性能计数
}
void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	// 更新控制能量
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt; // 更新能量积分时间

	// 每 500 毫秒更新一次执行器控制状态
	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status; // 创建执行器控制状态对象
		status.timestamp = vehicle_torque_setpoint.timestamp; // 设置时间戳

		// 计算控制能量并重置
		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time; // 计算控制功率
			_control_energy[i] = 0.f; // 重置控制能量
		}

		_actuator_controls_status_pub.publish(status); // 发布执行器控制状态
		_energy_integration_time = 0.f; // 重置能量积分时间
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false; // 默认非 VTOL 模式

	if (argc > 1) { // 如果有额外的命令行参数
		if (strcmp(argv[1], "vtol") == 0) { // 如果参数是 "vtol"
			vtol = true; // 设置 VTOL 模式
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol); // 创建 MulticopterRateControl 实例

	if (instance) {
		_object.store(instance); // 存储实例对象
		_task_id = task_id_is_work_queue; // 设置任务 ID

		if (instance->init()) { // 初始化实例
			return PX4_OK; // 返回 PX4_OK 表示成功
		}

	} else {
		PX4_ERR("alloc failed"); // 分配失败，打印错误信息
	}

	delete instance; // 删除实例
	_object.store(nullptr); // 清空实例对象
	_task_id = -1; // 重置任务 ID

	return PX4_ERROR; // 返回 PX4_ERROR 表示失败
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command"); // 打印未知命令的用法
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason); // 打印警告信息
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller"); // 打印模块名称和类型
    PRINT_MODULE_USAGE_COMMAND("start"); // 打印 start 命令
    PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true); // 打印 vtol 参数
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); // 打印默认命令

    return 0; // 返回 0 表示成功
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
    return MulticopterRateControl::main(argc, argv); // 调用 MulticopterRateControl 的 main 函数
}
