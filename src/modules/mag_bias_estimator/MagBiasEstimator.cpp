/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "MagBiasEstimator.hpp"

using namespace time_literals;
using matrix::Vector3f;

namespace mag_bias_estimator
{

// 构造函数
MagBiasEstimator::MagBiasEstimator() :
	ModuleParams(nullptr), // 初始化基类 ModuleParams
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) // 初始化基类 ScheduledWorkItem
{
	// 广播磁偏置估计的发布者
	_magnetometer_bias_estimate_pub.advertise();
}

// 析构函数
MagBiasEstimator::~MagBiasEstimator()
{
	// 释放性能计数器资源
	perf_free(_cycle_perf);
}

// 任务生成函数
int MagBiasEstimator::task_spawn(int argc, char *argv[])
{
	// 创建 MagBiasEstimator 对象
	MagBiasEstimator *obj = new MagBiasEstimator();

	// 检查对象是否创建成功
	if (!obj) {
		// 如果创建失败，打印错误信息并返回 -1
		PX4_ERR("alloc failed");
		return -1;
	}

	// 存储对象指针
	_object.store(obj);
	// 设置任务 ID 为工作队列任务 ID
	_task_id = task_id_is_work_queue;

	// 调度一个周期来启动任务
	obj->start();

	// 返回 0 表示成功
	return 0;
}

// 启动函数
void MagBiasEstimator::start()
{
	// 以 50 Hz 的频率调度任务
	ScheduleOnInterval(20_ms); // 50 Hz
}
} // namespace mag_bias_estimator

// MagBiasEstimator 类的 Run 方法
void MagBiasEstimator::Run()
{
	// 检查是否应该退出
	if (should_exit()) {
		// 清除调度
		ScheduleClear();
		// 退出并清理
		exit_and_cleanup();
	}

	// 检查 vehicle_status 是否更新
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		// 复制最新的 vehicle_status 数据
		if (_vehicle_status_sub.copy(&vehicle_status)) {
			// 如果 arming_state 发生变化
			if (_arming_state != vehicle_status.arming_state) {
				_arming_state = vehicle_status.arming_state;

				// 在任何 arming 状态变化时重置
				for (auto &reset : _reset_field_estimator) {
					reset = true;
				}

				// 如果系统已武装
				if (_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
					// 调度任务以 1 秒的间隔运行
					ScheduleOnInterval(1_s);

				} else {
					// 恢复到 50 Hz 调度
					ScheduleOnInterval(20_ms);
				}
			}

			// 检查系统是否在校准中
			bool system_calibrating = vehicle_status.calibration_enabled;

			// 如果校准状态发生变化
			if (system_calibrating != _system_calibrating) {
				_system_calibrating = system_calibrating;

				// 重置所有磁场估计器
				for (auto &reset : _reset_field_estimator) {
					reset = true;
				}
			}
		}
	}

	// 仅在未武装时运行
	if (_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}

	// 检查参数是否更新
	if (_parameter_update_sub.updated()) {
		// 清除更新标志
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// 从存储中更新参数
		updateParams();

		// 更新每个磁传感器的参数
		for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
			const auto calibration_count = _calibration[mag_index].calibration_count();
			_calibration[mag_index].ParametersUpdate();

			// 如果校准计数发生变化，重置磁场估计器
			if (calibration_count != _calibration[mag_index].calibration_count()) {
				_reset_field_estimator[mag_index] = true;
			}

			// 设置学习增益
			_bias_estimator[mag_index].setLearningGain(_param_mbe_learn_gain.get());
		}
	}

	// 在常规传感器校准期间不执行任何操作
	if (_system_calibrating) {
		return;
	}

	// 开始性能计数
	perf_begin(_cycle_perf);

	// 假设在两个磁传感器样本期间角速度恒定
	vehicle_angular_velocity_s vehicle_angular_velocity;

	// 更新车辆角速度
	if (_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity)) {

		const Vector3f angular_velocity{vehicle_angular_velocity.xyz};

		bool updated = false;

		// 遍历每个磁传感器
		for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
			sensor_mag_s sensor_mag;

			// 更新磁传感器数据
			while (_sensor_mag_subs[mag_index].update(&sensor_mag)) {

				updated = true;

				// 应用现有的磁传感器校准
				_calibration[mag_index].set_device_id(sensor_mag.device_id);

				const Vector3f mag_calibrated = _calibration[mag_index].Correct(Vector3f{sensor_mag.x, sensor_mag.y, sensor_mag.z});

				float dt = (sensor_mag.timestamp_sample - _timestamp_last_update[mag_index]) * 1e-6f;
				_timestamp_last_update[mag_index] = sensor_mag.timestamp_sample;

				// 如果时间间隔不合理，重置磁场估计器
				if (dt < 0.001f || dt > 0.2f) {
					_reset_field_estimator[mag_index] = true;
				}

				// 如果需要重置磁场估计器
				if (_reset_field_estimator[mag_index]) {
					// 重置
					_bias_estimator[mag_index].setBias(Vector3f{});
					_bias_estimator[mag_index].setField(mag_calibrated);

					_reset_field_estimator[mag_index] = false;
					_valid[mag_index] = false;
					_time_valid[mag_index] = 0;

				} else {
					updated = true;

					const Vector3f bias_prev = _bias_estimator[mag_index].getBias();

					// 更新偏置估计
					_bias_estimator[mag_index].updateEstimate(angular_velocity, mag_calibrated, dt);

					const Vector3f &bias = _bias_estimator[mag_index].getBias();
					const Vector3f bias_rate = (bias - bias_prev) / dt;

					// 如果偏置无效或超过阈值，重置磁场估计器
					if (!bias.isAllFinite() || bias.longerThan(5.f)) {
						_reset_field_estimator[mag_index] = true;
						_valid[mag_index] = false;
						_time_valid[mag_index] = 0;

					} else {
						// 计算适应度
						Vector3f fitness{
							fabsf(angular_velocity(0)) / fmaxf(fabsf(bias_rate(1)) + fabsf(bias_rate(2)), 0.02f),
							fabsf(angular_velocity(1)) / fmaxf(fabsf(bias_rate(0)) + fabsf(bias_rate(2)), 0.02f),
							fabsf(angular_velocity(2)) / fmaxf(fabsf(bias_rate(0)) + fabsf(bias_rate(1)), 0.02f)
						};

						const bool bias_significant = bias.longerThan(0.04f);
						const bool has_converged = fitness(0) > 20.f || fitness(1) > 20.f || fitness(2) > 20.f;

						// 如果偏置显著且已收敛，标记为有效
						if (bias_significant && has_converged) {
							if (!_valid[mag_index]) {
								_time_valid[mag_index] = hrt_absolute_time();
							}

							_valid[mag_index] = true;
						}
					}
				}
			}
		}

		// 如果有更新，发布磁偏置估计
		if (updated) {
			publishMagBiasEstimate();
		}
	}

	// 结束性能计数
	perf_end(_cycle_perf);
}
void MagBiasEstimator::publishMagBiasEstimate()
{
	// 创建一个磁偏置估计的消息对象
	magnetometer_bias_estimate_s mag_bias_est{};

	// 遍历每个磁传感器
	for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
		// 获取当前磁传感器的偏置
		const Vector3f &bias = _bias_estimator[mag_index].getBias();
		// 将偏置值存储到消息对象中
		mag_bias_est.bias_x[mag_index] = bias(0);
		mag_bias_est.bias_y[mag_index] = bias(1);
		mag_bias_est.bias_z[mag_index] = bias(2);

		// 设置偏置的有效性
		mag_bias_est.valid[mag_index] = _valid[mag_index];

		// 如果偏置有效
		if (_valid[mag_index]) {
			mag_bias_est.valid[mag_index] = true;
			// 检查偏置是否稳定（超过30秒）
			mag_bias_est.stable[mag_index] = (hrt_elapsed_time(&_time_valid[mag_index]) > 30_s);
		}
	}

	// 设置消息的时间戳
	mag_bias_est.timestamp = hrt_absolute_time();
	// 发布磁偏置估计消息
	_magnetometer_bias_estimate_pub.publish(mag_bias_est);
}

int MagBiasEstimator::print_status()
{
	// 遍历每个磁传感器
	for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
		// 如果磁传感器已校准
		if (_calibration[mag_index].device_id() != 0) {
			// 打印校准状态
			_calibration[mag_index].PrintStatus();

			// 获取当前磁传感器的偏置
			const Vector3f &bias = _bias_estimator[mag_index].getBias();

			// 打印偏置信息
			PX4_INFO("%d (%" PRIu32 ") bias: [% 05.3f % 05.3f % 05.3f]",
				 mag_index, _calibration[mag_index].device_id(),
				 (double)bias(0),
				 (double)bias(1),
				 (double)bias(2));
		}
	}

	return 0;
}

int MagBiasEstimator::print_usage(const char *reason)
{
	// 如果有原因，打印错误信息
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	// 打印模块描述
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Online magnetometer bias estimator.
)DESCR_STR");

    // 打印模块使用信息
    PRINT_MODULE_USAGE_NAME("mag_bias_estimator", "system");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

// 主函数，用于启动磁偏置估计器
extern "C" __EXPORT int mag_bias_estimator_main(int argc, char *argv[])
{
    return MagBiasEstimator::main(argc, argv);
}

} // namespace mag_bias_estimator
