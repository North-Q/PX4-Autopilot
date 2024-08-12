/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
/*
 * @file attitude_estimator_q_main.cpp
 *
 * 姿态估计器（基于四元数）
 *
 * @作者 Anton Babushkin <anton.babushkin@me.com>
 */

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::wrap_pi;

using namespace time_literals;

// 姿态估计器类，继承自 ModuleBase、ModuleParams 和 px4::WorkItem
class AttitudeEstimatorQ : public ModuleBase<AttitudeEstimatorQ>, public ModuleParams, public px4::WorkItem
{
public:

	AttitudeEstimatorQ(); // 构造函数
	~AttitudeEstimatorQ() override = default; // 析构函数

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]); // 任务生成函数

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]); // 自定义命令函数

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr); // 打印用法函数

	bool init(); // 初始化函数

private:

	void Run() override; // 运行函数，重写自 px4::WorkItem

	bool init_attitude_q(); // 初始化姿态四元数

	void update_gps_position(); // 更新 GPS 位置

	void update_magnetometer(); // 更新磁力计

	void update_motion_capture_odometry(); // 更新运动捕捉里程计

	void update_sensors(); // 更新传感器

	void update_visual_odometry(); // 更新视觉里程计

	void update_vehicle_attitude(); // 更新车辆姿态

	void update_vehicle_local_position(); // 更新车辆本地位置

	void update_parameters(bool force = false); // 更新参数

	bool update(float dt); // 更新函数

	// 立即更新磁偏角（单位：弧度），改变偏航旋转
	void update_mag_declination(float new_declination);

	const float _eo_max_std_dev = 100.0f;           /**< 估计方向的最大允许标准偏差 */
	const float _dt_min = 0.00001f; // 最小时间间隔
	const float _dt_max = 0.02f; // 最大时间间隔

	uORB::SubscriptionCallbackWorkItem _sensors_sub{this, ORB_ID(sensor_combined)}; // 传感器订阅回调

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // 参数更新订阅，间隔 1 秒

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)}; // 车辆姿态订阅
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)}; // 车辆 GPS 位置订阅
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)}; // 车辆本地位置订阅
	uORB::Subscription _vehicle_magnetometer_sub{ORB_ID(vehicle_magnetometer)}; // 车辆磁力计订阅
	uORB::Subscription _vehicle_mocap_odometry_sub{ORB_ID(vehicle_mocap_odometry)}; // 车辆运动捕捉里程计订阅
	uORB::Subscription _vehicle_visual_odometry_sub{ORB_ID(vehicle_visual_odometry)}; // 车辆视觉里程计订阅

	uORB::Publication<vehicle_attitude_s> _vehicle_attitude_pub{ORB_ID(vehicle_attitude)}; // 车辆姿态发布

	Vector3f    _accel{}; // 加速度
	Vector3f    _gyro{}; // 陀螺仪
	Vector3f    _gyro_bias{}; // 陀螺仪偏差
	Vector3f    _rates{}; // 速率

	Vector3f    _mag{}; // 磁力计
	Vector3f    _mocap_hdg{}; // 运动捕捉方向
	Vector3f    _vision_hdg{}; // 视觉方向

	Vector3f    _pos_acc{}; // 位置加速度
	Vector3f    _vel_prev{}; // 之前的速度

	Quatf       _q{}; // 四元数

	hrt_abstime _imu_timestamp{}; // IMU 时间戳
	hrt_abstime _imu_prev_timestamp{}; // 之前的 IMU 时间戳
	hrt_abstime _vel_prev_timestamp{}; // 之前的速度时间戳

	float       _bias_max{}; // 最大偏差
	float       _mag_decl{}; // 磁偏角

	bool        _data_good{false}; // 数据是否良好
	bool        _ext_hdg_good{false}; // 外部方向是否良好
	bool        _initialized{false}; // 是否已初始化

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ATT_W_ACC>)       _param_att_w_acc, // 加速度权重参数
		(ParamFloat<px4::params::ATT_W_MAG>)       _param_att_w_mag, // 磁力计权重参数
		(ParamFloat<px4::params::ATT_W_EXT_HDG>)   _param_att_w_ext_hdg, // 外部方向权重参数
		(ParamFloat<px4::params::ATT_W_GYRO_BIAS>) _param_att_w_gyro_bias, // 陀螺仪偏差权重参数
		(ParamFloat<px4::params::ATT_MAG_DECL>)    _param_att_mag_decl, // 磁偏角参数
		(ParamInt<px4::params::ATT_MAG_DECL_A>)    _param_att_mag_decl_a, // 磁偏角自动参数
		(ParamInt<px4::params::ATT_EXT_HDG_M>)     _param_att_ext_hdg_m, // 外部方向模式参数
		(ParamInt<px4::params::ATT_ACC_COMP>)      _param_att_acc_comp, // 加速度补偿参数
		(ParamFloat<px4::params::ATT_BIAS_MAX>)    _param_att_bias_mas, // 最大偏差参数
		(ParamInt<px4::params::SYS_HAS_MAG>)       _param_sys_has_mag // 系统是否有磁力计参数
	)
};
AttitudeEstimatorQ::AttitudeEstimatorQ() :
	ModuleParams(nullptr), // 初始化基类 ModuleParams
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) // 初始化基类 WorkItem
{
	update_parameters(true); // 强制更新参数
}

bool AttitudeEstimatorQ::init()
{
	if (!_sensors_sub.registerCallback()) { // 注册传感器订阅回调
		PX4_ERR("callback registration failed"); // 如果注册失败，打印错误信息
		return false; // 返回 false 表示初始化失败
	}

	return true; // 返回 true 表示初始化成功
}

void AttitudeEstimatorQ::Run()
{
	if (should_exit()) { // 检查是否应该退出
		_sensors_sub.unregisterCallback(); // 注销传感器订阅回调
		exit_and_cleanup(); // 退出并清理
		return;
	}

	if (_sensors_sub.updated()) { // 检查传感器数据是否更新
		_data_good = true; // 数据良好
		_ext_hdg_good = false; // 外部方向数据不良好

		update_parameters(); // 更新参数
		update_sensors(); // 更新传感器数据
		update_magnetometer(); // 更新磁力计数据
		update_visual_odometry(); // 更新视觉里程计数据
		update_motion_capture_odometry(); // 更新运动捕捉里程计数据
		update_gps_position(); // 更新 GPS 位置数据
		update_vehicle_local_position(); // 更新车辆本地位置数据
		update_vehicle_attitude(); // 更新车辆姿态数据
	}
}

void AttitudeEstimatorQ::update_gps_position()
{
	if (_vehicle_gps_position_sub.updated()) { // 检查 GPS 位置数据是否更新
		sensor_gps_s gps; // 定义 GPS 数据结构

		if (_vehicle_gps_position_sub.update(&gps)) { // 更新 GPS 数据
			if (_param_att_mag_decl_a.get() && (gps.eph < 20.0f)) {
				// 如果启用了自动磁偏角并且 GPS 精度足够高，则自动设置磁偏角
				update_mag_declination(get_mag_declination_radians(gps.lat, gps.lon));
			}
		}
	}
}

void AttitudeEstimatorQ::update_magnetometer()
{
	// 更新磁力计数据
	if (_vehicle_magnetometer_sub.updated()) { // 检查磁力计数据是否更新
		vehicle_magnetometer_s magnetometer; // 定义磁力计数据结构

		if (_vehicle_magnetometer_sub.update(&magnetometer)) { // 更新磁力计数据
			_mag(0) = magnetometer.magnetometer_ga[0]; // 更新磁力计 X 轴数据
			_mag(1) = magnetometer.magnetometer_ga[1]; // 更新磁力计 Y 轴数据
			_mag(2) = magnetometer.magnetometer_ga[2]; // 更新磁力计 Z 轴数据

			if (_mag.length() < 0.01f) { // 检查磁力计数据是否有效
				PX4_ERR("degenerate mag!"); // 如果无效，打印错误信息
				return;
			}
		}
	}
}

void AttitudeEstimatorQ::update_motion_capture_odometry()
{
	if (_vehicle_mocap_odometry_sub.updated()) { // 检查运动捕捉里程计数据是否更新
		vehicle_odometry_s mocap; // 定义运动捕捉里程计数据结构

		if (_vehicle_mocap_odometry_sub.update(&mocap)) { // 更新运动捕捉里程计数据
			// 验证运动捕捉姿态数据
			bool mocap_att_valid = PX4_ISFINITE(mocap.q[0])
					       && (PX4_ISFINITE(mocap.orientation_variance[0]) ? sqrtf(fmaxf(
							       mocap.orientation_variance[0],
							       fmaxf(mocap.orientation_variance[1],
									       mocap.orientation_variance[2]))) <= _eo_max_std_dev : true);

			if (mocap_att_valid) { // 如果运动捕捉姿态数据有效
				Dcmf Rmoc = Quatf(mocap.q); // 将四元数转换为方向余弦矩阵
				Vector3f v(1.0f, 0.0f, 0.4f); // 定义一个向量

				// Rmoc 是 Rwr（相对于世界的机器人），而 v 是相对于世界的。
				// 因此 Rmoc 必须转置为 (Rwr)' * Vw
				// Rrw * Vw = vn。这样我们就有了一致性
				_mocap_hdg = Rmoc.transpose() * v; // 计算运动捕捉方向

				// 运动捕捉外部方向使用（ATT_EXT_HDG_M 2）
				if (_param_att_ext_hdg_m.get() == 2) {
					// 检查数据是否超时
					_ext_hdg_good = mocap.timestamp_sample > 0 && (hrt_elapsed_time(&mocap.timestamp_sample) < 500000);
				}
			}
		}
	}
}

void AttitudeEstimatorQ::update_sensors()
{
	sensor_combined_s sensors; // 定义传感器数据结构

	if (_sensors_sub.update(&sensors)) { // 更新传感器数据
		// 使用最近的传感器数据更新验证器
		if (sensors.timestamp > 0) {
			_imu_timestamp = sensors.timestamp; // 更新 IMU 时间戳
			_gyro(0) = sensors.gyro_rad[0]; // 更新陀螺仪 X 轴数据
			_gyro(1) = sensors.gyro_rad[1]; // 更新陀螺仪 Y 轴数据
			_gyro(2) = sensors.gyro_rad[2]; // 更新陀螺仪 Z 轴数据
		}

		if (sensors.accelerometer_timestamp_relative != sensor_combined_s::RELATIVE_TIMESTAMP_INVALID) {
			_accel(0) = sensors.accelerometer_m_s2[0]; // 更新加速度计 X 轴数据
			_accel(1) = sensors.accelerometer_m_s2[1]; // 更新加速度计 Y 轴数据
			_accel(2) = sensors.accelerometer_m_s2[2]; // 更新加速度计 Z 轴数据

			if (_accel.length() < 0.01f) { // 检查加速度计数据是否有效
				PX4_ERR("degenerate accel!"); // 如果无效，打印错误信息
				return;
			}
		}
	}
}
void AttitudeEstimatorQ::update_vehicle_attitude()
{
	// 从上一次迭代开始的时间
	hrt_abstime now = hrt_absolute_time();
	// 计算时间间隔，并将其限制在最小和最大值之间
	const float dt = math::constrain((now - _imu_prev_timestamp) / 1e6f, _dt_min, _dt_max);
	_imu_prev_timestamp = now;

	// 如果更新成功
	if (update(dt)) {
		vehicle_attitude_s vehicle_attitude{};
		vehicle_attitude.timestamp_sample = _imu_timestamp; // 设置时间戳样本
		_q.copyTo(vehicle_attitude.q); // 将四元数复制到车辆姿态

		/* 这里不使用实例计数 */
		vehicle_attitude.timestamp = hrt_absolute_time(); // 设置当前时间戳
		_vehicle_attitude_pub.publish(vehicle_attitude); // 发布车辆姿态
	}
}

void AttitudeEstimatorQ::update_vehicle_local_position()
{
	if (_vehicle_local_position_sub.updated()) { // 检查本地位置数据是否更新
		vehicle_local_position_s lpos;

		if (_vehicle_local_position_sub.update(&lpos)) { // 更新本地位置数据

			// 如果启用了加速度补偿且位置数据有效
			if (_param_att_acc_comp.get() && (hrt_elapsed_time(&lpos.timestamp) < 20_ms)
			    && lpos.v_xy_valid && lpos.v_z_valid && (lpos.eph < 5.0f) && _initialized) {

				/* 位置数据有效 */
				const Vector3f vel(lpos.vx, lpos.vy, lpos.vz); // 获取速度向量

				/* 速度已更新 */
				if (_vel_prev_timestamp != 0 && lpos.timestamp != _vel_prev_timestamp) {
					float vel_dt = (lpos.timestamp - _vel_prev_timestamp) / 1e6f;
					/* 计算机体坐标系下的加速度 */
					_pos_acc = _q.rotateVectorInverse((vel - _vel_prev) / vel_dt);
				}

				_vel_prev_timestamp = lpos.timestamp; // 更新之前的速度时间戳
				_vel_prev = vel; // 更新之前的速度

			} else {
				/* 位置数据过时，重置加速度 */
				_pos_acc.zero();
				_vel_prev.zero();
				_vel_prev_timestamp = 0;
			}
		}
	}
}

void AttitudeEstimatorQ::update_visual_odometry()
{
	if (_vehicle_visual_odometry_sub.updated()) { // 检查视觉里程计数据是否更新
		vehicle_odometry_s vision;

		if (_vehicle_visual_odometry_sub.update(&vision)) { // 更新视觉里程计数据
			// 验证视觉姿态数据
			bool vision_att_valid = PX4_ISFINITE(vision.q[0])
						&& (PX4_ISFINITE(vision.orientation_variance[0]) ? sqrtf(fmaxf(
								vision.orientation_variance[0],
								fmaxf(vision.orientation_variance[1],
										vision.orientation_variance[2]))) <= _eo_max_std_dev : true);

			if (vision_att_valid) { // 如果视觉姿态数据有效
				Dcmf Rvis = Quatf(vision.q); // 将四元数转换为方向余弦矩阵
				Vector3f v(1.0f, 0.0f, 0.4f); // 定义一个向量

				// Rvis 是 Rwr（相对于世界的机器人），而 v 是相对于世界的。
				// 因此 Rvis 必须转置为 (Rwr)' * Vw
				// Rrw * Vw = vn。这样我们就有了一致性
				_vision_hdg = Rvis.transpose() * v; // 计算视觉方向

				// 视觉外部方向使用（ATT_EXT_HDG_M 1）
				if (_param_att_ext_hdg_m.get() == 1) {
					// 检查数据是否超时
					_ext_hdg_good = vision.timestamp_sample > 0 && (hrt_elapsed_time(&vision.timestamp_sample) < 500000);
				}
			}
		}
	}
}

void AttitudeEstimatorQ::update_parameters(bool force)
{
	// 检查参数是否更新
	if (_parameter_update_sub.updated() || force) {
		// 清除更新标志
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// 从存储中更新参数
		updateParams();

		// 如果系统没有磁力计，则禁用磁力计融合
		if (_param_sys_has_mag.get() == 0) {
			_param_att_w_mag.set(0.0f);
		}

		// 如果权重为零（即禁用磁力计），确保估计器初始化
		if (_param_att_w_mag.get() < FLT_EPSILON) {
			_mag(0) = 1.f;
			_mag(1) = 0.f;
			_mag(2) = 0.f;
		}

		// 更新磁偏角
		update_mag_declination(math::radians(_param_att_mag_decl.get()));
	}
}

bool AttitudeEstimatorQ::init_attitude_q()
{
	// 旋转矩阵可以很容易地从加速度和磁场向量构造
	// 'k' 是机体坐标系下的地球 Z 轴（向下）单位向量
	Vector3f k = -_accel;
	k.normalize();

	// 'i' 是机体坐标系下的地球 X 轴（向北）单位向量，与 'k' 正交
	Vector3f i = (_mag - k * (_mag * k));
	i.normalize();

	// 'j' 是机体坐标系下的地球 Y 轴（向东）单位向量，与 'k' 和 'i' 正交
	Vector3f j = k % i;

	// 填充旋转矩阵
	Dcmf R;
	R.row(0) = i;
	R.row(1) = j;
	R.row(2) = k;

	// 转换为四元数
	_q = R;

	// 补偿磁偏角
	Quatf decl_rotation = Eulerf(0.0f, 0.0f, _mag_decl);
	_q = _q * decl_rotation;

	_q.normalize();

	// 检查四元数是否有效
	if (_q.isAllFinite() && _q.length() > 0.95f && _q.length() < 1.05f) {
		_initialized = true; // 初始化成功

	} else {
		_initialized = false; // 初始化失败
	}

	return _initialized; // 返回初始化状态
}
bool AttitudeEstimatorQ::update(float dt)
{
	if (!_initialized) { // 如果未初始化
		if (!_data_good) { // 如果数据不可用
			return false; // 返回 false 表示更新失败
		}

		return init_attitude_q(); // 初始化姿态四元数
	}

	Quatf q_last = _q; // 保存上一次的四元数

	// 角速度修正
	Vector3f corr; // 修正向量
	float spinRate = _gyro.length(); // 计算陀螺仪的旋转速率

	if (_param_att_ext_hdg_m.get() > 0 && _ext_hdg_good) { // 如果启用了外部方向修正且数据良好
		if (_param_att_ext_hdg_m.get() == 1) {
			// 视觉方向修正
			// 将方向投影到全局坐标系并提取 XY 分量
			Vector3f vision_hdg_earth = _q.rotateVector(_vision_hdg);
			float vision_hdg_err = wrap_pi(atan2f(vision_hdg_earth(1), vision_hdg_earth(0)));
			// 将修正投影到机体坐标系
			corr += _q.rotateVectorInverse(Vector3f(0.0f, 0.0f, -vision_hdg_err)) * _param_att_w_ext_hdg.get();
		}

		if (_param_att_ext_hdg_m.get() == 2) {
			// 运动捕捉方向修正
			// 将方向投影到全局坐标系并提取 XY 分量
			Vector3f mocap_hdg_earth = _q.rotateVector(_mocap_hdg);
			float mocap_hdg_err = wrap_pi(atan2f(mocap_hdg_earth(1), mocap_hdg_earth(0)));
			// 将修正投影到机体坐标系
			corr += _q.rotateVectorInverse(Vector3f(0.0f, 0.0f, -mocap_hdg_err)) * _param_att_w_ext_hdg.get();
		}
	}

	if (_param_att_ext_hdg_m.get() == 0 || !_ext_hdg_good) {
		// 磁力计修正
		// 将磁场向量投影到全局坐标系并提取 XY 分量
		Vector3f mag_earth = _q.rotateVector(_mag);
		float mag_err = wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
		float gainMult = 1.0f;
		const float fifty_dps = 0.873f;

		if (spinRate > fifty_dps) {
			gainMult = math::min(spinRate / fifty_dps, 10.0f);
		}

		// 将磁力计修正投影到机体坐标系
		corr += _q.rotateVectorInverse(Vector3f(0.0f, 0.0f, -mag_err)) * _param_att_w_mag.get() * gainMult;
	}

	_q.normalize(); // 归一化四元数

	// 加速度计修正
	// 将地球坐标系的 'k' 单位向量投影到机体坐标系
	Vector3f k(
		2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
		2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
		(_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3))
	);

	// 如果未使用基于 GPS 速度的加速度补偿，
	// 仅在加速度的模接近 1 g 时融合加速度数据（减少漂移）。
	const float accel_norm_sq = _accel.norm_squared();
	const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
	const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;

	if (_param_att_acc_comp.get() || ((accel_norm_sq > lower_accel_limit * lower_accel_limit) &&
					  (accel_norm_sq < upper_accel_limit * upper_accel_limit))) {

		corr += (k % (_accel - _pos_acc).normalized()) * _param_att_w_acc.get();
	}

	// 陀螺仪偏差估计
	if (spinRate < 0.175f) {
		_gyro_bias += corr * (_param_att_w_gyro_bias.get() * dt);

		for (int i = 0; i < 3; i++) {
			_gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max);
		}
	}

	_rates = _gyro + _gyro_bias; // 更新陀螺仪速率

	// 前馈陀螺仪
	corr += _rates;

	// 将修正应用到状态
	_q += _q.derivative1(corr) * dt;

	// 归一化四元数
	_q.normalize();

	if (!_q.isAllFinite()) { // 如果四元数无效
		// 重置四元数到上一次的有效状态
		_q = q_last;
		_rates.zero();
		_gyro_bias.zero();
		return false; // 返回 false 表示更新失败
	}

	return true; // 返回 true 表示更新成功
}

void AttitudeEstimatorQ::update_mag_declination(float new_declination)
{
	// 应用初始磁偏角或微小旋转而不改变估计
	if (!_initialized || fabsf(new_declination - _mag_decl) < 0.0001f) {
		_mag_decl = new_declination;

	} else {
		// 立即旋转当前估计以避免陀螺仪偏差增长
		Quatf decl_rotation = Eulerf(0.0f, 0.0f, new_declination - _mag_decl);
		_q = _q * decl_rotation;
		_mag_decl = new_declination;
	}
}

int AttitudeEstimatorQ::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command"); // 打印未知命令的用法
}

int AttitudeEstimatorQ::task_spawn(int argc, char *argv[])
{
	AttitudeEstimatorQ *instance = new AttitudeEstimatorQ(); // 创建 AttitudeEstimatorQ 实例

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

int AttitudeEstimatorQ::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason); // 打印警告信息
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude estimator q.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("AttitudeEstimatorQ", "estimator");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0; // 返回 0 表示成功
}

extern "C" __EXPORT int attitude_estimator_q_main(int argc, char *argv[])
{
    return AttitudeEstimatorQ::main(argc, argv); // 调用 AttitudeEstimatorQ 的 main 函数
}
