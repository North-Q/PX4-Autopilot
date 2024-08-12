/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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


#include "EscBattery.hpp"

#include <math.h>

using namespace time_literals;

// 构造函数，初始化基类和成员变量
EscBattery::EscBattery() :
	ModuleParams(nullptr), // 初始化 ModuleParams 基类
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default), // 初始化 WorkItem 基类，指定工作队列配置
	_battery(1, this, ESC_BATTERY_INTERVAL_US, battery_status_s::BATTERY_SOURCE_ESCS) // 初始化 Battery 对象
{
}

// 初始化函数
bool
EscBattery::init()
{
	// 注册 ESC 状态订阅的回调函数
	if (!_esc_status_sub.registerCallback()) {
		PX4_ERR("callback registration failed"); // 如果注册失败，打印错误信息
		return false; // 返回 false 表示初始化失败
	}

	// 设置 ESC 状态订阅的时间间隔
	_esc_status_sub.set_interval_us(ESC_BATTERY_INTERVAL_US);

	return true; // 返回 true 表示初始化成功
}

// 参数更新函数
void
EscBattery::parameters_updated()
{
	// 更新参数
	ModuleParams::updateParams();
}

// 主运行函数，实际的工作逻辑在这里实现
void
EscBattery::Run()
{
	// 如果需要退出，取消回调并清理资源
	if (should_exit()) {
		_esc_status_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// 如果参数更新了，处理参数更新
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		parameters_updated();
	}

	// 读取 ESC 状态
	esc_status_s esc_status;

	if (_esc_status_sub.copy(&esc_status)) {

		// 检查 ESC 数量是否合法
		if (esc_status.esc_count == 0 || esc_status.esc_count > esc_status_s::CONNECTED_ESC_MAX) {
			return;
		}

		// 计算在线 ESC 的数量
		const uint8_t online_esc_count = math::countSetBits(esc_status.esc_online_flags);
		float average_voltage_v = 0.0f;
		float total_current_a = 0.0f;

		// 遍历所有 ESC，计算平均电压和总电流
		for (unsigned i = 0; i < esc_status.esc_count; ++i) {
			if ((1 << i) & esc_status.esc_online_flags) {
				average_voltage_v += esc_status.esc[i].esc_voltage;
				total_current_a += esc_status.esc[i].esc_current;
			}
		}

		// 计算平均电压
		average_voltage_v /= online_esc_count;

		// 更新电池状态
		_battery.setConnected(true);
		_battery.updateVoltage(average_voltage_v);
		_battery.updateCurrent(total_current_a);
		_battery.updateAndPublishBatteryStatus(esc_status.timestamp);
	}
}

// 任务生成函数
int EscBattery::task_spawn(int argc, char *argv[])
{
	// 创建 EscBattery 实例
	EscBattery *instance = new EscBattery();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		// 初始化实例
		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed"); // 分配失败，打印错误信息
	}

	// 清理资源
	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

// 自定义命令处理函数
int EscBattery::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command"); // 打印未知命令的使用说明
}

// 打印使用说明函数
int EscBattery::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason); // 打印原因
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements using information from the ESC status and publish it as battery status.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("esc_battery", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

// 主函数入口
extern "C" __EXPORT int esc_battery_main(int argc, char *argv[])
{
    return EscBattery::main(argc, argv); // 调用 EscBattery 的 main 函数
}
