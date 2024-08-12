/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file sensors.cpp
 * 传感器模块的实现文件
 *
 * 作者列表：
 * - Lorenz Meier <lorenz@px4.io>
 * - Julian Oes <julian@oes.ch>
 * - Thomas Gubler <thomas@px4.io>
 * - Anton Babushkin <anton@px4.io>
 * - Beat Küng <beat-kueng@gmx.net>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/battery/battery.h>
#include <lib/conversion/rotation.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/adc_report.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "analog_battery.h"

using namespace time_literals;

/**
 * 通道定义（例如 ADC_BATTERY_VOLTAGE_CHANNEL、ADC_BATTERY_CURRENT_CHANNEL 和 ADC_AIRSPEED_VOLTAGE_CHANNEL）在 board_config.h 中定义
 */

#ifndef BOARD_NUMBER_BRICKS
#error "battery_status 模块需要电源模块"
#endif

#if BOARD_NUMBER_BRICKS == 0
#error "battery_status 模块需要电源模块"
#endif

// BatteryStatus 类继承自 ModuleBase、ModuleParams 和 px4::ScheduledWorkItem
class BatteryStatus : public ModuleBase<BatteryStatus>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	// 构造函数
	BatteryStatus();
	// 析构函数
	~BatteryStatus() override;

	/** @see ModuleBase */
	// 任务生成函数
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	// 自定义命令函数
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	// 打印用法函数
	static int print_usage(const char *reason = nullptr);

	// 初始化函数
	bool init();

private:
	// 运行函数，重写自 px4::ScheduledWorkItem
	void Run() override;

	// 订阅参数更新的 uORB 主题，间隔为 1 秒
	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	// 订阅 ADC 报告的 uORB 主题，并设置回调
	uORB::SubscriptionCallbackWorkItem _adc_report_sub{this, ORB_ID(adc_report)};

	// 采样频率，单位为 Hz
	static constexpr uint32_t SAMPLE_FREQUENCY_HZ = 100;
	// 采样间隔，单位为微秒
	static constexpr uint32_t SAMPLE_INTERVAL_US  = 1_s / SAMPLE_FREQUENCY_HZ;

	// 第一个模拟电池对象
	AnalogBattery _battery1;

#if BOARD_NUMBER_BRICKS > 1
	// 如果电池数量大于 1，则定义第二个模拟电池对象
	AnalogBattery _battery2;
#endif

	// 模拟电池数组，根据电池数量初始化
	AnalogBattery *_analogBatteries[BOARD_NUMBER_BRICKS] {
		&_battery1,
#if BOARD_NUMBER_BRICKS > 1
		&_battery2,
#endif
	}; // 结束 _analogBatteries 数组定义

	// 循环性能计数器
	perf_counter_t	_loop_perf;

	/**
	 * 检查参数是否有变化。
	 */
	void 		parameter_update_poll(bool forced = false);

	/**
	 * 轮询 ADC 并更新读数。
	 *
	 * @param raw 组合传感器数据结构，用于返回数据。
	 */
	void		adc_poll();
};

BatteryStatus::BatteryStatus() :
	ModuleParams(nullptr), // 初始化基类 ModuleParams
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default), // 初始化基类 ScheduledWorkItem
	_battery1(1, this, SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE,
		  0), // 初始化第一个电池对象
#if BOARD_NUMBER_BRICKS > 1
	_battery2(2, this, SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE,
		  1), // 如果有多个电池，初始化第二个电池对象
#endif
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME)) // 初始化性能计数器
{
	updateParams(); // 更新参数
}

BatteryStatus::~BatteryStatus()
{
	ScheduleClear(); // 清除调度
}

void
BatteryStatus::parameter_update_poll(bool forced)
{
	// 检查参数是否有更新
	if (_parameter_update_sub.updated() || forced) {
		// 清除更新标志
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// 从存储中更新参数
		updateParams();
	}
}

void
BatteryStatus::adc_poll()
{
	/* 为了兼容旧版，我们发布与 VDD_5V_IN 选择源相关的电池状态
	 * 选择在硬件中完成，例如通过 LTC4417 或类似设备，或者可能是硬编码的
	 * 就像在 FMUv4 中一样
	 */

	/* 每个电池的读数，默认未读通道为 0 */
	float bat_current_adc_readings[BOARD_NUMBER_BRICKS] {}; // 电池电流 ADC 读数
	float bat_voltage_adc_readings[BOARD_NUMBER_BRICKS] {}; // 电池电压 ADC 读数
	bool has_bat_voltage_adc_channel[BOARD_NUMBER_BRICKS] {}; // 是否有电池电压 ADC 通道

	int selected_source = -1; // 选择的电源源

	adc_report_s adc_report; // ADC 报告结构体

	if (_adc_report_sub.update(&adc_report)) {

		/* 读取我们得到的所有通道 */
		for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
			for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {

				/* 一旦我们有了订阅，为最低（最高优先级的电源控制器）且有效的电池执行一次 */
				if (selected_source < 0 && _analogBatteries[b]->is_valid()) {
					/* 表示最低的电池（电源控制器上最高优先级的电源）
					 * 作为 VDD_5V_IN 的选择源
					 */
					selected_source = b;
				}

				/* 查找特定通道并将原始电压转换为测量数据 */

				if (adc_report.channel_id[i] >= 0) {
					if (adc_report.channel_id[i] == _analogBatteries[b]->get_voltage_channel()) {
						/* 电压（单位：伏特） */
						bat_voltage_adc_readings[b] = adc_report.raw_data[i] *
									      adc_report.v_ref /
									      adc_report.resolution;
						has_bat_voltage_adc_channel[b] = true;

					} else if (adc_report.channel_id[i] == _analogBatteries[b]->get_current_channel()) {
						bat_current_adc_readings[b] = adc_report.raw_data[i] *
									      adc_report.v_ref /
									      adc_report.resolution;
					}
				}

			}
		}

		for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {

			if (has_bat_voltage_adc_channel[b]) { // 如果没有配置电压通道，则不发布
				_analogBatteries[b]->updateBatteryStatusADC(
					hrt_absolute_time(), // 当前时间戳
					bat_voltage_adc_readings[b], // 电压读数
					bat_current_adc_readings[b] // 电流读数
				);
			}
		}
	}
}

void
BatteryStatus::Run()
{
	if (should_exit()) {
		exit_and_cleanup(); // 退出并清理
		return;
	}

	perf_begin(_loop_perf); // 开始性能计数

	/* 检查参数是否有更新 */
	parameter_update_poll();

	/* 检查电池电压 */
	adc_poll();

	perf_end(_loop_perf); // 结束性能计数
}

int
BatteryStatus::task_spawn(int argc, char *argv[])
{
	BatteryStatus *instance = new BatteryStatus(); // 创建 BatteryStatus 实例

	if (instance) {
		_object.store(instance); // 存储实例对象
		_task_id = task_id_is_work_queue; // 设置任务 ID

		if (instance->init()) { // 初始化实例
			return PX4_OK; // 初始化成功
		}

	} else {
		PX4_ERR("alloc failed"); // 分配失败
	}

	delete instance; // 删除实例
	_object.store(nullptr); // 清空实例对象
	_task_id = -1; // 重置任务 ID

	return PX4_ERROR; // 返回错误
}

bool
BatteryStatus::init()
{
	return _adc_report_sub.registerCallback(); // 注册 ADC 报告回调
}

int BatteryStatus::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command"); // 打印未知命令的用法
}

int BatteryStatus::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason); // 打印警告信息
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

该模块提供的功能包括：
- 读取来自 ADC 驱动程序的输出（通过 ioctl 接口）并发布 `battery_status`。


### Implementation
它运行在自己的线程中，并轮询当前选择的陀螺仪主题。

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("battery_status", "system"); // 打印模块名称
    PRINT_MODULE_USAGE_COMMAND("start"); // 打印启动命令
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); // 打印默认命令

    return 0;
}

extern "C" __EXPORT int battery_status_main(int argc, char *argv[])
{
    return BatteryStatus::main(argc, argv); // 主函数入口
}
