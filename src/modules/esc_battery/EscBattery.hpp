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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/parameter_update.h>
#include <battery/battery.h>


using namespace time_literals;

// EscBattery 类继承自 ModuleBase、ModuleParams 和 px4::WorkItem
class EscBattery : public ModuleBase<EscBattery>, public ModuleParams, public px4::WorkItem
{
public:
	// 构造函数
	EscBattery();
	// 默认析构函数
	~EscBattery() = default;

	/** @see ModuleBase */
	// 任务生成函数
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	// 自定义命令处理函数
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	// 打印使用说明函数
	static int print_usage(const char *reason = nullptr);

	// 初始化函数
	bool init();

private:
	// 重载的 Run 函数，实际的工作逻辑在这里实现
	void Run() override;

	// 参数更新函数
	void parameters_updated();

	// 订阅参数更新的 uORB 订阅对象，订阅间隔为 1 秒
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	// 订阅 ESC 状态的 uORB 订阅对象，带有回调功能
	uORB::SubscriptionCallbackWorkItem _esc_status_sub{this, ORB_ID(esc_status)};

	// ESC 电池状态更新的时间间隔，假设 ESC 反馈频率高于 50Hz
	static constexpr uint32_t ESC_BATTERY_INTERVAL_US = 20_ms;
	// 电池对象，用于管理和监控电池状态
	Battery _battery;
};
