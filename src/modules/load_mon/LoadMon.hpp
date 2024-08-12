/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#if defined(__PX4_NUTTX)
#include <malloc.h>
#endif
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform/cpuload.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/task_stack_info.h>

#if defined(__PX4_LINUX)
#include <sys/times.h>
#endif

namespace load_mon
{

// LoadMon 类继承自 ModuleBase、ModuleParams 和 px4::ScheduledWorkItem
class LoadMon : public ModuleBase<LoadMon>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	// 构造函数
	LoadMon();
	// 析构函数
	~LoadMon() override;

	// 任务生成函数
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	// 自定义命令处理函数
	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command"); // 打印未知命令的使用说明
	}

	/** @see ModuleBase */
	// 打印使用说明函数
	static int print_usage(const char *reason = nullptr);

	// 启动函数
	void start();

private:
	/** 执行计算并调度下一个周期。 */
	void Run() override;

	/** 计算 CPU 负载并发布。 */
	void cpuload();

	/* 仅在 Nuttx 上可用的堆栈检查 */
#if defined(__PX4_NUTTX)
	/* 计算堆栈使用情况 */
	void stack_usage();

	int _stack_task_index{0}; // 堆栈任务索引

	uORB::Publication<task_stack_info_s> _task_stack_info_pub{ORB_ID(task_stack_info)}; // 发布任务堆栈信息的 uORB 发布对象
#endif
	uORB::Publication<cpuload_s> _cpuload_pub {ORB_ID(cpuload)}; // 发布 CPU 负载信息的 uORB 发布对象

#if defined(__PX4_LINUX)
	FILE *_proc_fd = nullptr; // 文件指针，用于读取 /proc 文件系统
	/* 直接从 Linux 上的时钟周期计算使用情况 */
	clock_t _last_total_time_stamp{}; // 上次总时间戳
	clock_t _last_spent_time_stamp{}; // 上次消耗时间戳
#elif defined(__PX4_NUTTX)
	hrt_abstime _last_idle_time {0}; // 上次空闲时间
	hrt_abstime _last_idle_time_sample{0}; // 上次空闲时间采样
#endif

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")}; // 性能计数器，用于测量周期时间

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SYS_STCK_EN>) _param_sys_stck_en // 参数定义，是否启用堆栈检查
	)
};

} // namespace load_mon
