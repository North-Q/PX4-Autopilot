/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#include <battery/battery.h>
#include <parameters/param.h>

// AnalogBattery 类继承自 Battery 类
class AnalogBattery : public Battery
{
public:
	// 构造函数
	AnalogBattery(int index, ModuleParams *parent, const int sample_interval_us, const uint8_t source,
		      const uint8_t priority);

	/**
	 * 更新当前电池状态消息。
	 *
	 * @param voltage_raw 从 ADC 读取的电池电压，单位：伏特
	 * @param current_raw 电流感应电阻的电压，单位：伏特
	 * @param timestamp 读取 ADC 的时间（使用 hrt_absolute_time()）
	 * @param source 参数 BAT%d_SOURCE 定义的来源
	 * @param priority: 电池编号 -1。术语 priority 指的是 LTC4417 上的 Vn 连接
	 */
	void updateBatteryStatusADC(hrt_abstime timestamp, float voltage_raw, float current_raw);

	/**
	 * 检查该电池的电压 ADC 通道是否有效。
	 * 对应于 BOARD_BRICK_VALID_LIST
	 */
	bool is_valid();

	/**
	 * 获取用于读取该电池电压的 ADC 通道
	 */
	int get_voltage_channel();

	/**
	 * 获取用于读取该电池电流的 ADC 通道
	 */
	int get_current_channel();

protected:

	// 参数句柄结构体
	struct {
		param_t v_offs_cur; // 电压偏移电流参数句柄
		param_t v_div;      // 电压分压参数句柄
		param_t a_per_v;    // 每伏特安培数参数句柄
		param_t v_channel;  // 电压通道参数句柄
		param_t i_channel;  // 电流通道参数句柄
	} _analog_param_handles;

	// 参数值结构体
	struct {
		float v_offs_cur;   // 电压偏移电流参数值
		float v_div;        // 电压分压参数值
		float a_per_v;      // 每伏特安培数参数值
		int32_t v_channel;  // 电压通道参数值
		int32_t i_channel;  // 电流通道参数值
	} _analog_params;

	// 更新参数的虚函数
	virtual void updateParams() override;
};
