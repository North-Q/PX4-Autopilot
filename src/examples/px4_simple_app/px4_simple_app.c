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
 * @file px4_simple_app.c
 * PX4自动驾驶仪的最小应用示例
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>

// main函数必须命名为<module_name>_main，并从模块中导出
__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!"); // 打印欢迎信息

    /* 订阅vehicle_acceleration主题 */
    int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
    /* 将更新速率限制为5 Hz */
    orb_set_interval(sensor_sub_fd, 200);

    /* 广播attitude主题 */
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

    /* 可以使用这种技术等待多个主题，这里只使用一个 */
    px4_pollfd_struct_t fds[] = {
        { .fd = sensor_sub_fd,   .events = POLLIN },
        /* 这里可以有更多的文件描述符，形式如下：
         * { .fd = other_sub_fd,   .events = POLLIN },
         */
    };

    int error_counter = 0;

    for (int i = 0; i < 5; i++) {
        /* 等待1个文件描述符的传感器更新1000毫秒（1秒） */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* 处理poll结果 */
        if (poll_ret == 0) {
            /* 这意味着我们的提供者没有给我们数据 */
            PX4_ERR("在一秒内没有收到数据");

        } else if (poll_ret < 0) {
            /* 这是非常严重的错误 - 应该是紧急情况 */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* 使用计数器防止泛滥（并减慢我们的速度） */
                PX4_ERR("poll()返回值错误: %d", poll_ret);
            }

            error_counter++;

        } else {

            if (fds[0].revents & POLLIN) {
                /* 获取第一个文件描述符的数据 */
                struct vehicle_acceleration_s accel;
                /* 将传感器的原始数据复制到本地缓冲区 */
                orb_copy(ORB_ID(vehicle_acceleration), sensor_sub_fd, &accel);
                PX4_INFO("加速度计:\t%8.4f\t%8.4f\t%8.4f",
                     (double)accel.xyz[0],
                     (double)accel.xyz[1],
                     (double)accel.xyz[2]);

                /* 设置att并发布此信息供其他应用程序使用
                 以下内容没有任何意义，只是一个示例
                */
                att.q[0] = accel.xyz[0];
                att.q[1] = accel.xyz[1];
                att.q[2] = accel.xyz[2];

                orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
            }

            /* 这里可以有更多的文件描述符，形式如下：
             * if (fds[1..n].revents & POLLIN) {}
             */
        }
    }

    PX4_INFO("退出");

    return 0;
}
