/* Copyright 2020-2023 The MathWorks, Inc. */

#include "uORBMetaData.h"
orb_metadata_t* getorbData(uint8_T uORBID) {
    orb_metadata_t* orbData;
    switch (uORBID) {
    case 1:
        orbData = ORB_ID(action_request);
        break;
    case 2:
        orbData = ORB_ID(actuator_armed);
        break;
    case 3:
        orbData = ORB_ID(actuator_controls_status_0);
        break;
    case 4:
        orbData = ORB_ID(actuator_controls_status_1);
        break;
    case 5:
        orbData = ORB_ID(actuator_motors);
        break;
    case 6:
        orbData = ORB_ID(actuator_outputs);
        break;
    case 7:
        orbData = ORB_ID(actuator_outputs_sim);
        break;
    case 8:
        orbData = ORB_ID(actuator_outputs_debug);
        break;
    case 9:
        orbData = ORB_ID(actuator_servos);
        break;
    case 10:
        orbData = ORB_ID(actuator_servos_trim);
        break;
    case 11:
        orbData = ORB_ID(actuator_test);
        break;
    case 12:
        orbData = ORB_ID(adc_report);
        break;
    case 13:
        orbData = ORB_ID(airspeed);
        break;
    case 14:
        orbData = ORB_ID(airspeed_validated);
        break;
    case 15:
        orbData = ORB_ID(airspeed_wind);
        break;
    case 16:
        orbData = ORB_ID(autotune_attitude_control_status);
        break;
    case 17:
        orbData = ORB_ID(battery_status);
        break;
    case 18:
        orbData = ORB_ID(button_event);
        break;
    case 19:
        orbData = ORB_ID(safety_button);
        break;
    case 20:
        orbData = ORB_ID(camera_capture);
        break;
    case 21:
        orbData = ORB_ID(camera_status);
        break;
    case 22:
        orbData = ORB_ID(camera_trigger);
        break;
    case 23:
        orbData = ORB_ID(cellular_status);
        break;
    case 24:
        orbData = ORB_ID(collision_constraints);
        break;
    case 25:
        orbData = ORB_ID(collision_report);
        break;
    case 26:
        orbData = ORB_ID(control_allocator_status);
        break;
    case 27:
        orbData = ORB_ID(cpuload);
        break;
    case 28:
        orbData = ORB_ID(debug_array);
        break;
    case 29:
        orbData = ORB_ID(debug_key_value);
        break;
    case 30:
        orbData = ORB_ID(debug_value);
        break;
    case 31:
        orbData = ORB_ID(debug_vect);
        break;
    case 32:
        orbData = ORB_ID(differential_pressure);
        break;
    case 33:
        orbData = ORB_ID(distance_sensor);
        break;
    case 34:
        orbData = ORB_ID(ekf2_timestamps);
        break;
    case 35:
        orbData = ORB_ID(esc_report);
        break;
    case 36:
        orbData = ORB_ID(esc_status);
        break;
    case 37:
        orbData = ORB_ID(estimator_aid_src_baro_hgt);
        break;
    case 38:
        orbData = ORB_ID(estimator_aid_src_ev_hgt);
        break;
    case 39:
        orbData = ORB_ID(estimator_aid_src_gnss_hgt);
        break;
    case 40:
        orbData = ORB_ID(estimator_aid_src_rng_hgt);
        break;
    case 41:
        orbData = ORB_ID(estimator_aid_src_airspeed);
        break;
    case 42:
        orbData = ORB_ID(estimator_aid_src_sideslip);
        break;
    case 43:
        orbData = ORB_ID(estimator_aid_src_fake_hgt);
        break;
    case 44:
        orbData = ORB_ID(estimator_aid_src_mag_heading);
        break;
    case 45:
        orbData = ORB_ID(estimator_aid_src_gnss_yaw);
        break;
    case 46:
        orbData = ORB_ID(estimator_aid_src_ev_yaw);
        break;
    case 47:
        orbData = ORB_ID(estimator_aid_src_ev_pos);
        break;
    case 48:
        orbData = ORB_ID(estimator_aid_src_fake_pos);
        break;
    case 49:
        orbData = ORB_ID(estimator_aid_src_gnss_pos);
        break;
    case 50:
        orbData = ORB_ID(estimator_aid_src_aux_vel);
        break;
    case 51:
        orbData = ORB_ID(estimator_aid_src_optical_flow);
        break;
    case 52:
        orbData = ORB_ID(estimator_aid_src_terrain_optical_flow);
        break;
    case 53:
        orbData = ORB_ID(estimator_aid_src_ev_vel);
        break;
    case 54:
        orbData = ORB_ID(estimator_aid_src_gnss_vel);
        break;
    case 55:
        orbData = ORB_ID(estimator_aid_src_gravity);
        break;
    case 56:
        orbData = ORB_ID(estimator_aid_src_mag);
        break;
    case 57:
        orbData = ORB_ID(estimator_baro_bias);
        break;
    case 58:
        orbData = ORB_ID(estimator_gnss_hgt_bias);
        break;
    case 59:
        orbData = ORB_ID(estimator_rng_hgt_bias);
        break;
    case 60:
        orbData = ORB_ID(estimator_bias3d);
        break;
    case 61:
        orbData = ORB_ID(estimator_ev_pos_bias);
        break;
    case 62:
        orbData = ORB_ID(estimator_event_flags);
        break;
    case 63:
        orbData = ORB_ID(estimator_gps_status);
        break;
    case 64:
        orbData = ORB_ID(estimator_innovations);
        break;
    case 65:
        orbData = ORB_ID(estimator_innovation_variances);
        break;
    case 66:
        orbData = ORB_ID(estimator_innovation_test_ratios);
        break;
    case 67:
        orbData = ORB_ID(estimator_selector_status);
        break;
    case 68:
        orbData = ORB_ID(estimator_sensor_bias);
        break;
    case 69:
        orbData = ORB_ID(estimator_states);
        break;
    case 70:
        orbData = ORB_ID(estimator_status);
        break;
    case 71:
        orbData = ORB_ID(estimator_status_flags);
        break;
    case 72:
        orbData = ORB_ID(event);
        break;
    case 73:
        orbData = ORB_ID(failsafe_flags);
        break;
    case 74:
        orbData = ORB_ID(failure_detector_status);
        break;
    case 75:
        orbData = ORB_ID(follow_target);
        break;
    case 76:
        orbData = ORB_ID(follow_target_estimator);
        break;
    case 77:
        orbData = ORB_ID(follow_target_status);
        break;
    case 78:
        orbData = ORB_ID(generator_status);
        break;
    case 79:
        orbData = ORB_ID(geofence_result);
        break;
    case 80:
        orbData = ORB_ID(gimbal_controls);
        break;
    case 81:
        orbData = ORB_ID(gimbal_device_attitude_status);
        break;
    case 82:
        orbData = ORB_ID(gimbal_device_information);
        break;
    case 83:
        orbData = ORB_ID(gimbal_device_set_attitude);
        break;
    case 84:
        orbData = ORB_ID(gimbal_manager_information);
        break;
    case 85:
        orbData = ORB_ID(gimbal_manager_set_attitude);
        break;
    case 86:
        orbData = ORB_ID(gimbal_manager_set_manual_control);
        break;
    case 87:
        orbData = ORB_ID(gimbal_manager_status);
        break;
    case 88:
        orbData = ORB_ID(gps_dump);
        break;
    case 89:
        orbData = ORB_ID(gps_inject_data);
        break;
    case 90:
        orbData = ORB_ID(gripper);
        break;
    case 91:
        orbData = ORB_ID(health_report);
        break;
    case 92:
        orbData = ORB_ID(heater_status);
        break;
    case 93:
        orbData = ORB_ID(home_position);
        break;
    case 94:
        orbData = ORB_ID(hover_thrust_estimate);
        break;
    case 95:
        orbData = ORB_ID(input_rc);
        break;
    case 96:
        orbData = ORB_ID(internal_combustion_engine_status);
        break;
    case 97:
        orbData = ORB_ID(iridiumsbd_status);
        break;
    case 98:
        orbData = ORB_ID(irlock_report);
        break;
    case 99:
        orbData = ORB_ID(landing_gear);
        break;
    case 100:
        orbData = ORB_ID(landing_gear_wheel);
        break;
    case 101:
        orbData = ORB_ID(landing_target_innovations);
        break;
    case 102:
        orbData = ORB_ID(landing_target_pose);
        break;
    case 103:
        orbData = ORB_ID(launch_detection_status);
        break;
    case 104:
        orbData = ORB_ID(led_control);
        break;
    case 105:
        orbData = ORB_ID(log_message);
        break;
    case 106:
        orbData = ORB_ID(logger_status);
        break;
    case 107:
        orbData = ORB_ID(mag_worker_data);
        break;
    case 108:
        orbData = ORB_ID(magnetometer_bias_estimate);
        break;
    case 109:
        orbData = ORB_ID(manual_control_setpoint);
        break;
    case 110:
        orbData = ORB_ID(manual_control_input);
        break;
    case 111:
        orbData = ORB_ID(manual_control_switches);
        break;
    case 112:
        orbData = ORB_ID(mavlink_log);
        break;
    case 113:
        orbData = ORB_ID(mavlink_tunnel);
        break;
    case 114:
        orbData = ORB_ID(mission);
        break;
    case 115:
        orbData = ORB_ID(mission_result);
        break;
    case 116:
        orbData = ORB_ID(mode_completed);
        break;
    case 117:
        orbData = ORB_ID(mount_orientation);
        break;
    case 118:
        orbData = ORB_ID(navigator_mission_item);
        break;
    case 119:
        orbData = ORB_ID(flaps_setpoint);
        break;
    case 120:
        orbData = ORB_ID(spoilers_setpoint);
        break;
    case 121:
        orbData = ORB_ID(npfg_status);
        break;
    case 122:
        orbData = ORB_ID(obstacle_distance);
        break;
    case 123:
        orbData = ORB_ID(obstacle_distance_fused);
        break;
    case 124:
        orbData = ORB_ID(offboard_control_mode);
        break;
    case 125:
        orbData = ORB_ID(onboard_computer_status);
        break;
    case 126:
        orbData = ORB_ID(orb_test);
        break;
    case 127:
        orbData = ORB_ID(orb_multitest);
        break;
    case 128:
        orbData = ORB_ID(orb_test_large);
        break;
    case 129:
        orbData = ORB_ID(orb_test_medium);
        break;
    case 130:
        orbData = ORB_ID(orb_test_medium_multi);
        break;
    case 131:
        orbData = ORB_ID(orb_test_medium_wrap_around);
        break;
    case 132:
        orbData = ORB_ID(orb_test_medium_queue);
        break;
    case 133:
        orbData = ORB_ID(orb_test_medium_queue_poll);
        break;
    case 134:
        orbData = ORB_ID(orbit_status);
        break;
    case 135:
        orbData = ORB_ID(parameter_update);
        break;
    case 136:
        orbData = ORB_ID(ping);
        break;
    case 137:
        orbData = ORB_ID(position_controller_landing_status);
        break;
    case 138:
        orbData = ORB_ID(position_controller_status);
        break;
    case 139:
        orbData = ORB_ID(position_setpoint);
        break;
    case 140:
        orbData = ORB_ID(position_setpoint_triplet);
        break;
    case 141:
        orbData = ORB_ID(power_button_state);
        break;
    case 142:
        orbData = ORB_ID(power_monitor);
        break;
    case 143:
        orbData = ORB_ID(pps_capture);
        break;
    case 144:
        orbData = ORB_ID(pwm_input);
        break;
    case 145:
        orbData = ORB_ID(px4io_status);
        break;
    case 146:
        orbData = ORB_ID(qshell_req);
        break;
    case 147:
        orbData = ORB_ID(qshell_retval);
        break;
    case 148:
        orbData = ORB_ID(radio_status);
        break;
    case 149:
        orbData = ORB_ID(rate_ctrl_status);
        break;
    case 150:
        orbData = ORB_ID(rc_channels);
        break;
    case 151:
        orbData = ORB_ID(rc_parameter_map);
        break;
    case 152:
        orbData = ORB_ID(rpm);
        break;
    case 153:
        orbData = ORB_ID(rtl_time_estimate);
        break;
    case 154:
        orbData = ORB_ID(satellite_info);
        break;
    case 155:
        orbData = ORB_ID(sensor_accel);
        break;
    case 156:
        orbData = ORB_ID(sensor_accel_fifo);
        break;
    case 157:
        orbData = ORB_ID(sensor_baro);
        break;
    case 158:
        orbData = ORB_ID(sensor_combined);
        break;
    case 159:
        orbData = ORB_ID(sensor_correction);
        break;
    case 160:
        orbData = ORB_ID(sensor_gnss_relative);
        break;
    case 161:
        orbData = ORB_ID(sensor_gps);
        break;
    case 162:
        orbData = ORB_ID(vehicle_gps_position);
        break;
    case 163:
        orbData = ORB_ID(sensor_gyro);
        break;
    case 164:
        orbData = ORB_ID(sensor_gyro_fft);
        break;
    case 165:
        orbData = ORB_ID(sensor_gyro_fifo);
        break;
    case 166:
        orbData = ORB_ID(sensor_hygrometer);
        break;
    case 167:
        orbData = ORB_ID(sensor_mag);
        break;
    case 168:
        orbData = ORB_ID(sensor_optical_flow);
        break;
    case 169:
        orbData = ORB_ID(sensor_preflight_mag);
        break;
    case 170:
        orbData = ORB_ID(sensor_selection);
        break;
    case 171:
        orbData = ORB_ID(sensor_uwb);
        break;
    case 172:
        orbData = ORB_ID(sensors_status_baro);
        break;
    case 173:
        orbData = ORB_ID(sensors_status_mag);
        break;
    case 174:
        orbData = ORB_ID(sensors_status_imu);
        break;
    case 175:
        orbData = ORB_ID(system_power);
        break;
    case 176:
        orbData = ORB_ID(takeoff_status);
        break;
    case 177:
        orbData = ORB_ID(task_stack_info);
        break;
    case 178:
        orbData = ORB_ID(tecs_status);
        break;
    case 179:
        orbData = ORB_ID(telemetry_status);
        break;
    case 180:
        orbData = ORB_ID(tiltrotor_extra_controls);
        break;
    case 181:
        orbData = ORB_ID(timesync_status);
        break;
    case 182:
        orbData = ORB_ID(trajectory_bezier);
        break;
    case 183:
        orbData = ORB_ID(trajectory_setpoint);
        break;
    case 184:
        orbData = ORB_ID(trajectory_waypoint);
        break;
    case 185:
        orbData = ORB_ID(transponder_report);
        break;
    case 186:
        orbData = ORB_ID(tune_control);
        break;
    case 187:
        orbData = ORB_ID(uavcan_parameter_request);
        break;
    case 188:
        orbData = ORB_ID(uavcan_parameter_value);
        break;
    case 189:
        orbData = ORB_ID(ulog_stream);
        break;
    case 190:
        orbData = ORB_ID(ulog_stream_ack);
        break;
    case 191:
        orbData = ORB_ID(vehicle_acceleration);
        break;
    case 192:
        orbData = ORB_ID(vehicle_air_data);
        break;
    case 193:
        orbData = ORB_ID(vehicle_angular_acceleration_setpoint);
        break;
    case 194:
        orbData = ORB_ID(vehicle_angular_velocity);
        break;
    case 195:
        orbData = ORB_ID(vehicle_angular_velocity_groundtruth);
        break;
    case 196:
        orbData = ORB_ID(vehicle_attitude);
        break;
    case 197:
        orbData = ORB_ID(vehicle_attitude_groundtruth);
        break;
    case 198:
        orbData = ORB_ID(external_ins_attitude);
        break;
    case 199:
        orbData = ORB_ID(estimator_attitude);
        break;
    case 200:
        orbData = ORB_ID(vehicle_attitude_setpoint);
        break;
    case 201:
        orbData = ORB_ID(mc_virtual_attitude_setpoint);
        break;
    case 202:
        orbData = ORB_ID(fw_virtual_attitude_setpoint);
        break;
    case 203:
        orbData = ORB_ID(vehicle_command);
        break;
    case 204:
        orbData = ORB_ID(gimbal_v1_command);
        break;
    case 205:
        orbData = ORB_ID(vehicle_command_ack);
        break;
    case 206:
        orbData = ORB_ID(vehicle_constraints);
        break;
    case 207:
        orbData = ORB_ID(vehicle_control_mode);
        break;
    case 208:
        orbData = ORB_ID(vehicle_global_position);
        break;
    case 209:
        orbData = ORB_ID(vehicle_global_position_groundtruth);
        break;
    case 210:
        orbData = ORB_ID(external_ins_global_position);
        break;
    case 211:
        orbData = ORB_ID(estimator_global_position);
        break;
    case 212:
        orbData = ORB_ID(vehicle_imu);
        break;
    case 213:
        orbData = ORB_ID(vehicle_imu_status);
        break;
    case 214:
        orbData = ORB_ID(vehicle_land_detected);
        break;
    case 215:
        orbData = ORB_ID(vehicle_local_position);
        break;
    case 216:
        orbData = ORB_ID(vehicle_local_position_groundtruth);
        break;
    case 217:
        orbData = ORB_ID(external_ins_local_position);
        break;
    case 218:
        orbData = ORB_ID(estimator_local_position);
        break;
    case 219:
        orbData = ORB_ID(vehicle_local_position_setpoint);
        break;
    case 220:
        orbData = ORB_ID(vehicle_magnetometer);
        break;
    case 221:
        orbData = ORB_ID(vehicle_odometry);
        break;
    case 222:
        orbData = ORB_ID(vehicle_mocap_odometry);
        break;
    case 223:
        orbData = ORB_ID(vehicle_visual_odometry);
        break;
    case 224:
        orbData = ORB_ID(estimator_odometry);
        break;
    case 225:
        orbData = ORB_ID(vehicle_optical_flow);
        break;
    case 226:
        orbData = ORB_ID(estimator_optical_flow_vel);
        break;
    case 227:
        orbData = ORB_ID(vehicle_optical_flow_vel);
        break;
    case 228:
        orbData = ORB_ID(vehicle_rates_setpoint);
        break;
    case 229:
        orbData = ORB_ID(vehicle_roi);
        break;
    case 230:
        orbData = ORB_ID(vehicle_status);
        break;
    case 231:
        orbData = ORB_ID(vehicle_thrust_setpoint);
        break;
    case 232:
        orbData = ORB_ID(vehicle_thrust_setpoint_virtual_fw);
        break;
    case 233:
        orbData = ORB_ID(vehicle_thrust_setpoint_virtual_mc);
        break;
    case 234:
        orbData = ORB_ID(vehicle_torque_setpoint);
        break;
    case 235:
        orbData = ORB_ID(vehicle_torque_setpoint_virtual_fw);
        break;
    case 236:
        orbData = ORB_ID(vehicle_torque_setpoint_virtual_mc);
        break;
    case 237:
        orbData = ORB_ID(vehicle_trajectory_bezier);
        break;
    case 238:
        orbData = ORB_ID(vehicle_trajectory_waypoint);
        break;
    case 239:
        orbData = ORB_ID(vehicle_trajectory_waypoint_desired);
        break;
    case 240:
        orbData = ORB_ID(vtol_vehicle_status);
        break;
    case 241:
        orbData = ORB_ID(wind);
        break;
    case 242:
        orbData = ORB_ID(estimator_wind);
        break;
    case 243:
        orbData = ORB_ID(yaw_estimator_status);
        break;
    }
    return orbData;
}
