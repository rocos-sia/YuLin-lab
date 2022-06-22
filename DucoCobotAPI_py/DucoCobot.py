#!/usr/bin/env python
import sys
import glob
import threading
import time

sys.path.append('gen_py')
sys.path.append('lib')
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from gen_py.robot import RPCRobot
from gen_py.robot.ttypes import StateRobot, StateProgram, OperationMode, TaskState, Op


class DucoCobot:
    def __init__(self, ip, port):
        self.transport = TSocket.TSocket(ip, port)
        self.protocol = TBinaryProtocol.TBinaryProtocol(self.transport)
        self.client = RPCRobot.Client(self.protocol)
        
    op_ = Op()
    op_.time_or_dist_1 = 0
    op_.trig_io_1 = 1
    op_.trig_value_1 = False
    op_.trig_time_1 = 0.0
    op_.trig_dist_1 = 0.0
    op_.time_or_dist_2 = 0
    op_.trig_io_2 = 1
    op_.trig_value_2 = False
    op_.trig_time_2 = 0.0
    op_.trig_dist_2 = 0.0 

    def open(self):
        try:
            self.transport.open()
        except TTransport.TTransportException as e:
            print("open mesg:",repr(e))
            return -1
        else:
            return 0   

    def close(self):
        try:
            self.transport.close()
        except Exception as e:
            print("close mesg:",repr(e))
            return -1
        else:
            return 0

    def power_on(self, block):
        return self.client.power_on(block)

    def power_off(self, block):
        return self.client.power_off(block)

    def enable(self, block):
        return self.client.enable(block)

    def disable(self, block):
        return self.client.disable(block)

    def shutdown(self, block):
        return self.client.shutdown(block)

    def stop(self, block):
        return self.client.stop(block)

    def pause(self, block):
        return self.client.pause(block)

    def resume(self, block):
        return self.client.resume(block)

    def run_program(self, name, block):
        return self.client.run_program(name, block)

    def set_tool_data(self, name, tool_offset, payload, inertia_tensor):
        return self.client.set_tool_data(name, tool_offset, payload, inertia_tensor)

    def get_tool_load(self):
        return self.client.get_tool_load()

    def get_tcp_offset(self):
        return self.client.get_tcp_offset()

    def set_wobj(self, name, wobj):
        return self.client.set_wobj(name, wobj)

    def get_wobj(self):
        return self.client.get_wobj()

    def set_wobj_offset(self, wobj):
        return self.client.set_wobj_offset(wobj)

    def cal_fkine(self, joints_position, tool, wobj):
        return self.client.cal_fkine(joints_position, tool, wobj)

    def cal_ikine(self, p, q_near, tool, wobj):
        return self.client.cal_ikine(p, q_near, tool, wobj)

    def set_standard_digital_out(self, num, value, block):
        return self.client.set_standard_digital_out(num, value, block)

    def set_tool_digital_out(self, num, value, block):
        return self.client.set_tool_digital_out(num, value, block)

    def get_standard_digital_in(self, num):
        return self.client.get_standard_digital_in(num)

    def get_tool_digital_in(self, num):
        return self.client.get_tool_digital_in(num)

    def get_config_digital_in(self, num):
        return self.client.get_config_digital_in(num)

    def get_standard_analog_voltage_in(self, num):
        return self.client.get_standard_analog_voltage_in(num)

    def get_tool_analog_voltage_in(self, num):
        return self.client.get_tool_analog_voltage_in(num)

    def get_standard_analog_current_in(self, num):
        return self.client.get_standard_analog_current_in(num)

    def read_raw_data_485(self, len):
        return self.client.read_raw_data_485(len)

    def read_raw_data_485_h(self, head, len):
        return self.client.read_raw_data_485_h(head, len)

    def read_raw_data_485_ht(self, head, tail):
        return self.client.read_raw_data_485_ht(head, tail)

    def write_raw_data_485(self, data):
        return self.client.write_raw_data_485(data)

    def write_raw_data_485_h(self, data, head):
        return self.client.write_raw_data_485_h(data, head)

    def write_raw_data_485_ht(self, data, head, tail):
        return self.client.write_raw_data_485_ht(data, head, tail)

    def tool_read_raw_data_485(self, len):
        return self.client.tool_read_raw_data_485(len)

    def tool_read_raw_data_485_h(self, head, len):
        return self.client.tool_read_raw_data_485_h(head, len)

    def tool_read_raw_data_485_ht(self, head, tail):
        return self.client.tool_read_raw_data_485_ht(head, tail)

    def tool_write_raw_data_485(self, data):
        return self.client.tool_write_raw_data_485(data)

    def tool_write_raw_data_485_h(self, data, head):
        return self.client.tool_write_raw_data_485_h(data, head)

    def tool_write_raw_data_485_ht(self, data, head, tail):
        return self.client.tool_write_raw_data_485_ht(data, head, tail)

    def read_raw_data_can(self):
        return self.client.read_raw_data_can()

    def write_raw_data_can(self, id, data):
        return self.client.write_raw_data_can(id, data)

    def read_bool_reg(self, num):
        return self.client.read_bool_reg(num)

    def read_word_reg(self, num):
        return self.client.read_word_reg(num)

    def read_float_reg(self, num):
        return self.client.read_float_reg(num)

    def write_bool_reg(self, num, value):
        return self.client.write_bool_reg(num, value)

    def write_word_reg(self, num, value):
        return self.client.write_word_reg(num, value)

    def write_float_reg(self, num, value):
        return self.client.write_float_reg(num, value)

    def movej(self, joints_list, v, a, r, block, op=op_):
        return self.client.movej(joints_list, v, a, r, block, op)

    def movej2(self, joints_list, v, a, r, block, op=op_):
        return self.client.movej2(joints_list, v, a, r, block, op)

    def movej_pose(self, p, v, a, r, q_near, tool, wobj, block, op=op_):
        return self.client.movej_pose(p, v, a, r, q_near, tool, wobj, block, op)

    def movej_pose2(self, p, v, a, r, q_near, tool, wobj, block, op=op_):
        return self.client.movej_pose2(p, v, a, r, q_near, tool, wobj, block, op)

    def movel(self, p, v, a, r, q_near, tool, wobj, block, op=op_):
        return self.client.movel(p, v, a, r, q_near, tool, wobj, block, op)

    def movec(self, p1, p2, v, a, r, mode, q_near, tool, wobj, block, op=op_):
        return self.client.movec(p1, p2, v, a, r, mode, q_near, tool, wobj, block, op)

    def tcp_move(self, pose_offset, v, a, r, block, op=op_):
        return self.client.tcp_move(pose_offset, v, a, r, block, op)

    def tcp_move_2p(self, p1, p2, v, a, r, block, op=op_):
        return self.client.tcp_move_2p(p1, p2, v, a, r, block, op)

    def spline(self, pose_list, v, a, tool, wobj, block, op=op_):
        return self.client.spline(pose_list, v, a, tool, wobj, block, op)

    def speedj(self, joints_list, a, time, block):
        return self.client.speedj(joints_list, a, time, block)

    def speedl(self, pose_list, a, time, block):
        return self.client.speedl(pose_list, a, time, block)

    def speed_stop(self, block):
        return self.client.speed_stop(block)

    def servoj(self, joints_list, v, a, kp=200, kd=20):
        return self.client.servoj(joints_list, v, a, kp, kd)

    def servoj_pose(self, pose_list, v, a, q_near, tool, wobj, kp=200, kd=20):
        return self.client.servoj_pose(pose_list, v, a, q_near, tool, wobj, kp, kd)

    def teach_mode(self, block):
        return self.client.teach_mode(block)

    def end_teach_mode(self, block):
        return self.client.end_teach_mode(block)

    def modbus_add_signal(self, ip, slave_number, signal_address, signal_type, signal_name):
        return self.client.modbus_add_signal(ip, slave_number, signal_address, signal_type, signal_name)

    def modbus_delete_signal(self, signal_name):
        return self.client.modbus_delete_signal(signal_name)

    def modbus_read(self, signal_name):
        return self.client.modbus_read(signal_name)

    def modbus_write(self, signal_name, value):
        return self.client.modbus_write(signal_name, value)

    def modbus_set_frequency(self, signal_name, frequence):
        return self.client.modbus_set_frequency(signal_name, frequence)

    def get_last_error(self):
        return self.client.get_last_error()

    def get_noneblock_taskstate(self, id):
        return self.client.get_noneblock_taskstate(id)

    def log_info(self, message):
        self.client.log_info(message)

    def log_error(self, message):
        self.client.log_error(message)

    def simulation(self, sim, block):
        return self.client.simulation(sim, block)

    def speed(self, val):
        return self.client.speed(val)

    def get_robot_state(self):
        return self.client.get_robot_state()

    def get_flange_pose(self):
        return self.client.get_flange_pose()

    def get_flange_speed(self):
        return self.client.get_flange_speed()

    def get_flange_acceleration(self):
        return self.client.get_flange_acceleration()

    def get_tcp_pose(self):
        return self.client.get_tcp_pose()

    def get_tcp_speed(self):
        return self.client.get_tcp_speed()

    def get_tcp_acceleration(self):
        return self.client.get_tcp_acceleration()

    def get_tcp_force(self):
        return self.client.get_tcp_force()

    def get_actual_joints_position(self):
        return self.client.get_actual_joints_position()

    def get_target_joints_position(self):
        return self.client.get_target_joints_position()

    def get_actual_joints_speed(self):
        return self.client.get_actual_joints_speed()

    def get_target_joints_speed(self):
        return self.client.get_target_joints_speed()

    def get_actual_joints_acceleration(self):
        return self.client.get_actual_joints_acceleration()

    def get_target_joints_acceleration(self):
        return self.client.get_target_joints_acceleration()

    def get_actual_joints_torque(self):
        return self.client.get_actual_joints_torque()

    def get_target_joints_torque(self):
        return self.client.get_target_joints_torque()

    def stop_record_track(self):
        return self.client.stop_record_track()

    def start_record_track(self, name):
        return self.client.start_record_track(name)

    def collision_detect(self, value):
        return self.client.collision_detect()

    def replay(self, name, value):
        return self.client.replay(name, value)

    def set_load_data(self, value):
        return self.client.set_load_data(value)

    def fc_start(self):
        return self.client.fc_start()

    def fc_stop(self):
        return self.client.fc_stop()

    def fc_config(self, direction, ref_ft, damp, max_vel, num_list,toolname, wobjname, value):
        return self.client.fc_config(direction, ref_ft, damp, max_vel, num_list, toolname, wobjname, value)

    def fc_move(self):
        return self.client.fc_move()

    def fc_guard_act(self, direction, ref_ft, toolname, wobjname, type):
        return self.client.fc_guard_act(direction, ref_ft, toolname, wobjname, type)

    def fc_guard_deact(self):
        return self.client.fc_guard_deact()

    def fc_force_set_value(self, direction, ref_ft):
        return self.client.fc_force_set_value(direction, ref_ft)

    def fc_wait_pos(self, middle, range, absolute, duration, timeout):
        return self.client.fc_wait_pos(middle, range, absolute, duration, timeout)

    def fc_wait_vel(self, middle, range, absolute, duration, timeout):
        return self.client.fc_wait_vel(middle, range, absolute, duration, timeout)

    def fc_wait_ft(self, middle, range, absolute, duration, timeout):
        return self.client.fc_wait_ft(middle, range, absolute, duration, timeout)

    def fc_wait_logic(self, value):
        return self.client.fc_wait_logic(value)

    def fc_get_ft(self):
        return self.client.fc_get_ft()

    def fc_mode_is_active(self):
        return self.client.fc_mode_is_active()

    def start_realtime_mode(self):
        return self.client.start_realtime_mode()

    def end_realtime_mode(self):
        return self.client.end_realtime_mode()

    def enable_speed_optimization(self):
        return self.client.enable_speed_optimization()

    def disable_speed_optimization(self):
        return self.client.disable_speed_optimization()
    
    def trackEnqueue(self, track, block):
        return self.client.trackEnqueue(track, block)
    
    def trackClearQueue(self):
        return self.client.trackClearQueue()
    
    def getQueueSize(self):
        return self.client.getQueueSize()
    
    def trackJointMotion(self, speed, acc, block):
        return self.client.trackJointMotion(speed, acc, block)
    
    def trackCartMotion(self, speed, acc, block, tool, wobj):
        return self.client.trackCartMotion(speed, acc, block, tool, wobj, 0.002)
    
    def rpc_heartbeat(self):
        self.client.rpc_heartbeat()
            
