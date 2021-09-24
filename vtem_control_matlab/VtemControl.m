classdef VtemControl < handle
   properties
      DeviceAddress_
      Port_
      ctx_
      addr_input_start_ = 45392;
      addr_output_start_ = 40001;
      cpx_input_offset_ = 3;
      cpx_output_offset_ = 2;
      num_valves_ = 16;
   end
   methods
      function obj = VtemControl(DeviceAddress, Port)
         obj.DeviceAddress_ = DeviceAddress;
         obj.Port_ = Port;
      end
      function connect(obj)
         obj.ctx_ = modbus('tcpip', obj.DeviceAddress_, obj.Port_);
      end
      function disconnect(obj)
         % clear obj.ctx_;
         delete(obj.ctx_);
      end
      function [motion_app_id, valve_state] = get_single_motion_app(obj, slotIdx)
         addr = obj.addr_input_start_ + obj.cpx_input_offset_ + 3*slotIdx;
         status = read(obj.ctx_, 'holdingregs', addr, 1);
         
         motion_app_id_bits = bitget(status, 6:-1:1);
         motion_app_id = bit2dec(motion_app_id_bits);
         
         valve_state_bits = bitget(status, 8:-1:7);
         valve_state = bit2dec(valve_state_bits);
         
         app_state_bits = bitget(status, 16:-1:9);
         app_state = bit2dec(app_state_bits);
      end
      function set_single_motion_app(obj, slotIdx, motion_app_id, app_control)
         app_option = 0;
         
         motion_app_id_bin = bitget(motion_app_id, 6:-1:1);
         app_control_bin = bitget(app_control, 2:-1:1);
         app_option_bin = bitget(app_option, 8:-1:1);
         
         command_bits = [app_option_bin app_control_bin motion_app_id_bin];
         command = bit2dec(command_bits);
         
         addr = obj.addr_output_start_ + obj.cpx_output_offset_ + 3*slotIdx;
         write(obj.ctx_, 'holdingregs', addr, command);
      end
      function value = get_single_pressure(obj, idx)
         addr = obj.addr_input_start_ + obj.cpx_input_offset_ + 1 + 3*idx;
         value = read(obj.ctx_, 'holdingregs', addr, 1);
      end
      function data = get_all_pressures(obj)
         addr = obj.addr_input_start_ + obj.cpx_input_offset_;
         data = read(obj.ctx_, 'holdingregs', addr, 3*obj.num_valves_);
      end
      function set_single_pressure(obj, idx, value)
         addr = obj.addr_output_start_ + obj.cpx_output_offset_ + idx;
         write(obj.ctx_, 'holdingregs', addr, value);
      end
      function set_all_pressures(obj, data)
         addr = obj.addr_output_start_ + obj.cpx_output_offset_;
         write(obj.ctx_, 'holdingregs', addr, data);
      end
   end
end