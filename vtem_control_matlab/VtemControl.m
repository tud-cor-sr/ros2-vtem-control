classdef VtemControl < handle
   properties
      DeviceAddress_
      Port_
      ctx_
      connected_ = false;
      addr_input_start_ = 45392;
      addr_output_start_ = 40001;
      cpx_input_offset_ = 3;
      cpx_output_offset_ = 2;
      num_slots_ = 8;
   end
   methods
      function obj = VtemControl(DeviceAddress, Port)
         obj.DeviceAddress_ = DeviceAddress;
         obj.Port_ = Port;
      end
      function connect(obj)
         obj.ctx_ = modbus('tcpip', obj.DeviceAddress_, obj.Port_);
         obj.connected_ = true;
      end
      function disconnect(obj)
         % clear obj.ctx_;
         delete(obj.ctx_);
         obj.connected_ = false;
      end
      function ensure_connection(obj)
         if obj.connected_ ~= true
             throw(MException("VtemControl:ensure_connection", "Operation requires a modbus connection."))
         end
      end
      function [value, bits_str] = read_address(obj, addr)
         value = read(obj.ctx_, 'holdingregs', addr, 1);
         bits = bitget(value, 8:-1:1);
         bits_str = num2str(bits);
         bits_str(isspace(bits_str)) = '';
      end
      function [motion_app_id, valve_state] = get_single_motion_app(obj, slotIdx)
         obj.ensure_connection();
          
         % we read two bytes for all 3 entries 
         % (e.g. status, actual value 1, actual value 2)
         addr = obj.addr_input_start_ + obj.cpx_input_offset_ + 2*3*slotIdx;
         % this gives us an array containing two bytes
         status = read(obj.ctx_, 'holdingregs', addr, 2);
         
%          addr1 = obj.addr_input_start_ + obj.cpx_input_offset_ + 2*3*slotIdx
%          byte1 = read(obj.ctx_, 'holdingregs', addr1, 1);
%          byte1_bits = bitget(byte1, 8:-1:1)
%          addr2 = obj.addr_input_start_ + obj.cpx_input_offset_ + 2*3*slotIdx + 1
%          byte2 = read(obj.ctx_, 'holdingregs', addr2, 1);
%          byte2_bits = bitget(byte2, 8:-1:1)
         
%          byte1_bits = bitget(status(2), 8:-1:1)
%          byte2_bits = bitget(status(1), 8:-1:1)
         
         % somehow, the second byte seems to be stored in the first element
         % of the array and vice-versa
         motion_app_id_bits = bitget(status(2), 6:-1:1);
         motion_app_id = bit2dec(motion_app_id_bits);
         
         valve_state_bits = bitget(status(2), 8:-1:7);
         valve_state = bit2dec(valve_state_bits);
         
         app_state_bits = bitget(status(1), 8:-1:1);
         app_state = bit2dec(app_state_bits);
      end
      function set_single_motion_app(obj, slotIdx, motion_app_id, app_control)
         obj.ensure_connection();
          
         app_option = 0;
         
         motion_app_id_bin = bitget(motion_app_id, 6:-1:1);
         app_control_bin = bitget(app_control, 2:-1:1);
         app_option_bin = bitget(app_option, 8:-1:1);
         
         command_bits = [app_option_bin app_control_bin motion_app_id_bin];
         command = bit2dec(command_bits);
         
         addr = obj.addr_output_start_ + obj.cpx_output_offset_ + 3*slotIdx;
         write(obj.ctx_, 'holdingregs', addr, command);
      end
      function set_all_motion_apps(obj, motion_app_id, app_control)
          for slotIdx = 0:1:(obj.num_slots_-1)
              obj.set_single_motion_app(slotIdx, motion_app_id, app_control);
          end
      end
      function ensure_motion_app(obj, slotIdx, des_motion_app_id, des_valve_state)
         [motion_app_id, valve_state] = get_single_motion_app(obj, slotIdx);
          
         if motion_app_id ~= des_motion_app_id
             throw(MException("VtemControl:ensure_motion_app", "Operation requires activating the suitable motion app."))
         end
         
         if valve_state ~= des_valve_state
             throw(MException("VtemControl:ensure_motion_app", "Operation requires setting the desired valve state."))
         end
      end
      function activate_pressure_regulation_single_slot(obj, slotIdx)
         obj.set_single_motion_app(slotIdx, 3, 3);
      end
      function deactivate_pressure_regulation_single_slot(obj, slotIdx)
         obj.set_single_motion_app(slotIdx, 61, 0);
      end
      function activate_pressure_regulation_all_slots(obj)
         obj.set_all_motion_apps(3, 3);
      end
      function deactivate_pressure_regulation_all_slots(obj)
         obj.set_all_motion_apps(61, 0);
      end
      function [slotIdx, slotRemain] = get_slot_idx_from_valve_idx(valveIdx)
          slotIdx = floor(valveIdx);
          slotRemain = valveIdx - 2*slotIdx;
      end
      function value = get_single_pressure(obj, valveIdx)
         obj.ensure_connection(); 
         
         [slotIdx, slotRemain] = obj.get_slot_idx_from_valve_idx(valveIdx);
         
         obj.ensure_motion_app(slotIdx, 3, 3);
          
         addr = obj.addr_input_start_ + obj.cpx_input_offset_ + 2*3*slotIdx + 1 + slotRemain;
         bytes = read(obj.ctx_, 'holdingregs', addr, 2);

         bits = [bitget(bytes(1), 8:-1:1) bitget(bytes(2), 8:-1:1)];
         value = bit2dec(bits);
      end
      function set_single_pressure(obj, valveIdx, value)
         obj.ensure_connection();
         
         [slotIdx, slotRemain] = obj.get_slot_idx_from_valve_idx(valveIdx);
         
         obj.ensure_motion_app(slotIdx, 3, 3);
          
         addr = obj.addr_output_start_ + obj.cpx_output_offset_ + 3*slotIdx + 1 + slotRemain;
         write(obj.ctx_, 'holdingregs', addr, value);
      end
      function pressures = get_all_pressures(obj)
          pressures = zeros(2*obj.num_slots_, 1);
          for valveIdx = 0:1:(2*obj.num_slots_-1)
              pressures(valveIdx + 1) = obj.get_single_pressure(valveIdx);
          end
      end
      function set_all_pressures(obj, pressures)
          for valveIdx = 0:1:(2*obj.num_slots_-1)
              obj.set_single_pressure(valveIdx, pressures(valveIdx + 1));
          end
      end
   end
end