classdef VtemControl < handle
   properties
      DeviceAddress_
      Port_
      ctx_
      connected_ = false;
      addr_input_start_ = 45392;
      addr_output_start_ = 40001;
      cpx_input_offset_ = 4;
      cpx_output_offset_ = 3;
      num_slots_ = 8;
   end
   methods(Static)
      function [slotIdx, slotRemain] = get_slot_idx_from_valve_idx(valveIdx)
         slotIdx = floor(valveIdx / 2);
         slotRemain = valveIdx - 2*slotIdx;
      end
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
         value = read(obj.ctx_, 'holdingregs', addr, 1, 'int16');
         bits = bitget(value, 8:-1:1);
         bits_str = num2str(bits);
         bits_str(isspace(bits_str)) = '';
      end
      function [motion_app_id, valve_state] = get_single_motion_app(obj, slotIdx)
         obj.ensure_connection();
          
         % we read two bytes for all 3 entries 
         % (e.g. status, actual value 1, actual value 2)
         addr = obj.addr_input_start_ + obj.cpx_input_offset_ + 3*slotIdx;
         % this gives us an array containing two bytes
         status = read(obj.ctx_, 'holdingregs', addr, 1, 'int16');
         
%          addr1 = obj.addr_input_start_ + obj.cpx_input_offset_ + 2*3*slotIdx
%          byte1 = read(obj.ctx_, 'holdingregs', addr1, 1);
%          byte1_bits = bitget(byte1, 8:-1:1, 'int16')
%          addr2 = obj.addr_input_start_ + obj.cpx_input_offset_ + 2*3*slotIdx + 1
%          byte2 = read(obj.ctx_, 'holdingregs', addr2, 1);
%          byte2_bits = bitget(byte2, 8:-1:1, 'int16')
         
%          byte1_bits = bitget(status(2), 8:-1:1, 'int16')
%          byte2_bits = bitget(status(1), 8:-1:1, 'int16')
         
         % somehow, the second byte seems to be stored in the first element
         % of the array and vice-versa
         motion_app_id_bits = bitget(status, 6:-1:1, 'int16');
         motion_app_id = bit2dec(motion_app_id_bits);
         
         valve_state_bits = bitget(status, 8:-1:7, 'int16');
         valve_state = bit2dec(valve_state_bits);
         
         app_state_bits = bitget(status, 16:-1:9, 'int16');
         app_state = bit2dec(app_state_bits);
      end
      function set_single_motion_app(obj, slotIdx, motion_app_id, app_control)
         obj.ensure_connection();
          
         app_option = 0;
         
         motion_app_id_bin = bitget(motion_app_id, 6:-1:1, 'int16');
         app_control_bin = bitget(app_control, 2:-1:1, 'int16');
         app_option_bin = bitget(app_option, 8:-1:1, 'int16');
         
         command_bits = [app_option_bin app_control_bin motion_app_id_bin];
         command = bit2dec(command_bits);
         
         addr = obj.addr_output_start_ + obj.cpx_output_offset_ + 3*slotIdx;
         write(obj.ctx_, 'holdingregs', addr, command, 'int16');
      end
      function set_all_motion_apps(obj, motion_app_id, app_control)
          for slotIdx = 0:1:(obj.num_slots_-1)
              obj.set_single_motion_app(slotIdx, motion_app_id, app_control);
          end
      end
      function result = ensure_motion_app(obj, slotIdx, des_motion_app_id, des_valve_state, throw_exception)
         if nargin < 5
            throw_exception = true;
         end

         [motion_app_id, valve_state] = get_single_motion_app(obj, slotIdx);
          
         if motion_app_id ~= des_motion_app_id
             if throw_exception
                throw(MException("VtemControl:ensure_motion_app", "Operation requires activating the suitable motion app."))
             else
                result = false;
                return;
             end
         end
         
         if valve_state ~= des_valve_state
             if throw_exception
                throw(MException("VtemControl:ensure_motion_app", "Operation requires setting the desired valve state."))
             else
                result = false;
                return;
             end
         end

         result = true;
         return;
      end
      function acknowledge_errors_single_slot(obj, slotIdx)
         obj.set_single_motion_app(slotIdx, 62, 1);
         % Wait for errors to be acknowledged
         pause(0.5);
      end
      function acknowledge_errors_all_slots(obj)
         obj.set_all_motion_apps(62, 1);
         % Wait for errors to be acknowledged
         pause(0.5);
      end
      function result = activate_pressure_regulation_single_slot(obj, slotIdx)
         des_valve_mode = 3;
         des_app_control = 3;
         des_valve_state = 2;
         timeout = 20; % [s] duration during which we wait for the pressure regulation to deactivate
         sampling_rate = 20; % frequency with which we check the currently activate valve mode
         
         obj.set_single_motion_app(slotIdx, des_valve_mode, des_app_control);

         i = 0;
         config_mismatch = true;
         while config_mismatch
            if i / sampling_rate > timeout
                result = false;
                return;
            end

            [actual_valve_mode, valve_state] = obj.get_single_motion_app(slotIdx);
            
            if actual_valve_mode == des_valve_mode && valve_state == des_valve_state
                config_mismatch = false;
            end

            if config_mismatch
                i = i + 1;
                pause(1/sampling_rate);
            end
         end

         result = true;
         return;
      end
      function result = deactivate_pressure_regulation_single_slot(obj, slotIdx)
         des_valve_mode = 61;
         des_app_control = 0;
         des_valve_state = 1;
         timeout = 10; % [s] duration during which we wait for the pressure regulation to deactivate
         sampling_rate = 20; % frequency with which we check the currently activate valve mode
         exhaust_duration = 1; % [s] duration to let valve exhaust before shutting off motion app

         % Set valves to 0 bar (off).
         obj.set_single_pressure(2*slotIdx, 0);
         obj.set_single_pressure(2*slotIdx + 1, 0);

         % take some time to release pressure before shutting of the motion app
         pause(exhaust_duration);
         
         obj.set_single_motion_app(slotIdx, des_valve_mode, des_app_control);

         i = 0;
         config_mismatch = true;
         while config_mismatch
            if i / sampling_rate > timeout
                result = false;
                return;
            end

            [actual_valve_mode, valve_state] = obj.get_single_motion_app(slotIdx);
            
            if actual_valve_mode == des_valve_mode && valve_state == des_valve_state
                config_mismatch = false;
            end

            if config_mismatch
                i = i + 1;
                pause(1/sampling_rate);
            end
         end

         result = true;
         return;
      end
      function result = activate_pressure_regulation_all_slots(obj)
         des_valve_mode = 3;
         des_app_control = 3;
         des_valve_state = 2;
         timeout = 20; % [s] duration during which we wait for the pressure regulation to deactivate
         sampling_rate = 20; % frequency with which we check the currently activate valve mode
         
         obj.set_all_motion_apps(des_valve_mode, des_app_control);

         i = 0;
         config_mismatch = true;
         while config_mismatch
            if i / sampling_rate > timeout
                result = false;
                return;
            end

            config_mismatch = false;
            for slotIdx = 0:1:(obj.num_slots_-1)
                [actual_valve_mode, valve_state] = obj.get_single_motion_app(slotIdx);
                
                if actual_valve_mode ~= des_valve_mode || valve_state ~= des_valve_state
                    config_mismatch = true;
                end
            end

            if config_mismatch
                i = i + 1;
                pause(1 / sampling_rate);
            end
         end

         result = true;
         return;
      end
      function result = deactivate_pressure_regulation_all_slots(obj)
         des_valve_mode = 61;
         des_app_control = 0;
         des_valve_state = 1;
         timeout = 10; % [s] duration during which we wait for the pressure regulation to deactivate
         sampling_rate = 20; % frequency with which we check the currently activate valve mode
         exhaust_duration = 1; % [s] duration to let valve exhaust before shutting off motion app

         % Set valves to 0 bar (off).
         for slotIdx = 0:1:(obj.num_slots_-1)
            obj.set_single_pressure(2*slotIdx, 0);
            obj.set_single_pressure(2*slotIdx + 1, 0);
         end

         % take some time to release pressure before shutting of the motion app
         wait(exhaust_duration);
         
         obj.set_all_motion_apps(des_valve_mode, des_app_control);

         i = 0;
         config_mismatch = true;
         while config_mismatch
            if i / sampling_rate > timeout
                result = false;
                return;
            end

            config_mismatch = false;
            for slotIdx = 0:1:(obj.num_slots_-1)
                [actual_valve_mode, valve_state] = obj.get_single_motion_app(slotIdx);
                
                if actual_valve_mode ~= des_valve_mode || valve_state ~= des_valve_state
                    config_mismatch = true;
                end
            end

            if config_mismatch
                i = i + 1;
                pause(1 / sampling_rate);
            end
         end

         result = true;
         return;
      end
      function value = get_single_pressure(obj, valveIdx)
         obj.ensure_connection(); 
         
         [slotIdx, slotRemain] = VtemControl.get_slot_idx_from_valve_idx(valveIdx);
         
         if obj.ensure_motion_app(slotIdx, 3, 2, false) == false
             % we return zero pressure if the pressure regulation app is not active
            value = 0;
            return;
         end
          
         addr = obj.addr_input_start_ + obj.cpx_input_offset_ + 3*slotIdx + 1 + slotRemain;
         bytes = read(obj.ctx_, 'holdingregs', addr, 1, 'int16');

         bits = bitget(bytes, 16:-1:1, 'int16');
         value = bit2dec(bits);
      end
      function set_single_pressure(obj, valveIdx, value)
         obj.ensure_connection();
         
         [slotIdx, slotRemain] = VtemControl.get_slot_idx_from_valve_idx(valveIdx);
         
         obj.ensure_motion_app(slotIdx, 3, 2);
          
         addr = obj.addr_output_start_ + obj.cpx_output_offset_ + 3*slotIdx + 1 + slotRemain;
         write(obj.ctx_, 'holdingregs', addr, value, 'int16');
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