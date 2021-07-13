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
      function value = get_single_pressure(obj, idx)
         addr = obj.addr_input_start_ + obj.cpx_input_offset_ + 3*idx;
         value = read(obj.ctx_, 'inputregs', addr, 1);
      end
      function data = get_all_pressures(obj)
         addr = obj.addr_input_start_ + obj.cpx_input_offset_;
         data = read(obj.ctx_, 'inputregs', addr, 3*obj.num_valves_);
      end
      function set_single_pressure(obj, idx, value)
         addr = obj.addr_output_start_ + obj.cpx_output_offset_ + idx;
         disp(addr)
         write(obj.ctx_, 'holdingregs', addr, value);
      end
      function set_all_pressures(obj, data)
         addr = obj.addr_output_start_ + obj.cpx_output_offset_;
         write(obj.ctx_, 'holdingregs', addr, data);
      end
   end
end