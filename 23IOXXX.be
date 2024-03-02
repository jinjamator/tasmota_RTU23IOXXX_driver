# class serial
#   static SERIAL_8N1 ="SERIAL_8N1"
#   def init(gpio_rx,gpio_tx,speed,fmt)
#     debug("fake serial init")
#     debug(gpio_rx,gpio_tx,speed,fmt)
#   end
#   def read()
#     debug("called read")
#     return bytes("010304091C00003869")
#   end
#   def write(data)
#     debug("called write " + data.tohex())
#   end
# end 





LOW_BYTES = bytes("00C0C101C30302C2C60607C705C5C404CC0C0DCD0FCFCE0E0ACACB0BC90908C8D81819D91BDBDA1A1EDEDF1FDD1D1CDC14D4D515D71716D6D21213D311D1D010F03031F133F3F23236F6F737F53534F43CFCFD3DFF3F3EFEFA3A3BFB39F9F83828E8E929EB2B2AEAEE2E2FEF2DEDEC2CE42425E527E7E62622E2E323E12120E0A06061A163A3A26266A6A767A56564A46CACAD6DAF6F6EAEAA6A6BAB69A9A86878B8B979BB7B7ABABE7E7FBF7DBDBC7CB47475B577B7B67672B2B373B17170B0509091519353529296565797559594549C5C5D9D5F9F9E5E5A9A9B5B99595898884849894B8B8A4A4E8E8F4F8D4D4C8C44848545874746868242438341818040")
HIGH_BYTES = bytes("00C1814001C0804101C0804100C1814001C0804100C1814000C1814001C0804101C0804100C1814000C1814001C0804100C1814001C0804101C0804100C1814001C0804100C1814000C1814001C0804100C1814001C0804101C0804100C1814000C1814001C0804101C0804100C1814001C0804100C1814000C1814001C0804101C0804100C1814000C1814001C0804100C1814001C0804101C0804100C1814000C1814001C0804101C0804100C1814001C0804100C1814000C1814001C0804100C1814001C0804101C0804100C1814001C0804100C1814000C1814001C0804101C0804100C1814000C1814001C0804100C1814001C0804101C0804100C18140")


def modbus_crc16(data)
    var crc_high = 0xFF
    var crc_low = 0xFF
    for i: 0 .. size(data)-1
        var byte = data[i]
        var index = crc_high ^ int(byte)
        crc_high = crc_low ^ HIGH_BYTES[index]
        crc_low = LOW_BYTES[index]
    end
    return [crc_low,crc_high]
end

def get_modbus_packet(data)
    if classname(data) != "bytes"
        data=bytes(data)
    end
    var crc=modbus_crc16(data)
    data.add(crc[1],1)
    data.add(crc[0],1)
    return data
end

def debug(msg)
end




class ModbusRTU
    var speed,gpio_rx,gpio_tx, ser
    var devices
    var rec_buffer
  
  def init(gpio_rx, gpio_tx, speed, fmt)
    debug("bus_init")
    self.speed=speed
    self.gpio_rx=gpio_rx
    self.gpio_tx=gpio_tx
    self.ser=serial(gpio_rx,gpio_tx,speed,fmt)
    self.devices = {}
    self.rec_buffer=bytes()
  end

  def add_device(device)
    device.bus=self
    debug(device)
    debug("bus added device. ID " + str(device.id) + " type " +str(classname(device)))
    self.devices[device.id]=device
    device.attached_to_bus_event()
  end

  def send(data)
    var packet = get_modbus_packet(data)
    self.ser.write(packet)
  end

  def parse_msg_from_buffer()
    
    var buf_size=size(self.rec_buffer)
    if self.rec_buffer[0] > 0 && self.rec_buffer[0] < 248 && self.devices.find(self.rec_buffer[0])
      debug("found valid id " + str(self.rec_buffer[0]))
      if self.rec_buffer[1] > 80
        debug("error")
      end

      if self.rec_buffer[1] == 3
        debug("Function 03, Read Holding Registers")
        var len=self.rec_buffer[2]
        if size(self.rec_buffer) < len + 5
          debug("message incomplete " + str(self.rec_buffer))
          return nil
        end
        var data=self.rec_buffer[3 .. len + 2]
        var crc=self.rec_buffer[len + 3 .. len + 4]
        var calc_crc=modbus_crc16(self.rec_buffer[0..len + 2])

        if crc[0] == calc_crc[1]  && crc[1] == calc_crc[0]
          debug("CRC is valid")
          
          self.devices[self.rec_buffer[0]].command3_received(data)
          self.rec_buffer.clear()
        else
          debug("CRC is invalid, dropping buffer")
          debug(self.rec_buffer)
          self.rec_buffer.clear()
          return nil
        end
      end
    else
      debug("flush buffer as we don't know how to handle the data")
      debug(self.rec_buffer)
      self.rec_buffer.clear()
      return nil
    end
    
  end

  def receive()
    var msg=self.ser.read()
    self.rec_buffer+=msg
    if size(self.rec_buffer) > 5
      self.parse_msg_from_buffer()
    end
  end
  
  def every_250ms()
    self.receive()
  end

end

class MODBusDevice
  var id, bus, next_handler3
  def init(slave_id)
    self.id=slave_id
  end

  def command3_received(data)
    debug("not yet implemented. ID " + str(self.id) + " received command 3, data:" + data.tohex())
  end

  def send(command,register,data)
    self.bus.send(  bytes().add(self.id) + bytes(command) + bytes(register) + bytes(data))
  end

  def attached_to_bus_event()
    debug("attached_to_bus_event not implemented yet")
  end


end

class RTU23IOXXX: MODBusDevice
  
  var pid
  var io_count
  var inputs_state

  def init(slave_id)
    super(self).init(slave_id)
    self.inputs_state=bytes("00000000")
  end

  def attached_to_bus_event()
    self.next_handler3=self.detect
    self.send("03", "00F7", "000275F9")

  end

  def detect(data)
    self.pid=data.geti(0,-2)
    self.io_count=self.pid-2300
    print("found a eletechsup 23IOD" +str(self.io_count) + " at address " + str(self.id))
    # tasmota.global.devices_present += self.io_count

  end

  def command3_received(data)
    debug(data)
    if self.next_handler3
      self.next_handler3(self,data)
      self.next_handler3=nil
    else
      debug("no handler registred -> ignoring data " + str(data))
    end

  end

  def update_inputs(data)
    # 00 00 80 00
      var idx_map
      for i: 0..3
        var offset=list(1,0,3,2)[i]
        debug("offset " + str(offset) )
        if self.inputs_state[offset] == data[offset]
          continue
        end
        for bidx: 0..7
          if (data[offset] & 0x01 << bidx) != (self.inputs_state[offset] & 0x01 << bidx)
            print("change input " + str((i*8)+(bidx + 1)))
          end
        end
    end
    self.inputs_state=data
  end

  def poll_all_inputs()
    debug("polling all inputs")
    self.next_handler3=self.update_inputs
    self.send("03","00C0","0002")
  end

  def every_250ms()
    self.poll_all_inputs()
  end

end

bus=ModbusRTU(16, 13, 9600, serial.SERIAL_8N1)
dev1=RTU23IOXXX(1)
bus.add_device(dev1)
tasmota.add_driver(bus)
tasmota.add_driver(dev1)


# bus.receive()
# dev1.poll_all_inputs()
# dev1.update_inputs(bytes("00000000"))
# dev1.update_inputs(bytes("FF000000"))
# dev1.update_inputs(bytes("FE000000"))






    
