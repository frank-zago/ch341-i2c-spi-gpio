-- SPDX-License-Identifier: GPL-2.0-or-later
-- Copyright 2023, Frank Zago

-- Wireshark dissector for the CH341 in I2C / SPI / GPIO mode
-- Not for the UART or printer mode

-- Invoke with:
--    tshark -Xlua_script:ch341-i2c-dissector.lua -r xxxx.pcap -V
--    wireshark -Xlua_script:ch341-i2c-dissector.lua -r xxxx.pcap

-- Under Wireshark, if the LUA script is modified, it can be reloaded
-- with ctrl+shit+L

-- Caveats:
--   . The Wireshark trace must include the USB "GET DESCRIPTOR" response
--     for the device to be identified and this dissector used.
--   . There are holes in the decoding, especially regarding what bytes
--     are available.
--   . The Wireshark LUA interface doesn't allow to match response
--     with request, so the status won't be decoded for instance.
--   . A C version could be better, but this dissector can still
--     decode about 7500 packets per second.


-- Bitwise AND. Unfortunately Wireshark on Fedora Linux ships with an
-- antique version that doesn't support any kind of bitwise
-- operations. Works on an octet only, which is all that's needed
-- here.
function band(x, y)
   res = 0
   mask = 128
   for i=0,7 do
      if x >= mask then
	 bit_x = 1
	 x = x - mask
      else
	 bit_x = 0
      end
      if y >= mask then
	 bit_y = 1
	 y = y - mask
      else
	 bit_y = 0
      end
      mask = math.floor(mask / 2)
      res = res * 2
      if bit_x == 1 and bit_y == 1 then
	 res = res + 1
      end
   end
   return res
end


local ch341_serial_protocol = Proto("ch341",  "WCH ch341 I2C/SPI/GPIO")
local usb_direction = Field.new("usb.endpoint_address.direction")
local usb_transfer_type = Field.new("usb.transfer_type")

-- Segment commands
local PARA_STATUS = 0xa0
local PARA_SET_CFG = 0xa1
local PARA_CMD_W0 = 0xa6
local PARA_CMD_W1 = 0xa7
local SPI_STREAM  = 0xa8
local I2C_STREAM  = 0xaa
local UIO_STREAM  = 0xab
local PARA_CMD_R0 = 0xac
local PARA_CMD_R1 = 0xad

--
-- I2C
--

local I2C_STM_END = 0x00
local I2C_STM_US  = 0x40
local I2C_STM_MS  = 0x50
local I2C_STM_SET = 0x60
local I2C_STM_STA = 0x74
local I2C_STM_STO = 0x75
local I2C_STM_OUT = 0x80
local I2C_STM_IN  = 0xC0

local i2c_speed = {
   [0] = "20kHz",
   [1] = "100kHz",
   [2] = "400kHz",
   [3] = "750kHz"
}

function i2c_stream(buffer, index, length, tree)

   tree:add(buffer(index, 1), "I2C streaming start")
   length = length - 1
   index = index + 1

   while length > 0 do

      local sub_cmd = buffer(index, 1):uint()

      if sub_cmd == I2C_STM_END then
	 tree:add(buffer(index, 1), "I2C streaming end")
	 return

      elseif sub_cmd == I2C_STM_STA then
	 tree:add(buffer(index, 1), "I2C START")
	 length = length - 1
	 index = index + 1

      elseif sub_cmd == I2C_STM_STO then
	 tree:add(buffer(index, 1), "I2C STOP")
	 length = length - 1
	 index = index + 1

      elseif sub_cmd >= I2C_STM_OUT and sub_cmd <= (I2C_STM_OUT + 0x1f) then
	 data_len = sub_cmd - I2C_STM_OUT
	 sub_cmd = I2C_STM_OUT

	 if data_len == 0 then
	    -- Note: 0x80 is special but it still includes the address
	    data_len = 1
	    out_tree = tree:add(buffer(index, 1 + data_len), "I2C stream out -- status requested")
	 else
	    out_tree = tree:add(buffer(index, 1 + data_len), "I2C stream out")
	 end

	 index = index + 1
	 length = length - 1

	 -- The address
	 if length > 0 and data_len > 0 then
	    local addr = buffer(index, 1):uint()
	    out_tree:add(buffer(index, 1), "I2C address: " .. string.format("0x%x", addr / 2) .. "  R/W bit: " .. addr % 2)
	    index = index + 1
	    length = length - 1
	    data_len = data_len - 1
	 else
	    return
	 end

	 if data_len > 0 then
	    out_tree:add(buffer(index, data_len), "Data")

	    index = index + data_len
	    length = length - data_len

	    if length < 0 then
	       return
	    end
	 end

      elseif sub_cmd >= I2C_STM_IN and sub_cmd <= (I2C_STM_IN + 0x20) then
	 data_len = sub_cmd - I2C_STM_IN
	 sub_cmd = I2C_STM_IN

	 if data_len == 0 then
	    -- 0 means 1 too.
	    data_len = 1
	 end

	 tree:add(buffer(index, 1), "I2C stream in: " .. data_len .. " bytes")

	 index = index + 1
	 length = length - 1

      elseif sub_cmd >= I2C_STM_SET and sub_cmd <= (I2C_STM_SET + 0xf) then
	 data = sub_cmd - I2C_STM_SET
	 sub_cmd = I2C_STM_SET
	 tree:add(buffer(index, 1), "I2C stream settings: speed " .. i2c_speed[band(data, 3)])

	 index = index + 1
	 length = length - 1

      elseif sub_cmd >= I2C_STM_MS and sub_cmd <= I2C_STM_MS + 0xf then
	 local delay = sub_cmd - I2C_STM_MS
	 tree:add(buffer(index, 1), "I2C " .. delay .. "ms delay")
	 length = length - 1
	 index = index + 1

      else
	 tree:add(buffer(index, length), "bad data " .. string.format("0x%x", sub_cmd))
	 return 0
      end
   end
end

--
-- Pins and GPIO
--

local UIO_STM_IN  = 0x00
local UIO_STM_END = 0x20
local UIO_STM_DIR = 0x40
local UIO_STM_OUT = 0x80
local UIO_STM_US  = 0xC0

function para_status(buffer, index, length, tree)
   tree:add(buffer(index, 1), "request pins status")
end

-- Decode GPIO pin filter. Which bytes are valid in the message.
function gpio_cfg_filter(filter)
   str = ""
   if band(filter, 0x01) == 0x01 then
      str = str .. "DATA[8..15] "
   end

   if band(filter, 0x02) == 0x02 then
      str = str .. "DIR[8..15] "
   end

   if band(filter, 0x04) == 0x04 then
      str = str .. "DATA[0..7] "
   end

   if band(filter, 0x08) == 0x08 then
      str = str .. "DIR[0..7] "
   end

   if band(filter, 0x010) == 0x10 then
      str = str .. "DATA[16..19] "
   end

   return str
end

function para_set_cfg(buffer, index, length, tree)
   tree:add(buffer(index, 1), "set pins configuration")
   length = length - 1
   index = index + 1

   -- Always 0x6a. 0x60 + 0xa?
   tree:add(buffer(index, 1), "one byte of stuff")

   length = length - 1
   index = index + 1

   tree:add(buffer(index, 1), "valid:" .. gpio_cfg_filter(buffer(index, 1):uint()))
   length = length - 1
   index = index + 1

   tree:add(buffer(index, 1), "DATA[8..15]")
   index = index + 1
   tree:add(buffer(index, 1), "DIR[8..15]")
   index = index + 1
   tree:add(buffer(index, 1), "DATA[0..7]")
   index = index + 1
   tree:add(buffer(index, 1), "DIR[0..7]")
   index = index + 1
   tree:add(buffer(index, 1), "DATA[16..19]")
   index = index + 1
   length = length - 5

   tree:add(buffer(index, length), "other stuff")
end

function uio_stream(buffer, index, length, tree)
   tree:add(buffer(index, 1), "UIO streaming start")
   length = length - 1
   index = index + 1

   while length > 0 do
      local sub_cmd = buffer(index, 1):uint()

      if sub_cmd == UIO_STM_END then
	 tree:add(buffer(index, 1), "UIO streaming end")
	 return

      elseif sub_cmd >= UIO_STM_OUT and sub_cmd <= UIO_STM_OUT + 0x3f then
	 local mask = sub_cmd - UIO_STM_OUT
	 tree:add(buffer(index, 1), "UIO write pins")
	 length = length - 1
	 index = index + 1

      elseif sub_cmd == UIO_STM_IN then
	 tree:add(buffer(index, 1), "UIO read pins")
	 length = length - 1
	 index = index + 1

      elseif sub_cmd >= UIO_STM_DIR and sub_cmd <= UIO_STM_DIR + 0x3f then
	 local mask = sub_cmd - UIO_STM_DIR
	 tree:add(buffer(index, 1), "UIO set pins direction")
	 length = length - 1
	 index = index + 1

      else
	 tree:add(buffer(index, length), "bad data " .. string.format("0x%x", sub_cmd))
	 return 0
      end
   end
end

--
-- SPI
--
function spi_stream(buffer, index, length, tree)
   tree:add(buffer(index, 1), "SPI streaming")
   length = length - 1
   index = index + 1

   tree:add(buffer(index, length), "Data")
end

--
-- EPP parallel port
--
function para_stream(buffer, index, length, tree)
   tree:add(buffer(index, 1), "Parallel streaming")
   length = length - 1
   index = index + 1

   tree:add(buffer(index, length), "Data")
end

--
-- CORE
--

local commands = {
   [I2C_STREAM]   = i2c_stream,
   [PARA_STATUS]  = para_status,
   [PARA_SET_CFG] = para_set_cfg,
   [UIO_STREAM]   = uio_stream,
   [SPI_STREAM]   = spi_stream,
   [PARA_CMD_W0]  = para_stream,
   [PARA_CMD_R0 ] = para_stream,
   [PARA_CMD_W1]  = para_stream,
   [PARA_CMD_R1 ] = para_stream
}

function ch341_serial_protocol.dissector(buffer, pinfo, tree)

   -- Only requests with direction OUT are handled for now
   if pinfo.p2p_dir ~= 0 then
      return
   end

   if tonumber(tostring(usb_direction())) == 1 then
      return
   end

   -- Only BULK requests
   if tonumber(tostring(usb_transfer_type())) ~= 3 then
      return
   end

   length = buffer:len()
   if length == 0 then
      return
   end

   pinfo.cols.protocol = ch341_serial_protocol.name

   local subtree = tree:add(buffer(index, length), "CH341 Protocol Data")
   local index = 0

   -- Each 32-byte segment are independent and must be decoded
   -- individually. Some segments may contain just a few bytes of
   -- significant data, and junk afterwards.
   while length > 0 do
      local segment_length = length
      if segment_length > 32 then
	 segment_length = 32
      end

      local command = buffer(index, 1):uint()
      local cmd = commands[command]

      if cmd ~= nil then
	 cmd(buffer, index, segment_length, subtree)
      else
	 -- unsupported command - should print an error instead
	 tree:add(buffer(index, segment_length), "unsupported command " .. string.format("0x%x", command))
      end

      index = index + 32
      length = length - 32
   end
end

-- Register the dissector for the USB VID:PID of the CH341 in
-- I2C/SPI/GPIO mode.
DissectorTable.get("usb.product"):add(0x1a865512, ch341_serial_protocol)
