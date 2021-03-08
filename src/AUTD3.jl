# File: AUTD3.jl
# Project: src
# Created Date: 11/02/2020
# Author: Shun Suzuki
# -----
# Last Modified: 09/03/2021
# Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
# -----
# Copyright (c) 2020 Hapis Lab. All rights reserved.
# 

module AUTD3

using StaticArrays

export ModSamplingFreq, ModBufSize, OptMethod
export Gain, Modulation, Sequence, Link, AUTD
export open_autd, dispose, add_device, calibrate, clear, set_silent_mode, stop, is_open, is_silent_mode, wavelength, set_wavelength
export num_devices, num_transducers, remaining_in_buffer
export enumerate_adapters, firmware_info_list
export focal_point_gain, grouped_gain, bessel_beam_gain, plane_wave_gain, custom_gain, holo_gain, transducer_test_gain, null_gain
export sine_modulation, modulation, custom_modulation, raw_pcm_modulation, saw_modulation, square_modulation, wav_modulation
export sequence, add_point, add_points, set_freq, get_freq, get_sampling_freq, get_sampling_freq_div, circum_sequence
export soem_link, twincat_link, local_twincat_link
export append_gain, append_gain_sync, append_modulation,append_modulation_sync, append_sequence
export append_stm_gain, start_stm, stop_stm, finish_stm
export flush
export device_direction, trans_position, device_idx_for_trans_idx

include("NativeMethods.jl")

@enum ModSamplingFreq begin
    SMPL_125_HZ = 125
    SMPL_250_HZ = 250
    SMPL_500_HZ = 500
    SMPL_1_KHZ = 1000
    SMPL_2_KHZ = 2000
    SMPL_4_KHZ = 4000
    SMPL_8_KHZ = 8000
end

@enum ModBufSize begin
    BUF_125 = 125
    BUF_250 = 250
    BUF_500 = 500
    BUF_1000 = 1000
    BUF_2000 = 2000
    BUF_4000 = 4000
    BUF_8000 = 8000
    BUF_16000 = 16000
    BUF_32000 = 32000
end

@enum OptMethod begin
    SDP = 0
    EVD = 1
    GS = 2
    GS_PAT = 3
    NAIVE = 4
    LM = 5
end

mutable struct SDPParam
    _regularization::Float32
    _repeat::Int32
    _lambda::Float32
    _normalize::Bool
    function SDPParam(r::Float32, repeat::Int32, l::Float32, n::Bool)
        new(r, repeat, l, n)
    end    
    function SDPParam()
        new(-1f0, Int32(-1), -1f0, true)
    end
end

mutable struct Configuration
    _mod_sampl_freq::ModSamplingFreq
    _mod_buf_size::ModBufSize
    function Configuration(mod_sampl_freq::ModSamplingFreq, mod_buf_size::ModBufSize)
        new(mod_sampl_freq, mod_buf_size)
    end    
    function Configuration()
        new(SMPL_4_KHZ, BUF_4000)
    end
end

mutable struct Gain
    _gain_ptr::Ptr{Cvoid}
    _disposed::Bool
    function Gain(gain_ptr::Ptr{Cvoid})
        gain = new(gain_ptr, false)
        finalizer(gain -> dispose(gain), gain)
        gain
    end
end

mutable struct Modulation
    _mod_ptr::Ptr{Cvoid}
    _disposed::Bool
    function Modulation(mod_ptr::Ptr{Cvoid})
        modulation = new(mod_ptr, false)
        finalizer(modulation -> dispose(modulation), modulation)
        modulation
    end
end

mutable struct Sequence
    _seq_ptr::Ptr{Cvoid}
    _disposed::Bool
    function Sequence(seq_ptr::Ptr{Cvoid})
        seq = new(seq_ptr, false)
        finalizer(seq -> dispose(seq), seq)
        seq
    end
end

mutable struct Link
    _link_ptr::Ptr{Cvoid}
    function Link(link_ptr::Ptr{Cvoid})
        new(link_ptr)
    end
end

mutable struct AUTD
    _handle::Ptr{Cvoid}
    _disposed::Bool
    function AUTD()
        chandle = Ref(Ptr{Cvoid}(0))
        autd_create_controller(chandle)
        autd = new(chandle[], false)
        finalizer(autd -> dispose(autd), autd)
        autd
    end
end

function open_autd(autd::AUTD, link::Link)
    autd_open_controller_with(autd._handle, link._link_ptr)
end

function dispose(self::AUTD)
    if !self._disposed
        autd_close_controller(self._handle)
        autd_free_controller(self._handle)
        self._disposed = true
    end
end

function dispose(self::Gain)
    if !self._disposed
        autd_delete_gain(self._gain_ptr)
        self._disposed = true
    end
end

function dispose(self::Modulation)
    if !self._disposed
        autd_delete_modulation(self._mod_ptr)
        self._disposed = true
    end
end

function dispose(self::Sequence)
    if !self._disposed
        autd_delete_sequence(self._seq_ptr)
        self._disposed = true
    end
end

function enumerate_adapters()
    res = []
    phandle = Ref(Ptr{Cvoid}(0))
    size = autd_get_adapter_pointer(phandle)
    handle::Ptr{Cvoid} = phandle[]

    for i in 0:size - 1
        sb_desc = zeros(UInt8, 128)
        sb_name = zeros(UInt8, 128)
        autd_get_adapter(handle, i, sb_desc,  sb_name)
        push!(res, [String(strip(String(sb_desc), '\0')), String(strip(String(sb_name), '\0'))])
    end

    autd_free_adapter_pointer(handle)
    res
end

function firmware_info_list(autd::AUTD)
    res = []
    phandle = Ref(Ptr{Cvoid}(0))
    size = autd_get_firmware_info_list_pointer(autd._handle, phandle)
    handle::Ptr{Cvoid} = phandle[]

    for i in 0:size - 1
        sb_cpu = zeros(UInt8, 128)
        sb_fpga = zeros(UInt8, 128)
        autd_get_firmware_info(handle, i, sb_cpu,  sb_fpga)
        push!(res, [String(strip(String(sb_cpu), '\0')), String(strip(String(sb_fpga), '\0'))])
    end

    autd_free_firmware_info_list_pointer(handle)
    res
end

function add_device(autd::AUTD, pos::SVector{3,Float32}, rot::SVector{3,Float32}; group_id=0)
    x, y, z = pos
    az1, ay, az2 = rot
    autd_add_device(autd._handle, x, y, z, az1, ay, az2, group_id)
end

function add_device(autd::AUTD, pos::SVector{3,Float32}, qua::SVector{4,Float32}; group_id=0)
    x, y, z = pos
    qw, qx, qy, qz = qua
    autd_add_device_quaternion(autd._handle, x, y, z, qw, qx, qy, qz, group_id)
end

function calibrate(autd::AUTD, config::Configuration=Configuration())
    autd_calibrate(autd._handle, config._mod_sampl_freq, config._mod_buf_size)
end

function clear(autd::AUTD)
    autd_clear(autd._handle)
end

function set_silent_mode(autd::AUTD, silent::Bool)
    autd_set_silent_mode(autd._handle, silent)
end

function stop(autd::AUTD)
    autd_stop(autd._handle)
end

function is_open(autd::AUTD)
    autd_is_open(autd._handle)
end

function is_silent_mode(autd::AUTD)
    autd_is_silent_mode(autd._handle)
end

function wavelength(autd::AUTD)
    autd_wavelength(autd._handle)
end

function set_wavelength(autd::AUTD, wavelength::Float32)
    autd_set_wavelength(autd._handle, wavelength)
end

function num_devices(autd::AUTD)
    autd_num_devices(autd._handle)
end

function num_transducers(autd::AUTD)
    autd_num_transducers(autd._handle)
end

function remaining_in_buffer(autd::AUTD)
    autd_remaining_in_buffer(autd._handle)
end

function adjust_amp(amp::Float32)
    d = asin(amp) / pi
    UInt8(510.0 * d)
end

function focal_point_gain_with_duty(position::SVector{3,Float32}; duty::UInt8)
    x, y, z = position
    chandle = Ref(Ptr{Cvoid}(0))
    autd_focal_point_gain(chandle, x, y, z, duty)
    Gain(chandle[])
end

function focal_point_gain(position::SVector{3,Float32}; amp::Float32=1.0f0)
    focal_point_gain_with_duty(position; duty=adjust_amp(amp))
end

function grouped_gain(group_ids::Array{Int32,1}, gains::Array{Gain,1})
    len = length(group_ids)
    gains = map(g -> g._gain_ptr, gains)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_grouped_gain(chandle, group_ids, gains, len)
    Gain(chandle[])
end

function bessel_beam_gain_with_duty(position::SVector{3,Float32}, direction::SVector{3,Float32}, theta_z::Float32; duty::UInt8)
    x, y, z = position
    nx, ny, nz = direction
    chandle = Ref(Ptr{Cvoid}(0))
    autd_bessel_beam_gain(chandle, x, y, z, nx, ny, nz, theta_z, duty)
    Gain(chandle[])
end

function bessel_beam_gain(position::SVector{3,Float32}, direction::SVector{3,Float32}, theta_z::Float32; amp::Float32=1.0f0)
    bessel_beam_gain_with_duty(position, direction, theta_z; duty=adjust_amp(amp))
end

function plane_wave_gain_with_duty(direction::SVector{3,Float32}; duty::UInt8)
    nx, ny, nz = direction
    chandle = Ref(Ptr{Cvoid}(0))
    autd_plane_wave_gain(chandle, nx, ny, nz, duty)
    Gain(chandle[])
end

function plane_wave_gain(direction::SVector{3,Float32}; amp::Float32=1.0f0)
    plane_wave_gain_with_duty(direction; duty=adjust_amp(amp))
end

function custom_gain(data::Array{UInt16,1})
    len = length(data)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_custom_gain(chandle, data, len)
    Gain(chandle[])
end

function holo_gain(foci::Array{SVector{3,Float32},1}, amps::Array{Float32,1}; method::OptMethod=SDP, params=nothing)
    len = length(foci)

    foci_array = zeros(Float32, len * 3)
    for (i, focus) in enumerate(foci)
        foci_array[3 * (i - 1) + 1] = focus[1]
        foci_array[3 * (i - 1) + 2] = focus[2]
        foci_array[3 * (i - 1) + 3] = focus[3]
    end

    chandle = Ref(Ptr{Cvoid}(0))
    ptr = params == nothing ? Ptr{Cvoid}(0) : Base.unsafe_convert(Ptr{Cvoid}, Ref(params))
    autd_holo_gain(chandle, foci_array, amps, len, Int32(method), ptr)
    Gain(chandle[])
end

function transducer_test_gain(trans_idx::Int32, duty::UInt8, phase::UInt8)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_transducer_test_gain(chandle, trans_idx, duty, phase)
    Gain(chandle[])
end

function null_gain()
    chandle = Ref(Ptr{Cvoid}(0))
    autd_null_gain(chandle)
    Gain(chandle[])
end

function modulation(amp::UInt8=0xFF)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_modulation(chandle, amp)
    Modulation(chandle[])
end

    function custom_modulation(data::Array{UInt8,1})
    len = length(data)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_custom_modulation(chandle, data, len)
    Gain(chandle[])
end

function raw_pcm_modulation(filename::String, sampl_freq::Float32)
    len = length(data)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_raw_pcm_modulation(chandle, filename, sampl_freq)
    Gain(chandle[])
end

function saw_modulation(freq::Int32)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_saw_modulation(chandle, freq)
    Modulation(chandle[])
end

function sine_modulation(freq::Int32, amp::Float32=1.0f0, offset::Float32=0.5f0)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_sine_modulation(chandle, freq, amp, offset)
    Modulation(chandle[])
end

function square_modulation(freq::Int32, low::UInt8=0x00, high::UInt8=0xFF)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_square_modulation(chandle, freq, low, high)
    Modulation(chandle[])
    end

function wav_modulation(filename::String)
    len = length(data)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_wav_modulation(chandle, filename)
    Gain(chandle[])
end

function sequence()
    chandle = Ref(Ptr{Cvoid}(0))
    autd_sequence(chandle)
    Sequence(chandle[])
end

function add_point(seq::Sequence, point::SVector{3,Float32})
    x, y, z = point
    autd_sequence_append_point(seq._seq_ptr, x, y, z)
end

function add_points(seq::Sequence, points::Array{SVector{3,Float32},1})
    len = length(points)

    points_array = zeros(Float32, len * 3)
    for (i, point) in enumerate(points)
        points_array[3 * (i - 1) + 1] = point[1]
        points_array[3 * (i - 1) + 2] = point[2]
        points_array[3 * (i - 1) + 3] = point[3]
    end

    autd_sequence_append_points(seq._seq_ptr, points_array, len)
end

function set_freq(seq::Sequence, freq::Float32)
    autd_sequence_set_freq(seq._seq_ptr, freq)
end

function get_freq(seq::Sequence)
    autd_sequence_freq(seq._seq_ptr)
    end

function get_sampling_freq(seq::Sequence)
    autd_sequence_sampling_freq(seq._seq_ptr)
end

function get_sampling_freq_div(seq::Sequence)
    autd_sequence_sampling_freq_div(seq._seq_ptr)
end

function circum_sequence(center::SVector{3,Float32}, normal::SVector{3,Float32}, radius::Float32, n::UInt64)
    x, y, z = center
    nx, ny, nz = normal
    chandle = Ref(Ptr{Cvoid}(0))
    autd_circum_sequence(chandle, x, y, z, nx, ny, nz, radius, n)
    Sequence(chandle[])
end

function soem_link(ifname::String, device_num::Int32)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_soem_link(chandle, ifname, device_num)
    Link(chandle[])
end

function twincat_link(ipv4_addr::String, ams_net_id::String)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_twincat_link(chandle, ipv4_addr, ams_net_id)
    Link(chandle[])
end

function local_twincat_link()
    chandle = Ref(Ptr{Cvoid}(0))
    autd_local_twincat_link(chandle)
    Link(chandle[])
end

function emulator_link(autd::AUTD, addr::String, port::UInt16)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_emulator_link(chandle, addr, port, autd._handle)
    Link(chandle[])
end

function append_gain(autd::AUTD, gain::Gain)
    autd_append_gain(autd._handle, gain._gain_ptr)
end

function append_gain_sync(autd::AUTD, gain::Gain;wait_for_send::Bool=false)
    autd_append_gain_sync(autd._handle, gain._gain_ptr, wait_for_send)
end

function append_modulation(autd::AUTD, mod::Modulation)
    autd_append_modulation(autd._handle, mod._mod_ptr)
end

function append_modulation_sync(autd::AUTD, mod::Modulation)
    autd_append_modulation_sync(autd._handle, mod._mod_ptr)
end

    function append_stm_gain(autd::AUTD, gain::Gain)
    autd_append_stm_gain(autd._handle, gain._gain_ptr)
end

function start_stm(autd::AUTD, freq::Float32)
    autd_start_stm(autd._handle, freq)
end

function stop_stm(autd::AUTD)
    autd_stop_stm(autd._handle)
end

function finish_stm(autd::AUTD)
    autd_finish_stm(autd._handle)
end

function append_sequence(autd::AUTD, seq::Sequence)
    autd_append_sequence(autd._handle, seq._seq_ptr)
end

function flush(autd::AUTD)
    autd_flush(autd._handle)
end

function device_idx_for_trans_idx(autd::AUTD, global_trans_idx::Int32)
    autd_device_idx_for_trans_idx(autd._handle, global_trans_idx)
end

function trans_position(autd::AUTD, global_trans_idx::Int32)
    p = autd_trans_position_by_global(autd._handle, global_trans_idx)
    SVector(p[1], p[2], p[3])
end

function trans_position(autd::AUTD, device_idx::Int32, local_trans_idx::Int32)
    p = autd_trans_position_by_local(autd._handle, device_idx, local_trans_idx)
    SVector(p[1], p[2], p[3])
end

function device_direction(autd::AUTD, device_idx::Int32)
    p = autd_device_direction(autd._handle, device_idx)
    SVector(p[1], p[2], p[3])
end

end
