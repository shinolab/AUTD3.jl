# File: AUTD3.jl
# Project: src
# Created Date: 11/02/2020
# Author: Shun Suzuki
# -----
# Last Modified: 24/05/2021
# Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
# -----
# Copyright (c) 2020 Hapis Lab. All rights reserved.
# 

module AUTD3

using StaticArrays

export Gain, Modulation, Sequence, Link, AUTD
export open_autd, dispose, add_device, synchronize, clear, set_silent_mode, stop, is_open, is_silent_mode, wavelength, set_wavelength
export num_devices, num_transducers
export device_direction_x, device_direction_y, device_direction_z, trans_position, device_idx_for_trans_idx
export enumerate_adapters, firmware_info_list
export last_error
export focal_point_gain, focal_point_gain_with_duty, grouped_gain, bessel_beam_gain, bessel_beam_gain_with_duty, plane_wave_gain,plane_wave_gain_with_duty, custom_gain, holo_gain, transducer_test_gain, null_gain
export holo_gain_sdp, holo_gain_evd, holo_gain_gs, holo_gain_gspat, holo_gain_lm, holo_gain_naive
export static_modulation, sine_modulation, custom_modulation, saw_modulation, square_modulation
export sequence, add_point, add_points, set_freq, get_freq, get_sampling_freq, get_sampling_freq_div, circum_sequence
export soem_link, twincat_link
export send, send_gain, send_modulation, send_seq
export add_stm_gain, start_stm, stop_stm, finish_stm

include("NativeMethods.jl")

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
    autd_open_controller(autd._handle, link._link_ptr)
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

function add_device(autd::AUTD, pos::SVector{3,Float64}, rot::SVector{3,Float64}; group_id=0)
    x, y, z = pos
    az1, ay, az2 = rot
    autd_add_device(autd._handle, x, y, z, az1, ay, az2, group_id)
end

function add_device(autd::AUTD, pos::SVector{3,Float64}, qua::SVector{4,Float64}; group_id=0)
    x, y, z = pos
    qw, qx, qy, qz = qua
    autd_add_device_quaternion(autd._handle, x, y, z, qw, qx, qy, qz, group_id)
end

function synchronize(autd::AUTD; mod_sampling_freq_div::UInt16=UInt16(10), mod_buf_size::UInt16=UInt16(4000))
    autd_synchronize(autd._handle, mod_sampling_freq_div, mod_buf_size)
end

function clear(autd::AUTD)
    autd_clear(autd._handle)
end

function update_ctrl_flags(autd::AUTD)
    autd_update_ctrl_flags(autd._handle)
end

function stop(autd::AUTD)
    autd_stop(autd._handle)
end

function is_open(autd::AUTD)
    autd_is_open(autd._handle)
end

function last_error() 
    size = autd_get_last_error(Ptr{Cvoid}(0))
    err = zeros(UInt8, size)
    autd_get_last_error(err)
    String(strip(String(err), '\0'))
end

function set_silent_mode(autd::AUTD, silent::Bool)
    autd_set_silent_mode(autd._handle, silent)
end

function is_silent_mode(autd::AUTD)
    autd_is_silent_mode(autd._handle)
end

function wavelength(autd::AUTD)
    autd_wavelength(autd._handle)
end

function set_wavelength(autd::AUTD, wavelength::Float64)
    autd_set_wavelength(autd._handle, wavelength)
end

function num_devices(autd::AUTD)
    autd_num_devices(autd._handle)
end

function num_transducers(autd::AUTD)
    autd_num_transducers(autd._handle)
end

function adjust_amp(amp::Float64)
    d = asin(amp) / pi
    UInt8(510.0 * d)
end

function focal_point_gain_with_duty(position::SVector{3,Float64}; duty::UInt8)
    x, y, z = position
    chandle = Ref(Ptr{Cvoid}(0))
    autd_focal_point_gain(chandle, x, y, z, duty)
    Gain(chandle[])
end

function focal_point_gain(position::SVector{3,Float64}; amp::Float64=1.0)
    focal_point_gain_with_duty(position; duty=adjust_amp(amp))
end

function grouped_gain(group_ids::Array{Tuple{Int32,Gain},1})
    chandle = Ref(Ptr{Cvoid}(0))
    autd_grouped_gain(chandle)
    for (id, gain) in group_ids
        autd_grouped_gain_add(chandle[], id, gain)
    end
    Gain(chandle[])
end

function bessel_beam_gain_with_duty(position::SVector{3,Float64}, direction::SVector{3,Float64}, theta_z::Float64; duty::UInt8)
    x, y, z = position
    nx, ny, nz = direction
    chandle = Ref(Ptr{Cvoid}(0))
    autd_bessel_beam_gain(chandle, x, y, z, nx, ny, nz, theta_z, duty)
    Gain(chandle[])
end

function bessel_beam_gain(position::SVector{3,Float64}, direction::SVector{3,Float64}, theta_z::Float64; amp::Float64=1.0)
    bessel_beam_gain_with_duty(position, direction, theta_z; duty=adjust_amp(amp))
end

function plane_wave_gain_with_duty(direction::SVector{3,Float64}; duty::UInt8)
    nx, ny, nz = direction
    chandle = Ref(Ptr{Cvoid}(0))
    autd_plane_wave_gain(chandle, nx, ny, nz, duty)
    Gain(chandle[])
end

function plane_wave_gain(direction::SVector{3,Float64}; amp::Float64=1.0)
    plane_wave_gain_with_duty(direction; duty=adjust_amp(amp))
end

function custom_gain(data::Array{UInt16,1})
    len = length(data)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_custom_gain(chandle, data, len)
    Gain(chandle[])
end

function _pack_foci(foci::Array{SVector{3,Float64},1})
    len = length(foci)
    foci_array = zeros(Float64, len * 3)
    for (i, focus) in enumerate(foci)
        foci_array[3 * (i - 1) + 1] = focus[1]
        foci_array[3 * (i - 1) + 2] = focus[2]
        foci_array[3 * (i - 1) + 3] = focus[3]
    end
    foci_array
end

function holo_gain_sdp(foci::Array{SVector{3,Float64},1}, amps::Array{Float64,1}; alpha::Float64=1e-3, lambda::Float64=0.9, repeat::UInt64=UInt64(100), normalize::Bool=true)
    len = length(foci)

    foci_array = _pack_foci(foci) 
    backend = Ref(Ptr{Cvoid}(0))
    autd_eigen_backend(backend)

    chandle = Ref(Ptr{Cvoid}(0))
    autd_holo_gain_sdp(chandle, backend[], foci_array, amps, len, alpha, lambda, repeat, normalize)
    autd_delete_backend(backend[])
    Gain(chandle[])
end

function holo_gain_evd(foci::Array{SVector{3,Float64},1}, amps::Array{Float64,1}; gamma::Float64=1.0, normalize::Bool=true)
    len = length(foci)

    foci_array = _pack_foci(foci) 
    backend = Ref(Ptr{Cvoid}(0))
    autd_eigen_backend(backend)

    chandle = Ref(Ptr{Cvoid}(0))
    autd_holo_gain_evd(chandle, backend[], foci_array, amps, len, gamma, normalize)
    autd_delete_backend(backend[])
    Gain(chandle[])
end

function holo_gain_naive(foci::Array{SVector{3,Float64},1}, amps::Array{Float64,1})
    len = length(foci)

    foci_array = _pack_foci(foci) 
    backend = Ref(Ptr{Cvoid}(0))
    autd_eigen_backend(backend)

    chandle = Ref(Ptr{Cvoid}(0))
    autd_holo_gain_naive(chandle, backend[], foci_array, amps, len)
    autd_delete_backend(backend[])
    Gain(chandle[])
end

function holo_gain_gs(foci::Array{SVector{3,Float64},1}, amps::Array{Float64,1}; repeat::UInt64=UInt64(100))
    len = length(foci)

    foci_array = _pack_foci(foci) 
    backend = Ref(Ptr{Cvoid}(0))
    autd_eigen_backend(backend)

    chandle = Ref(Ptr{Cvoid}(0))
    autd_holo_gain_gs(chandle, backend[], foci_array, amps, len, repeat)
    autd_delete_backend(backend[])
    Gain(chandle[])
end

function holo_gain_gspat(foci::Array{SVector{3,Float64},1}, amps::Array{Float64,1}; repeat::UInt64=UInt64(100))
    len = length(foci)

    foci_array = _pack_foci(foci) 
    backend = Ref(Ptr{Cvoid}(0))
    autd_eigen_backend(backend)

    chandle = Ref(Ptr{Cvoid}(0))
    autd_holo_gain_gspat(chandle, backend[], foci_array, amps, len, repeat)
    autd_delete_backend(backend[])
    Gain(chandle[])
end

function holo_gain_lm(foci::Array{SVector{3,Float64},1}, amps::Array{Float64,1}; eps_1::Float64=1e-8,eps_2::Float64=1e-8,tau::Float64=1e-3,k_max::UInt64=UInt64(5),initial::Array{Float64,1}=[])
    len = length(foci)

    foci_array = _pack_foci(foci) 
    backend = Ref(Ptr{Cvoid}(0))
    autd_eigen_backend(backend)

    chandle = Ref(Ptr{Cvoid}(0))
    initial_len = length(initial)
    init = initial_len == 0 ? Ptr{Cvoid}(0) : initial
    autd_holo_gain_lm(chandle, backend[], foci_array, amps, len, eps_1, eps_2, tau, k_max, init, initial_len)
    autd_delete_backend(backend[])
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

function static_modulation(amp::UInt8=0xFF)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_static_modulation(chandle, amp)
    Modulation(chandle[])
end

function custom_modulation(data::Array{UInt8,1})
    len = length(data)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_custom_modulation(chandle, data, len)
    Modulation(chandle[])
end

function saw_modulation(freq::Int32)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_saw_modulation(chandle, freq)
    Modulation(chandle[])
end

function sine_modulation(freq::Int32, amp::Float64=1.0, offset::Float64=0.5)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_sine_modulation(chandle, freq, amp, offset)
    Modulation(chandle[])
end

function square_modulation(freq::Int32, low::UInt8=0x00, high::UInt8=0xFF)
    chandle = Ref(Ptr{Cvoid}(0))
    autd_square_modulation(chandle, freq, low, high)
    Modulation(chandle[])
end

function sequence()
    chandle = Ref(Ptr{Cvoid}(0))
    autd_sequence(chandle)
    Sequence(chandle[])
end

function add_point(seq::Sequence, point::SVector{3,Float64})
    x, y, z = point
    autd_sequence_add_point(seq._seq_ptr, x, y, z)
end

function add_points(seq::Sequence, points::Array{SVector{3,Float64},1})
    len = length(points)

    points_array = zeros(Float64, len * 3)
    for (i, point) in enumerate(points)
        points_array[3 * (i - 1) + 1] = point[1]
        points_array[3 * (i - 1) + 2] = point[2]
        points_array[3 * (i - 1) + 3] = point[3]
    end

    autd_sequence_add_points(seq._seq_ptr, points_array, len)
end

function set_freq(seq::Sequence, freq::Float64)
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

function circum_sequence(center::SVector{3,Float64}, normal::SVector{3,Float64}, radius::Float64, n::UInt64)
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

function twincat_link()
    chandle = Ref(Ptr{Cvoid}(0))
    autd_twincat_link(chandle)
Link(chandle[])
end

function send(autd::AUTD, gain::Gain, mod::Modulation)
    autd_send_gain_modulation(autd._handle, gain._gain_ptr, mod._mod_ptr)
end

function send_gain(autd::AUTD, gain::Gain)
    autd_send_gain(autd._handle, gain._gain_ptr)
end

function send_modulation(autd::AUTD, mod::Modulation)
    autd_send_modulation(autd._handle,  mod._mod_ptr)
end

function send_seq(autd::AUTD, seq::Sequence)
    autd_send_sequence(autd._handle, seq._seq_ptr)
end

function add_stm_gain(autd::AUTD, gain::Gain)
    autd_add_stm_gain(autd._handle, gain._gain_ptr)
end
 
function start_stm(autd::AUTD, freq::Float64)
    autd_start_stm(autd._handle, freq)
end

function stop_stm(autd::AUTD)
    autd_stop_stm(autd._handle)
end

function finish_stm(autd::AUTD)
    autd_finish_stm(autd._handle)
end

function device_idx_for_trans_idx(autd::AUTD, global_trans_idx::Int32)
    autd_device_idx_for_trans_idx(autd._handle, global_trans_idx)
end

function trans_position(autd::AUTD, global_trans_idx::Int32)
    x = Ref{Float64}(0)
    y = Ref{Float64}(0)
    z = Ref{Float64}(0)
    autd_trans_position_by_global(autd._handle, global_trans_idx, x, y, z)
    SVector(x[], y[], z[])
end

function trans_position(autd::AUTD, device_idx::Int32, local_trans_idx::Int32)
    x = Ref{Float64}(0)
    y = Ref{Float64}(0)
    z = Ref{Float64}(0)
    autd_trans_position_by_local(autd._handle, device_idx, local_trans_idx, x, y, z)
    SVector(x[], y[], z[])
end

function device_direction_x(autd::AUTD, device_idx::Int32)
    x = Ref{Float64}(0)
    y = Ref{Float64}(0)
    z = Ref{Float64}(0)
    autd_device_x_direction(autd._handle, device_idx, x, y, z)
    SVector(x[], y[], z[])
end

function device_direction_y(autd::AUTD, device_idx::Int32)
    x = Ref{Float64}(0)
    y = Ref{Float64}(0)
    z = Ref{Float64}(0)
    autd_device_y_direction(autd._handle, device_idx, x, y, z)
    SVector(x[], y[], z[])
end

function device_direction_z(autd::AUTD, device_idx::Int32)
    x = Ref{Float64}(0)
    y = Ref{Float64}(0)
    z = Ref{Float64}(0)
    autd_device_z_direction(autd._handle, device_idx, x, y, z)
    SVector(x[], y[], z[])
end

end
