# File: NativeMethods.jl
# Project: src
# Created Date: 10/09/2020
# Author: Shun Suzuki
# -----
# Last Modified: 09/03/2021
# Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
# -----
# Copyright (c) 2021 Hapis Lab. All rights reserved.
# 


function get_lib_ext()
    if Sys.iswindows()
        return ".dll"
    elseif Sys.isapple()
        return ".dylib"
    elseif Sys.islinux()
        return ".so"
    end
end

function get_lib_prefix()
    if Sys.iswindows()
        return ""
    else 
        return "lib"
    end
end

const _main_dll_name = joinpath(@__DIR__, "bin", get_lib_prefix() * "autd3capi" * get_lib_ext())
const _holo_gain_dll_name = joinpath(@__DIR__, "bin", get_lib_prefix() * "autd3capi-gain-holo" * get_lib_ext())
const _soem_link_dll_name = joinpath(@__DIR__, "bin", get_lib_prefix() * "autd3capi-link-soem" * get_lib_ext())
const _twincat_link_dll_name = joinpath(@__DIR__, "bin", get_lib_prefix() * "autd3capi-link-twincat" * get_lib_ext())
const _modulation_from_file_dll_name = joinpath(@__DIR__, "bin", get_lib_prefix() * "autd3capi-modulation-from-file" * get_lib_ext())

autd_create_controller(out) = ccall((:AUTDCreateController,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}},), out)
autd_open_controller_with(handle,p_link) = ccall((:AUTDOpenControllerWith,  _main_dll_name), Int32, (Ptr{Cvoid}, Ptr{Cvoid},), handle, p_link)
autd_add_device(handle,x,y,z,rz1,ry,rz2,group_id) = ccall((:AUTDAddDevice,  _main_dll_name), Int32, (Ptr{Cvoid}, Float32, Float32, Float32, Float32, Float32, Float32, Int32,), handle, x, y, z, rz1, ry, rz2, group_id)
autd_add_device_quaternion(handle,x,y,z,qua_w,qua_x,qua_y,qua_z,group_id) = ccall((:AUTDAddDeviceQuaternion,  _main_dll_name), Int32, (Ptr{Cvoid}, Float32, Float32, Float32, Float32, Float32, Float32, Float32, Int32,), handle, x, y, z, qua_w, qua_x, qua_y, qua_z, group_id)
autd_calibrate(handle,smpl_freq,buf_size) = ccall((:AUTDCalibrate,  _main_dll_name), Bool, (Ptr{Cvoid}, Int32, Int32,), handle, smpl_freq, buf_size)
autd_close_controller(handle) = ccall((:AUTDCloseController,  _main_dll_name), Cvoid, (Ptr{Cvoid},), handle)
autd_clear(handle) = ccall((:AUTDClear,  _main_dll_name), Cvoid, (Ptr{Cvoid},), handle)
autd_free_controller(handle) = ccall((:AUTDFreeController,  _main_dll_name), Cvoid, (Ptr{Cvoid},), handle)
autd_set_silent_mode(handle,mode) = ccall((:AUTDSetSilentMode,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Bool,), handle, mode)
autd_stop(handle) = ccall((:AUTDStop,  _main_dll_name), Cvoid, (Ptr{Cvoid},), handle)
autd_get_adapter_pointer(out) = ccall((:AUTDGetAdapterPointer,  _soem_link_dll_name), Int32, (Ref{Ptr{Cvoid}},), out)
autd_get_adapter(p_adapter,index,desc,name) = ccall((:AUTDGetAdapter,  _soem_link_dll_name), Cvoid, (Ptr{Cvoid}, Int32, Ref{UInt8}, Ref{UInt8},), p_adapter, index, desc, name)
autd_free_adapter_pointer(p_adapter) = ccall((:AUTDFreeAdapterPointer,  _soem_link_dll_name), Cvoid, (Ptr{Cvoid},), p_adapter)
autd_get_firmware_info_list_pointer(handle,out) = ccall((:AUTDGetFirmwareInfoListPointer,  _main_dll_name), Int32, (Ptr{Cvoid}, Ref{Ptr{Cvoid}},), handle, out)
autd_get_firmware_info(p_firm_info_list,index,cpu_ver,fpga_ver) = ccall((:AUTDGetFirmwareInfo,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Int32, Ref{UInt8}, Ref{UInt8},), p_firm_info_list, index, cpu_ver, fpga_ver)
autd_free_firmware_info_list_pointer(p_firm_info_list) = ccall((:AUTDFreeFirmwareInfoListPointer,  _main_dll_name), Cvoid, (Ptr{Cvoid},), p_firm_info_list)
autd_is_open(handle) = ccall((:AUTDIsOpen,  _main_dll_name), Bool, (Ptr{Cvoid},), handle)
autd_is_silent_mode(handle) = ccall((:AUTDIsSilentMode,  _main_dll_name), Bool, (Ptr{Cvoid},), handle)
autd_wavelength(handle) = ccall((:AUTDWavelength,  _main_dll_name), Float32, (Ptr{Cvoid},), handle)
autd_set_wavelength(handle,wavelength) = ccall((:AUTDSetWavelength,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Float32,), handle, wavelength)
autd_num_devices(handle) = ccall((:AUTDNumDevices,  _main_dll_name), Int32, (Ptr{Cvoid},), handle)
autd_num_transducers(handle) = ccall((:AUTDNumTransducers,  _main_dll_name), Int32, (Ptr{Cvoid},), handle)
autd_remaining_in_buffer(handle) = ccall((:AUTDRemainingInBuffer,  _main_dll_name), UInt64, (Ptr{Cvoid},), handle)
autd_focal_point_gain(gain,x,y,z,duty) = ccall((:AUTDFocalPointGain,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Float32, Float32, Float32, UInt8,), gain, x, y, z, duty)
autd_grouped_gain(gain,group_ids,in_gains,size) = ccall((:AUTDGroupedGain,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Ptr{Int32}, Ptr{Ptr{Cvoid}}, Int32,), gain, group_ids, in_gains, size)
autd_bessel_beam_gain(gain,x,y,z,n_x,n_y,n_z,theta_z,duty) = ccall((:AUTDBesselBeamGain,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Float32, Float32, Float32, Float32, Float32, Float32, Float32, UInt8,), gain, x, y, z, n_x, n_y, n_z, theta_z, duty)
autd_plane_wave_gain(gain,n_x,n_y,n_z,duty) = ccall((:AUTDPlaneWaveGain,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Float32, Float32, Float32, UInt8,), gain, n_x, n_y, n_z, duty)
autd_custom_gain(gain,data,data_length) = ccall((:AUTDCustomGain,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Ptr{UInt16}, Int32,), gain, data, data_length)
autd_holo_gain(gain,points,amps,size,method,params) = ccall((:AUTDHoloGain,  _holo_gain_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Ptr{Float32}, Ptr{Float32}, Int32, Int32, Ptr{Cvoid},), gain, points, amps, size, method, params)
autd_transducer_test_gain(gain,idx,duty,phase) = ccall((:AUTDTransducerTestGain,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Int32, UInt8, UInt8,), gain, idx, duty, phase)
autd_null_gain(gain) = ccall((:AUTDNullGain,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}},), gain)
autd_delete_gain(gain) = ccall((:AUTDDeleteGain,  _main_dll_name), Cvoid, (Ptr{Cvoid},), gain)
autd_modulation(mod,amp) = ccall((:AUTDModulation,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, UInt8,), mod, amp)
autd_custom_modulation(mod,buf,size) = ccall((:AUTDCustomModulation,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Ptr{UInt8}, UInt32,), mod, buf, size)
autd_raw_pcm_modulation(mod,filename,sampling_freq) = ccall((:AUTDRawPCMModulation,  _modulation_from_file_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Cstring, Float32,), mod, filename, sampling_freq)
autd_saw_modulation(mod,freq) = ccall((:AUTDSawModulation,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Int32,), mod, freq)
autd_sine_modulation(mod,freq,amp,offset) = ccall((:AUTDSineModulation,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Int32, Float32, Float32,), mod, freq, amp, offset)
autd_square_modulation(mod,freq,low,high) = ccall((:AUTDSquareModulation,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Int32, UInt8, UInt8,), mod, freq, low, high)
autd_wav_modulation(mod,filename) = ccall((:AUTDWavModulation,  _modulation_from_file_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Cstring,), mod, filename)
autd_delete_modulation(mod) = ccall((:AUTDDeleteModulation,  _main_dll_name), Cvoid, (Ptr{Cvoid},), mod)
autd_sequence(out) = ccall((:AUTDSequence,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}},), out)
autd_sequence_append_point(seq,x,y,z) = ccall((:AUTDSequenceAppendPoint,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Float32, Float32, Float32,), seq, x, y, z)
autd_sequence_append_points(seq,points,size) = ccall((:AUTDSequenceAppendPoints,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Float32}, UInt64,), seq, points, size)
autd_sequence_set_freq(seq,freq) = ccall((:AUTDSequenceSetFreq,  _main_dll_name), Float32, (Ptr{Cvoid}, Float32,), seq, freq)
autd_sequence_freq(seq) = ccall((:AUTDSequenceFreq,  _main_dll_name), Float32, (Ptr{Cvoid},), seq)
autd_sequence_sampling_freq(seq) = ccall((:AUTDSequenceSamplingFreq,  _main_dll_name), Float32, (Ptr{Cvoid},), seq)
autd_sequence_sampling_freq_div(seq) = ccall((:AUTDSequenceSamplingFreqDiv,  _main_dll_name), UInt16, (Ptr{Cvoid},), seq)
autd_circum_sequence(out,x,y,z,nx,ny,nz,radius,n) = ccall((:AUTDCircumSequence,  _main_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Float32, Float32, Float32, Float32, Float32, Float32, Float32, UInt64,), out, x, y, z, nx, ny, nz, radius, n)
autd_delete_sequence(seq) = ccall((:AUTDDeleteSequence,  _main_dll_name), Cvoid, (Ptr{Cvoid},), seq)
autd_soem_link(out,ifname,device_num) = ccall((:AUTDSOEMLink,  _soem_link_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Cstring, Int32,), out, ifname, device_num)
autd_twincat_link(out,ipv4_addr,ams_net_id) = ccall((:AUTDTwinCATLink,  _twincat_link_dll_name), Cvoid, (Ref{Ptr{Cvoid}}, Cstring, Cstring,), out, ipv4_addr, ams_net_id)
autd_local_twincat_link(out) = ccall((:AUTDLocalTwinCATLink,  _twincat_link_dll_name), Cvoid, (Ref{Ptr{Cvoid}},), out)
autd_append_gain(handle,gain) = ccall((:AUTDAppendGain,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid},), handle, gain)
autd_append_gain_sync(handle,gain,wait_for_send) = ccall((:AUTDAppendGainSync,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}, Bool,), handle, gain, wait_for_send)
autd_append_modulation(handle,mod) = ccall((:AUTDAppendModulation,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid},), handle, mod)
autd_append_modulation_sync(handle,mod) = ccall((:AUTDAppendModulationSync,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid},), handle, mod)
autd_append_stm_gain(handle,gain) = ccall((:AUTDAppendSTMGain,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid},), handle, gain)
autd_start_stm(handle,freq) = ccall((:AUTDStartSTModulation,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Float32,), handle, freq)
autd_stop_stm(handle) = ccall((:AUTDStopSTModulation,  _main_dll_name), Cvoid, (Ptr{Cvoid},), handle)
autd_finish_stm(handle) = ccall((:AUTDFinishSTModulation,  _main_dll_name), Cvoid, (Ptr{Cvoid},), handle)
autd_append_sequence(handle,seq) = ccall((:AUTDAppendSequence,  _main_dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid},), handle, seq)
autd_flush(handle) = ccall((:AUTDFlush,  _main_dll_name), Cvoid, (Ptr{Cvoid},), handle)
autd_device_idx_for_trans_idx(handle,global_trans_idx) = ccall((:AUTDDeviceIdxForTransIdx,  _main_dll_name), Int32, (Ptr{Cvoid}, Int32,), handle, global_trans_idx)
autd_trans_position_by_global(handle,global_trans_idx) = ccall((:AUTDTransPositionByGlobal,  _main_dll_name), Array{Float32,1}, (Ptr{Cvoid}, Int32,), handle, global_trans_idx)
autd_trans_position_by_local(handle,device_idx,local_trans_idx) = ccall((:AUTDTransPositionByLocal,  _main_dll_name), Array{Float32,1}, (Ptr{Cvoid}, Int32, Int32,), handle, device_idx, local_trans_idx)
autd_device_direction(handle,device_idx) = ccall((:AUTDDeviceDirection,  _main_dll_name), Array{Float32,1}, (Ptr{Cvoid}, Int32,), handle, device_idx)

