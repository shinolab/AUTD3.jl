# File: NativeMethods.jl
# Project: src
# Created Date: 11/02/2020
# Author: Shun Suzuki
# -----
# Last Modified: 04/04/2020
# Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
# -----
# Copyright (c) 2020 Hapis Lab. All rights reserved.
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

const _dll_name = joinpath(@__DIR__, "bin", get_lib_prefix() * "autd3capi" * get_lib_ext())

# Controller
autd_create_controller(handle_ptr) = ccall((:AUTDCreateController, _dll_name), Cvoid, (Ref{Ptr{Cvoid}},), handle_ptr)
autd_open(handle_ptr, linktype, location) = ccall((:AUTDOpenController, _dll_name), Int32, (Ptr{Cvoid}, Int32, Cstring), handle_ptr, linktype, location)
autd_close(handle_ptr) = ccall((:AUTDCloseController, _dll_name), Cvoid, (Ptr{Cvoid},), handle_ptr)
autd_free(handle_ptr) = ccall((:AUTDFreeController, _dll_name), Cvoid, (Ptr{Cvoid},), handle_ptr)

autd_add_device(handle_ptr, x, y, z, rz1, ry, rz2, id) = ccall((:AUTDAddDevice, _dll_name), Int32,
        (Ptr{Cvoid}, Float64, Float64, Float64, Float64, Float64, Float64, Int32,),
        handle_ptr, x, y, z, rz1, ry, rz2, id)

autd_calibrate_modulation(handle_ptr) = ccall((:AUTDCalibrateModulation, _dll_name), Bool, (Ptr{Cvoid},), handle_ptr)

autd_set_silent_mode(handle_ptr, silent) = ccall((:AUTDSetSilentMode, _dll_name), Cvoid, (Ptr{Cvoid}, Bool), handle_ptr, silent)

autd_stop(handle_ptr) = ccall((:AUTDStop, _dll_name), Cvoid, (Ptr{Cvoid}), handle_ptr)

autd_get_adapter_pointer(handle_ptr) = ccall((:AUTDGetAdapterPointer, _dll_name), Int32, (Ref{Ptr{Cvoid}},), handle_ptr)
autd_get_adapter(handle, index, decs_p, name_p) = ccall((:AUTDGetAdapter, _dll_name), Cvoid, (Ptr{Cvoid}, Int32, Ref{UInt8}, Ref{UInt8}), handle, index, decs_p, name_p)
autd_free_adapter_pointer(handle) = ccall((:AUTDFreeAdapterPointer, _dll_name), Cvoid, (Ptr{Cvoid},), handle)

autd_get_firm_info_list_pointer(autd_ptr, handle_ptr) = ccall((:AUTDGetFirmwareInfoListPointer, _dll_name), Int32, (Ptr{Cvoid}, Ref{Ptr{Cvoid}},), autd_ptr,  handle_ptr)
autd_get_firm_info(handle, index, cpu_p, fpga_p) = ccall((:AUTDGetFirmwareInfo, _dll_name), Cvoid, (Ptr{Cvoid}, Int32, Ref{UInt8}, Ref{UInt8}), handle, index, cpu_p, fpga_p)
autd_free_firm_info_list_pointer(handle) = ccall((:AUTDFreeFirmwareInfoListPointer, _dll_name), Cvoid, (Ptr{Cvoid},), handle)

# Gain
autd_focal_point_gain(gain_ptr, x, y, z, amp) = ccall((:AUTDFocalPointGain, _dll_name), Cvoid, 
        (Ref{Ptr{Cvoid}}, Float64, Float64, Float64, UInt8), 
        gain_ptr, x, y, z, amp)

autd_delete_gain(gain_ptr) = ccall((:AUTDDeleteGain, _dll_name), Cvoid,  (Ptr{Cvoid},), gain_ptr)

# Modulation
autd_modulation(mod_ptr, amp) = ccall((:AUTDModulation, _dll_name), Cvoid, 
        (Ref{Ptr{Cvoid}}, UInt8), 
        mod_ptr, amp)

autd_sine_modulation(mod_ptr, freq, amp, offset) = ccall((:AUTDSineModulation, _dll_name), Cvoid, 
        (Ref{Ptr{Cvoid}}, Int32, Float64, Float64), 
        mod_ptr, freq, amp, offset)

autd_delete_modulation(gain_ptr) = ccall((:AUTDDeleteModulation, _dll_name), Cvoid,  (Ptr{Cvoid},), gain_ptr)

# Low Level Interface
autd_append_gain(handle_ptr, gain_ptr) = ccall((:AUTDAppendGain, _dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}), handle_ptr, gain_ptr)
autd_append_gain_sync(handle_ptr, gain_ptr, wait_for_send) = ccall((:AUTDAppendGainSync, _dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}, Bool), handle_ptr, gain_ptr, wait_for_send)
autd_append_modulation(handle_ptr, mod_ptr) = ccall((:AUTDAppendModulation, _dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}), handle_ptr, mod_ptr)
autd_append_modulation_sync(handle_ptr, mod_ptr) = ccall((:AUTDAppendModulationSync, _dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}), handle_ptr, mod_ptr)

autd_append_stm_gain(handle_ptr, gain_ptr) = ccall((:AUTDAppendSTMGain, _dll_name), Cvoid, (Ptr{Cvoid}, Ptr{Cvoid}), handle_ptr, gain_ptr)
autd_start_stm(handle_ptr, freq) = ccall((:AUTDStartSTModulation, _dll_name), Cvoid, (Ptr{Cvoid}, Float64), handle_ptr, freq)
autd_stop_stm(handle_ptr) = ccall((:AUTDStopSTModulation, _dll_name), Cvoid, (Ptr{Cvoid},), handle_ptr)
autd_finish_stm(handle_ptr) = ccall((:AUTDFinishSTModulation, _dll_name), Cvoid, (Ptr{Cvoid},), handle_ptr)
