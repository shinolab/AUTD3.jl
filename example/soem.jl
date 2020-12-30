# File: soem.jl
# Project: example
# Created Date: 30/12/2020
# Author: Shun Suzuki
# -----
# Last Modified: 30/12/2020
# Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
# -----
# Copyright (c) 2020 Hapis Lab. All rights reserved.
# 

using Printf

using AUTD3
using StaticArrays

function simple(autd::AUTD) 
    set_silent_mode(autd, true)

    g = focal_point_gain(SVector(90.f0, 80.f0, 150.f0))
    freq::Int32 = 150
    m = sine_modulation(freq)

    append_gain_sync(autd, g)
    append_modulation_sync(autd, m)
    
    println("press enter to exit...")
    readline()

    dispose(g)
    dispose(m)
    stop(autd)
end

function bessel(autd::AUTD) 
    set_silent_mode(autd, true)

    g = bessel_beam_gain(SVector(90.f0, 80.f0, 150.f0), SVector(0.f0, 0.f0, 1.f0), 13.0f0 / 180.0f0 * pi)
    freq::Int32 = 150
    m = sine_modulation(freq)

    append_gain_sync(autd, g)
    append_modulation_sync(autd, m)
    
    println("press enter to exit...")
    readline()

    dispose(g)
    dispose(m)
    stop(autd)
end

function holo(autd::AUTD) 
    set_silent_mode(autd, true)

    foci = [
        SVector(120.f0, 80.f0, 150.f0),
        SVector(60.f0, 80.f0, 150.f0)
    ]
    amps = [1.0f0, 1.0f0]
    g = holo_gain(foci, amps)
    freq::Int32 = 150
    m = sine_modulation(freq)

    append_gain_sync(autd, g)
    append_modulation_sync(autd, m)
    
    println("press enter to exit...")
    readline()

    dispose(g)
    dispose(m)
    stop(autd)
end


function stm(autd::AUTD) 
    set_silent_mode(autd, false)

    m = modulation()
    append_modulation_sync(autd, m)

    center = SVector(90.f0, 80.f0, 150.f0)

    radius = 30.0f0
    size = 200
    for i in 1:size
        theta::Float32 = 2pi * i / size
        r = center + radius * SVector(cos(theta), sin(theta), 0f0)
        f = focal_point_gain(r)
        append_stm_gain(autd, f)
    end

    start_stm(autd, 1f0)
    
    println("press enter to exit...")
    readline()

    dispose(m)
    stop(autd)
end

function seq(autd::AUTD) 
    set_silent_mode(autd, false)

    m = modulation()
    append_modulation_sync(autd, m)

    center = SVector(90.f0, 80.f0, 150.f0)
    normal = SVector(0.f0, 0.f0, 1.f0)
    radius = 30.0f0
    size::UInt64 = 200
    seq = circum_sequence(center, normal, radius, size)
    set_freq(seq, 200f0)

    append_sequence(autd, seq)
    
    println("press enter to exit...")
    readline()

    dispose(m)
    stop(autd)
end

function run(autd::AUTD)
    samples = [
        (simple, "Single Focal Point Sample"),
        (bessel, "Bessel beam Sample"),
        (holo, "Multiple Focal Points Sample"),
        (stm, "Spatio-Temporal Modulation Sample"),
        (seq, "PointSequence (Hardware STM) Sample")
    ]
    
    firm_info_list = firmware_info_list(autd)
    for (i, firm_info) in enumerate(firm_info_list)
        @printf("AUTD[%d]: CPU: %s, FPGA: %s\n", i, firm_info[1], firm_info[2])
    end

    clear(autd)
    calibrate(autd)

    while true
        for (i, (_, name)) in enumerate(samples)
            @printf("[%d]: %s\n", i, name)
        end
        println("[Other]: finish")
        print("Choose number: ")

        idx = tryparse(Int64, readline())
        if idx == nothing || idx > length(samples)
            break
        end

        (fn, _) = samples[idx]
        fn(autd)

    end

    clear(autd)
    dispose(autd)
end

function get_adapter()
    adapters = enumerate_adapters();
    for (i, adapter) in enumerate(adapters)
        @printf("[%d]: %s, %s\n", i, adapter[2], adapter[1])
    end

    print("Input number: ")
    idx = parse(Int64, readline())

    adapters[idx][2]
end

function main()
    autd = AUTD()

    add_device(autd, SVector(0.f0, 0.f0, 0.f0), SVector(0.f0, 0.f0, 0.f0))

    adapter = get_adapter()
    link = soem_link(adapter, num_devices(autd))
    open_autd(autd, link)

    run(autd)
end

main()
