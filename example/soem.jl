# File: soem.jl
# Project: example
# Created Date: 30/12/2020
# Author: Shun Suzuki
# -----
# Last Modified: 03/06/2021
# Modified By: Shun Suzuki (suzuki@hapis.k.u-tokyo.ac.jp)
# -----
# Copyright (c) 2020 Hapis Lab. All rights reserved.
# 

using Printf

using AUTD3
using StaticArrays

function simple(autd::AUTD) 
    set_silent_mode(autd, true)

    g = focal_point_gain(SVector(90., 80., 150.))
    freq::Int32 = 150
    m = sine_modulation(freq)

    send(autd, g, m)
    
    println("press enter to exit...")
    readline()

    dispose(g)
    dispose(m)
    stop(autd)
end

function bessel(autd::AUTD) 
    set_silent_mode(autd, true)

    g = bessel_beam_gain(SVector(90., 80., 150.), SVector(0., 0., 1.), 13.0 / 180.0 * pi)
    freq::Int32 = 150
    m = sine_modulation(freq)

    send(autd, g, m)
    
    println("press enter to exit...")
    readline()

    dispose(g)
    dispose(m)
    stop(autd)
end

function holo(autd::AUTD) 
    set_silent_mode(autd, true)

    foci = [
        SVector(120., 80., 150.),
        SVector(60., 80., 150.)
    ]
    amps = [1.0, 1.0]
    g = holo_gain_sdp(foci, amps)

    freq::Int32 = 150
    m = sine_modulation(freq)

    send(autd, g, m)
    
    println("press enter to exit...")
    readline()

    dispose(g)
    dispose(m)
    stop(autd)
end


function stm(autd::AUTD) 
    set_silent_mode(autd, false)

    m = static_modulation()
    send_modulation(autd, m)

    center = SVector(90., 80., 150.)

    radius = 30.0
    size = 200
    stm_cnt = stm(autd)
    for i in 1:size
        theta::Float64 = 2pi * i / size
        r = center + radius * SVector(cos(theta), sin(theta), 0)
        f = focal_point_gain(r)
        add_stm_gain(stm_cnt, f)
        dispose(f)
    end

    freq::Float64 = 1.0
    start_stm(stm_cnt, freq)
    
    println("press enter to exit...")
    readline()

    stop_stm(stm_cnt)
    finish_stm(stm_cnt)

    dispose(m)
    stop(autd)
end

function seq(autd::AUTD) 
    set_silent_mode(autd, false)

    m = static_modulation()
    send_modulation(autd, m)

    center = SVector(90., 80., 150.)
    normal = SVector(0., 0., 1.)
    radius = 30.0
    size::UInt64 = 200
    seq = circum_sequence(center, normal, radius, size)
    freq::Float64 = 200.0
    set_freq(seq, freq)

    send_seq(autd, seq)
    
    println("press enter to exit...")
    readline()

    dispose(seq)
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
    synchronize(autd)

    while true
        for (i, (_, name)) in enumerate(samples)
            @printf("[%d]: %s\n", i, name)
        end
        println("[Other]: finish")
        print("Choose number: ")
 
        idx = tryparse(Int64, readline())
        if idx === nothing || idx > length(samples) || idx < 1
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
    idx = tryparse(Int64, readline())
    if idx === nothing || idx > length(adapters) || idx < 1
        println("choose correct number!")
        return ""
    end

    adapters[idx][2]
end

function main()
    autd = AUTD()

    add_device(autd, SVector(0., 0., 0.), SVector(0., 0., 0.))

    adapter = get_adapter()
    link = soem_link(adapter, num_devices(autd), UInt32(1))
    if !open_autd(autd, link)
        println(last_error())
        return
    end

    run(autd)
end

main()
