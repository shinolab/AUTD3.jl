![Windows](https://github.com/shinolab/AUTD3.jl/workflows/Windows/badge.svg)
![Linux](https://github.com/shinolab/AUTD3.jl/workflows/Linux/badge.svg)
![MacOS](https://github.com/shinolab/AUTD3.jl/workflows/MacOS/badge.svg)

# AUTD3.jl #

[autd3 library](https://github.com/shinolab/autd3-library-software) wrapper for Julia 1.3 

version: 0.3.0

## Install ##

```
(v1.3) pkg> add https://github.com/shinolab/AUTD3.jl.git
```

## Requirements

If you are using Windows, install [Npcap](https://nmap.org/npcap/) with WinPcap API-compatible mode (recomennded) or [WinPcap](https://www.winpcap.org/).

If you are using Linux/MacOS, you may need to install and run AUTD3.jl as root. 
```
sudo julia
julia> ]
(v1.3) pkg> add https://github.com/shinolab/AUTD3.jl.git
julia> using AUTD3
``` 

## Exmaple

```julia
using AUTD3

function get_adapter()
    adapters = enumerate_adapters();
    for (i, adapter) in enumerate(adapters)
        println("[" * string(i) * "]: " * adapter[2] * ", " * adapter[1])
    end

    print("Input number: ")
    idx = parse(Int64, readline())

    adapters[idx][2]
end

function main()
    autd = AUTD()

    add_device(autd, (0., 0., 0.), (0., 0., 0.))

    adapter = get_adapter()

    open_autd(autd, SOEM, adapter)

    g = focal_point_gain((90., 80., 150.))
    m = sine_modulation(150)

    append_gain_sync(autd, g)
    append_modulation_sync(autd, m)

    println("press any key to exit...")
    readline();

    dispose(g)
    dispose(m)
    dispose(autd)
end

main();
println("finish")
```

# Author #

Suzuki Shun, 2020
