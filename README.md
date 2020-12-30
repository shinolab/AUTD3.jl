![build](https://github.com/shinolab/AUTD3.jl/workflows/build/badge.svg)

# AUTD3.jl #

[autd3 library](https://github.com/shinolab/autd3-library-software) wrapper for Julia 1.5

version: 0.8.0

## Install ##

```
(v1.5) pkg> add https://github.com/shinolab/AUTD3.jl.git
```

## Requirements

If you are using Windows, install [Npcap](https://nmap.org/npcap/) with WinPcap API-compatible mode (recomennded) or [WinPcap](https://www.winpcap.org/).

If you are using Linux/MacOS, you may need to install and run AUTD3.jl as root. 
```
sudo julia
julia> ]
(v1.5) pkg> add https://github.com/shinolab/AUTD3.jl.git
julia> using AUTD3
``` 

## Exmaple

see [example](./example)

# Author #

Suzuki Shun, 2020
