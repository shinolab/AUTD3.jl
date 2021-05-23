![build](https://github.com/shinolab/AUTD3.jl/workflows/build/badge.svg)

# AUTD3.jl

[autd3 library](https://github.com/shinolab/autd3-library-software) wrapper for Julia 1.6

version: 1.0.0

## :hammer_and_wrench: Install

```
(v1.6) pkg> add https://github.com/shinolab/AUTD3.jl.git
```

## :ballot_box_with_check: Requirements

If you use Windows and `soem_link`, install [Npcap](https://nmap.org/npcap/) with WinPcap API-compatible mode (recomennded) or [WinPcap](https://www.winpcap.org/).

If you are using Linux/MacOS, you may need to install and run AUTD3.jl as root. 
```
sudo julia
julia> ]
(v1.6) pkg> add https://github.com/shinolab/AUTD3.jl.git
julia> using AUTD3
``` 

## :beginner: Example

see [example](./example)

## :copyright: LICENSE

See [LICENSE](./LICENSE) and [NOTICE](./NOTICE).

# Author

Suzuki Shun, 2020-2021
