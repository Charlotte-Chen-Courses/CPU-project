# ASAP7 SRAM bank used by the I-cache

`srambank_128x4x64_6t122.v` is the simulation model of the ASAP7 7-nm SRAM bank
(512 words x 64 bits = 4 KB), copied unchanged from ASAP7_SRAM_0p0
(`/courses/ee6693/tech/asap7/ASAP7_SRAM_0p0/generated/verilog/`). BSD 3-Clause: see
`LICENSE.asap7`. Cite: V. Vashishtha, M. Vangala, P. Sharma, L. T. Clark, "Robust 7-nm SRAM
design on a predictive PDK," ISCAS 2017.

Simulation compiles this model. Synthesis does NOT: Design Compiler links the macro from its
library instead, so it stays one black-box cell with timing arcs.

`fix_units.py` makes that library usable. The generated `.lib` declares ns and pF, but:
- the 8 tables on the `dataout` pins use ps / fF indices (and ps delay values);
- `max_capacitance` on `dataout` is in fF;
- `output_voltage` names an undefined group;
- the address bus type has the wrong `bit_width`.

It writes a corrected copy (`*_ns.lib`), which `lc_shell` compiles to the `.db` that synthesis
links. The macro's area is 0 in the library; use the LEF outline instead
(16.0 x 77.76 um = 1244.2 um^2).
