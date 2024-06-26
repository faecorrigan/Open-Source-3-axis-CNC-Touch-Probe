(Exported by FreeCAD)
(Post Processor: linuxcnc_post)
(Output Time:2024-06-05 20:02:26.836328)
(begin preamble)
G17 G54 G40 G49 G80 G90
G21
(begin operation: Fixture)
(machine units: mm/min)
G54 
(finish operation: Fixture)
(begin operation: TC: 1mm_cut3mm001)
(machine units: mm/min)
(TC: 1mm_cut3mm001) 
M5
M6 T4 
G43 H4 
M3 S15000 
(finish operation: TC: 1mm_cut3mm001)
(begin operation: Profile003)
(machine units: mm/min)
(Profile003) 
(Compensated Tool Path. Diameter: 1.0) 
G0 Z5.000 
G0 X19.223 Y24.223 
G0 Z3.000 
G1 X19.223 Y24.223 Z-2.300 F571.000 
G2 X19.121 Y23.816 Z-2.300 I-0.173 J-0.173 F571.000 
G3 X18.550 Y23.300 Z-2.300 I-0.024 J-0.547 F571.000 
G1 X18.550 Y6.700 Z-2.300 F571.000 
G3 X18.977 Y6.205 Z-2.300 I0.500 J0.000 F571.000 
G2 X19.165 Y5.734 Z-2.300 I0.058 J-0.250 F571.000 
G2 X18.816 Y5.879 Z-2.300 I-0.115 J0.216 F571.000 
G3 X18.300 Y6.450 Z-2.300 I-0.547 J0.024 F571.000 
G1 X6.700 Y6.450 Z-2.300 F571.000 
G3 X6.205 Y6.023 Z-2.300 I0.000 J-0.500 F571.000 
G2 X5.974 Y5.707 Z-2.300 I-0.274 J-0.043 F571.000 
G2 X5.879 Y6.184 Z-2.300 I-0.024 J0.243 F571.000 
G3 X6.450 Y6.700 Z-2.300 I0.024 J0.547 F571.000 
G1 X6.450 Y23.300 Z-2.300 F571.000 
G3 X6.023 Y23.795 Z-2.300 I-0.500 J0.000 F571.000 
G2 X5.707 Y24.026 Z-2.300 I-0.043 J0.274 F571.000 
G2 X6.184 Y24.121 Z-2.300 I0.243 J0.024 F571.000 
G3 X6.700 Y23.550 Z-2.300 I0.547 J-0.024 F571.000 
G1 X18.300 Y23.550 Z-2.300 F571.000 
G3 X18.795 Y23.977 Z-2.300 I-0.000 J0.500 F571.000 
G2 X19.026 Y24.293 Z-2.300 I0.274 J0.043 F571.000 
G2 X19.223 Y24.223 Z-2.300 I0.024 J-0.243 F571.000 
G0 Z5.000 
G0 Z5.000 
(finish operation: Profile003)
(begin operation: Profile004)
(machine units: mm/min)
(Profile004) 
(Compensated Tool Path. Diameter: 1.0) 
G0 Z5.000 
G0 X25.354 Y30.354 
G0 Z3.000 
G1 X25.354 Y30.354 Z-2.300 F571.000 
G2 X25.500 Y30.000 Z-2.300 I-0.354 J-0.354 F571.000 
G1 X25.500 Y0.000 Z-2.300 F571.000 
G2 X25.000 Y-0.500 Z-2.300 I-0.500 J-0.000 F571.000 
G1 X0.000 Y-0.500 Z-2.300 F571.000 
G2 X-0.500 Y0.000 Z-2.300 I-0.000 J0.500 F571.000 
G1 X-0.500 Y30.000 Z-2.300 F571.000 
G2 X0.000 Y30.500 Z-2.300 I0.500 J0.000 F571.000 
G1 X25.000 Y30.500 Z-2.300 F571.000 
G2 X25.354 Y30.354 Z-2.300 I0.000 J-0.500 F571.000 
G0 Z5.000 
G0 Z5.000 
(finish operation: Profile004)
(begin postamble)
M05
G17 G54 G90 G80 G40
M2
