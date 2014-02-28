#!/usr/bin/env python

import math

# Open file
file = open("lookupTables.c", 'w')

# Supply voltage lookup table
file.write("voltageLookup = {")
for input in range(0, 4096):
    V_S = (((3.3 * 64.0 * (10000.0 + 68000.0) / 10000.0) / 4095) * input)
    if input < 4095:
        file.write(str(int(round(V_S))) + ", ")
    else:
        file.write(str(int(round(V_S))))
file.write("};\n")

# Controller temperature lookup table
file.write("controllerTemperatureLookup = {")
for input in range(0, 4096):
    T_C = (((1.43 - ((3.3 / 4095.0) * input)) * 64.0 / 0.0043) + 250.0)
    if input < 4095:
        file.write(str(int(round(T_C))) + ", ")
    else:
        file.write(str(int(round(T_C))))
file.write("};\n")

# Motor temperature lookup table
file.write("motorTemperatureLookup = {32767, ")
for input in range(1, 4095):
    R_T = ((input * 68000.0) / 4095.0) / (1.0 - (input / 4095.0))
    T_M = (64.0 * (((3585.0 * 298.15) / (3585.0 + (math.log(R_T / 10000.0) * 298.15))) - 273.15))
    output = int(round(T_M))
    if output > 32767:
        file.write(str(32767) + ", ")
    elif output < -32768:
        file.write(str(-32768) + ", ")
    else:
        file.write(str(output) + ", ")
file.write("-7867};")

# Close file
file.close()