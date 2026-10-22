# The ASAP7 SRAM .lib declares time in ns and capacitance in pF, but 8 tables on the output pins
# use ps / fF indices (and ps delay values). Scale those indices by 1/1000, and the delay /
# transition values too; power values are left as they are (not used for timing).
import re, sys
src, dst = sys.argv[1], sys.argv[2]
lines = open(src).read().split("\n")
out, group, fixing, n_tables = [], None, False, 0
scale = lambda m: ", ".join("%g" % (float(x) / 1000.0) for x in m.split(","))
for l in lines:
    g = re.match(r"\s*(cell_rise|cell_fall|rise_transition|fall_transition|rise_power|fall_power)\b", l)
    if g: group, fixing = g.group(1), False
    m = re.match(r"(\s*index_[12] \(\")([^\"]*)(\"\);.*)", l)
    if m and group:
        if m.group(2).startswith("5,") or m.group(2).startswith("5.76,"):
            fixing = True
            if l.strip().startswith("index_1"): n_tables += 1
        if fixing:
            l = m.group(1) + scale(m.group(2)) + m.group(3)
    elif fixing and group in ("cell_rise", "cell_fall", "rise_transition", "fall_transition"):
        v = re.match(r"(\s*\")([^\"]*)(\".*)", l)
        if v: l = v.group(1) + scale(v.group(2)) + v.group(3)
    if fixing and l.strip() == "}": fixing = False; group = None
    out.append(l)
# other defects in the generated file (each reported by Library Compiler):
#  - output_voltage refers to a group that is never defined (pins already name VDD / VSS)
#  - max_capacitance given in fF on the output bus (> 1 pF is impossible here)
#  - a bus type whose bit_width disagrees with bit_from..bit_to
txt = "\n".join(out)
n_ov = len(re.findall(r"\n\s*output_voltage : default_VDD_VSS_output;", txt))
txt = re.sub(r"\n\s*output_voltage : default_VDD_VSS_output;", "", txt)
def fix_cap(m):
    v = float(m.group(2)); return m.group(1) + ("%g" % (v / 1000.0) if v > 1.0 else m.group(2)) + ";"
txt, n_cap = re.subn(r"(max_capacitance : )([0-9.]+);", fix_cap, txt)
def fix_width(m):
    w = int(m.group(2)) - int(m.group(3)) + 1
    return m.group(0).replace("bit_width : %s;" % m.group(1), "bit_width : %d;" % w)
txt = re.sub(r"bit_width : (\d+);\s*\n\s*bit_from\s*: (\d+);\s*\n\s*bit_to\s*: (\d+);", fix_width, txt)
open(dst, "w").write(txt)
print("output_voltage lines removed:", n_ov, " max_capacitance lines checked:", n_cap)
print("tables fixed:", n_tables)
