# RISC-V
Verilog implementation of a RISC-V core targeted for FPGAs.

FPGA project is for this board https://github.com/ataradov/max10_bb

## Resource Utilization

Below is a table of some configurations and corresponding resource utilization.

```
+------------------------------+
| Config    |  LE  | REG | MUL |
+-----------+------+-----+-----+
| RVC + BS  | 2256 | 441 |  -  |
| RVC + MUL | 1807 | 410 |  8  |
| RVC       | 1972 | 481 |  -  |
| BS        | 1970 | 442 |  -  |
| MUL       | 1511 | 410 |  8  |
| -         | 1662 | 481 |  -  |
+-----------+------+-----+-----+
```
RVC - compressed ISA support, BS - Barrel Shifter, MUL - single cycle hardware multiplier.

It synthesizes at at least 60 MHz in the slowest grade MAX 10.

