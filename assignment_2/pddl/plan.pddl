Number of literals: 14
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
92% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 17.000
b (16.000 | 30.000)b (13.000 | 60.002)b (12.000 | 60.002)b (9.000 | 90.004)b (8.000 | 90.004)b (5.000 | 120.006)b (4.000 | 120.006)b (1.000 | 130.007);;;; Solution Found
; States evaluated: 26
; Cost: 160.008
; Time 0.00
0.000: (goto rob wp0 wp1)  [30.000]
30.001: (search rob wp1 m11)  [10.000]
30.002: (goto rob wp1 wp2)  [30.000]
60.003: (search rob wp2 m12)  [10.000]
60.004: (goto rob wp2 wp3)  [30.000]
90.005: (search rob wp3 m13)  [10.000]
90.006: (goto rob wp3 wp4)  [30.000]
120.007: (search rob wp4 m15)  [10.000]
130.008: (go-home rob wp4 wp0)  [30.000]
