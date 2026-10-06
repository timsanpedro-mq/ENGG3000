# Tuesday Code
Tuesday Code is an edit of Monday Code to more closely align with Maribelle's driving maths
It has also made significant simulation parameters in racer.py to include speed estimation during simulation, and modified car characteristics which has affected performance drastically.
The edits from controller.c are outlined here: 
STRAIGHT_SPEED: 75 > 50
R100_SPEED: 13 > 9
R150_SPEED: 17 > 9
R200_SPEED: 19 > 9
TR_EFFORT_MAX: 110 > 48

## Demo 7.0 (controller.c)
Builds on 6.1 (crossbar turn plan + encoder distance + translational/rotational PI). 6.1 is kept as `controller_demo6.1_backup.c`.
- TR_EFFORT_MAX = 48 was capping the car at ~0.28 m/s everywhere, so the planned speeds were never reached. Effort cap removed; wheel loops now have speed feedforward (KFF, FRIC_PWM) plus the same PI.
- Whole-lap track plan: each straight and radius is a section with its own length, direction and speed. Crossbars still trigger each turn and resync the plan.
- Braking on straights from encoder distance to the next slower section (v^2 = v_turn^2 + 2*DECEL*d).
- Turn feedforward from the planned radius (wheel split v*HALF_TRACK/R), switched slightly early (FF_LEAD_MM) and eased in (FF_SLEW).
- Line steering is PD (LKP, LKD over D_WINDOW ticks); encoder speed averaged over ENC_AVG ticks.
- Sim result: flying laps ~4.96 s, first lap ~5.15 s (6.1: 18.2 s), no line loss from 8 start positions/headings or with sensor distance 20/30 mm, mass 170/250 g, motor voltage +/-10%, encoder/wheel size +/-3%.
- Grip acceleration (forward/braking + cornering, 50 ms average) capped at 5 m/s^2 to match the 6.1 turn-speed maths. The sim's total-acceleration graph reads higher (~7 m/s^2) because it also counts yaw angular acceleration.
