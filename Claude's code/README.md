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

## Demo 7.1
- Red start/finish line handled: if the red tape reads as white it is no longer taken as the T1 crossbar. Turn crossbars are only accepted within 200 mm of where the plan expects them, and the start line is used as an extra distance resync each lap. Car can be placed on, or up to ~300 mm behind, the start line.
- Tested in the sim with the start line both visible and invisible to the sensors: all 16 cases pass, flying laps ~4.96-4.99 s.

## Demo 7.2 - stability at the same pace
Same lap time as 7.1 (flying laps ~4.97 s in the sim) but far more tolerant of real-car differences.
- Sensor clean-up: only the block of adjacent lit sensors nearest the last line position is used, so a flickering sensor can't jerk the steering.
- Crossbar must be seen on 2 consecutive readings, so sensor flicker next to the line can't fake a crossbar and start a turn early.
- Slow left/right motor-mismatch trim (`KROTI_SLOW`), learned on straights only. Handles one motor 15% weaker (7.1 went off the track).
- Encoder self-calibration from the known crossbar-to-crossbar distances; also corrects speed targets. Handles wheel size -8% to +8% (7.1 failed at +5%).
- Fixed: 7.1 rounded each tick's encoder distance down (~2% under-count).
- Sim results (Tuesday racer.py): 28-case stress set 20 -> 28 pass; sensor flicker over 20 random seeds 7.1 up to 17/20 fail, 7.2 0/20; in racer.py itself 21 cases x (start line visible/invisible) all pass (7.1 fails 5 of the 21).
- Peak grip acceleration ~5.1 m/s^2 (50 ms average), marginally above the 5 m/s^2 target, in exchange for covering 15% motor mismatch on both sides.
