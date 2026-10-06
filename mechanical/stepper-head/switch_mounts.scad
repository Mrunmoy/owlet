// owlet stepper head, by Mrunmoy
// Copyright (c) 2026 Mrunmoy. SPDX-License-Identifier: MIT
// https://github.com/Mrunmoy/owlet
//
// Mounts for the limit switches on the stepper head: the common 3D-printer
// endstop module (a lever microswitch on a 20.6 x 12.9 mm board with two
// corner holes and an S/G/V plug).
//
//   openscad -D 'part="pan_bracket"' -o stl/pan_bracket.stl switch_mounts.scad
//     parts: pan_bracket | pan_flag | assembly
//
// Pan: two brackets glued to the base deck at radius `r_sw`, 95 degrees
// either side of straight ahead; one flag glued under the yoke at the same
// radius, hanging down to press the switch levers. Tilt mounts come once
// the tilt shaft's overhang is known.
//
// All dimensions in mm. Measure the board's hole spacing and set hole_dx /
// hole_dy; until then the holes are slotted.

part = "assembly";
$fn = 48;

// ── The switch board ────────────────────────────────────────────
bd_l     = 20.6;       // board length (along the lever)
bd_w     = 12.9;       // board width
bd_t     = 1.6;        // bare PCB
hole_d   = 3.2;        // corner holes, M3 clearance
hole_dx  = 18.3;       // from the measured 21.13 mm diagonal, assuming equal margins at both corners
hole_dy  = 10.6;       // as above; the diagonal alone cannot separate the two
slot     = 1.0;        // slot length each way: covers the unknown split of the diagonal
under    = 2.5;        // standoff: just enough for the solder tails under the board
lever_up = 5;          // how far the lever stands above the switch body when at rest
body_h   = 6.5;        // switch body above the board

// ── Where they go on the head ───────────────────────────────────
r_sw     = 36;         // radius of the switch levers and the flag from the pan axis
sw_angle = 95;         // flag meets the lever at +/- this many degrees from straight ahead
deck_to_yoke = 22;     // clear height between the base deck and the yoke underside
flag_len = 8;          // how far the flag hangs below the yoke. Set by the switch, not by taste: the
                       // lever only travels about 3 mm, so the flag's edge sits about 3 mm below the
                       // lever's rest height and must stay clear of the switch body beneath it
flag_w   = 6;
flag_t   = 3;

// ── Pan bracket: glued flat to the deck, board lying on it ──────
module pan_bracket()
{
    plate_l = bd_l + 4;
    plate_w = bd_w + 4;
    difference()
    {
        union()
        {
            translate([-plate_l / 2, -plate_w / 2, 0]) cube([plate_l, plate_w, 2]);  // foot
            for (sx = [-1, 1]) translate([sx * hole_dx / 2, sx * hole_dy / 2, 0])
                cylinder(d = 7, h = 2 + under);                                     // standoffs under the holes
            // a lip the board's far edge rests on, so it cannot tilt when pressed
            translate([-plate_l / 2, plate_w / 2 - 2, 0]) cube([plate_l, 2, 2 + under]);
        }
        for (sx = [-1, 1]) translate([sx * hole_dx / 2, sx * hole_dy / 2, -1]) hull()
        {
            translate([-slot, 0, 0]) cylinder(d = hole_d, h = 10);
            translate([ slot, 0, 0]) cylinder(d = hole_d, h = 10);
        }
        // M3 nut pockets underneath, so the screws need no spanner
        for (sx = [-1, 1]) translate([sx * hole_dx / 2, sx * hole_dy / 2, -1]) hull()
        {
            translate([-slot, 0, 0]) cylinder(d = 5.7 / cos(30), h = 3.5, $fn = 6);
            translate([ slot, 0, 0]) cylinder(d = 5.7 / cos(30), h = 3.5, $fn = 6);
        }
    }
}

// ── Pan flag: glued to the yoke's underside, blade hangs down ───
module pan_flag()
{
    pad = 14;
    translate([-pad / 2, -pad / 2, 0]) cube([pad, pad, flag_t]);                     // glue pad
    // blade: ramped on both faces along the sweep, so it rides onto the lever from either side
    hull()
    {
        translate([-flag_w / 2, -flag_t / 2, 0]) cube([flag_w, flag_t, 1]);
        translate([-flag_w / 2 + 2, -flag_t / 2, -flag_len + 1]) cube([flag_w - 4, flag_t, 1]);
    }
}

// ── Mock switch board for the assembly view ─────────────────────
module board()
{
    color("darkgreen") translate([-bd_l / 2, -bd_w / 2, 0]) cube([bd_l, bd_w, bd_t]);
    color("#222") translate([-bd_l / 2 + 3, -bd_w / 2 + 2, bd_t]) cube([13, 6, body_h]);     // switch body
    color("silver") translate([-bd_l / 2 + 3, -bd_w / 2 + 2, bd_t + body_h]) rotate([0, -8, 0]) cube([17, 1, 0.5]); // lever
    color("white") translate([bd_l / 2 - 6, -bd_w / 2 + 1, bd_t]) cube([4, 10, 3]);           // plug
}

module assembly()
{
    // the deck is z = 0 here; the yoke's underside is deck_to_yoke above it
    color("steelblue", 0.3) translate([-60, -40, -5]) cube([120, 80, 5]);
    // each board faces its lever tip toward the flag, so the flag presses the tip, not the hinge
    for (s = [-1, 1]) rotate(s * sw_angle) translate([r_sw, 0, 0]) rotate(-s * 90)
    {
        color("white") pan_bracket();
        translate([0, 0, 2 + under]) board();
    }
    color("orange") translate([r_sw, 0, deck_to_yoke - flag_t]) pan_flag();
    color("gray", 0.3) translate([-45, -26, deck_to_yoke]) cube([110, 52, 5]);
}

if (part == "assembly")   assembly();
if (part == "pan_bracket") pan_bracket();
if (part == "pan_flag")    translate([0, 0, flag_t]) rotate([180, 0, 0]) pan_flag();   // pad down, blade up
