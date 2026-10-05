// owlet stepper head, by Mrunmoy
// Copyright (c) 2026 Mrunmoy. SPDX-License-Identifier: MIT
// https://github.com/Mrunmoy/owlet
//
// Belt-driven pan-tilt head for two pancake NEMA 17 steppers.
// Each axis: 20T motor pulley -> 200 mm closed GT2 belt -> 60T pulley on a
// 5 mm shaft running in two 625ZZ bearings (3:1 reduction).
//
// Select what to render with:  openscad -D 'part="yoke"' ...
//   assembly | base_top | base_bottom | yoke | cradle | small_parts
//
// All dimensions in mm. Measure your own pulleys and adjust the
// "MEASURE" values before printing.

part = "assembly";
tilt_deg = 0;          // assembly view only
$fn = 72;

// ── Motor (from the seller drawing) ─────────────────────────────
m_w        = 42.3;     // body square
m_len      = 23.5;     // body length
m_boss_d   = 22;       // pilot boss
m_boss_h   = 2;
m_shaft_l  = 22;       // shaft length from mounting face
m_hole_sp  = 31;       // M3 hole square
conn_w     = 16.2;
conn_out   = 5.5;
conn_len   = 9.2;

// ── Drive train ─────────────────────────────────────────────────
belt_len   = 200;      // closed loop GT2
belt_w     = 6;
t_small    = 20;
t_big      = 60;
p20_h      = 16;       // MEASURE: total height of 20T pulley
p20_fl_d   = 16;       // MEASURE: flange diameter
p60_h      = 18;       // MEASURE: total height of 60T pulley incl. hub
p60_fl_d   = 42;       // MEASURE: flange diameter
tension    = 3;        // +/- motor slot travel for belt tension

pd_s = t_small * 2 / PI;
pd_b = t_big   * 2 / PI;
function centre_dist(L, D, d) =
    let (b = 2 * L - PI * (D + d)) (b + sqrt(b * b - 8 * pow(D - d, 2))) / 8;
cc = centre_dist(belt_len, pd_b, pd_s);   // ~58.6 mm
echo(str("Belt centre distance = ", cc, " mm"));

// ── Hardware fits ───────────────────────────────────────────────
brg_od     = 16;       // 625ZZ: 5 x 16 x 5
brg_w      = 5;
brg_fit    = 0.2;      // added to pocket diameter
brg_proud  = 1;        // bearing sticks out of deck/arm by this much
shaft_d    = 5;
shaft_fit  = 0.3;
m3_clear   = 3.4;
m3_tap     = 2.8;      // self-tapping pilot in plastic
nut_af     = 5.7;      // M3 hex nut across flats, with clearance
nut_t      = 2.7;
spacer_t   = 1.5;

// ── Base ────────────────────────────────────────────────────────
bp_t       = 6;        // bottom plate
box_h      = 27;       // cavity for the pan motor
deck_t     = 5;
wall       = 3;
by         = 40;       // half width
bx0        = -(cc + m_w / 2 + tension + 4 + wall);
bx1        = 58;
ear        = 12;       // bottom plate overhang for hold-down screws
z_deck     = bp_t + box_h;
z_deck_top = z_deck + deck_t;
boss_d     = 9;
boss_pts   = [ [bx0 + wall + 2.5, by - wall - 2.5], [bx0 + wall + 2.5, -(by - wall - 2.5)],
               [bx1 - wall - 2.5, by - wall - 2.5], [bx1 - wall - 2.5, -(by - wall - 2.5)],
               [26, by - wall - 2.5],               [26, -(by - wall - 2.5)] ];

// ── Yoke ────────────────────────────────────────────────────────
yoke_gap   = 84;       // clear width between arms
arm_t      = 5;
arm_hw     = 26;       // arm half width
floor_t    = 5;
gus        = 20;       // gusset foot length
gus_h      = 35;
gus_t      = 3;
hub_d      = 20;
hub_h      = 14;
floor_clr  = 1.5;      // motor to floor at full slack
H_tilt     = cc + m_w / 2 + tension + floor_clr;   // floor top -> tilt axis
z_yoke     = z_deck_top + brg_proud + spacer_t + p60_h + spacer_t;   // floor underside
z_floor    = z_yoke + floor_t;
z_tilt     = z_floor + H_tilt;
z_tmotor   = z_tilt - cc;
xa         = yoke_gap / 2;                         // inner face of arm A

// ── Cradle ──────────────────────────────────────────────────────
cheek_t    = 8;
plat_drop  = 18;       // platform top below tilt axis (camera CoM near axis)
plat_t     = 4;
plat_hw    = 22;       // platform half depth
cr_x       = xa - spacer_t;                        // outer face of cheek

// ── Helpers ─────────────────────────────────────────────────────
// Cut for an M3 set screw with a captive nut. Shaft along Z through the
// origin, screw along +X, nut slot open towards +Z.
module setscrew_cut(len = 12, open = 20)
{
    rotate([0, 90, 0]) cylinder(d = m3_clear, h = len);
    translate([shaft_d / 2 + 1.6, -nut_af / 2, -3.3]) cube([nut_t, nut_af, open]);
}

module bearing_pocket()   // axis Z, pocket opens towards +Z from z = 0 plane top
{
    translate([0, 0, -(brg_w - brg_proud)]) cylinder(d = brg_od + brg_fit, h = 20);
    translate([0, 0, -20]) cylinder(d = 11, h = 40);
}

module motor_cut(t)       // slotted NEMA17 face pattern, slots along X, plate 0..t in Z
{
    hull() for (d = [-tension, tension])
        translate([d, 0, -1]) cylinder(d = m_boss_d + 1, h = t + 2);
    for (sx = [-1, 1], sy = [-1, 1])
        hull() for (d = [-tension, tension])
            translate([sx * m_hole_sp / 2 + d, sy * m_hole_sp / 2, -1])
                cylinder(d = m3_clear, h = t + 2);
}

// ── Printed parts (assembly coordinates) ────────────────────────
module base_top()
{
    difference()
    {
        union()
        {
            translate([bx0, -by, z_deck]) cube([bx1 - bx0, 2 * by, deck_t]);
            difference()
            {
                translate([bx0, -by, bp_t]) cube([bx1 - bx0, 2 * by, box_h]);
                translate([bx0 + wall, -by + wall, bp_t - 1])
                    cube([bx1 - bx0 - 2 * wall, 2 * by - 2 * wall, box_h + 1]);
            }
            for (p = boss_pts) translate([p[0], p[1], bp_t]) cylinder(d = boss_d, h = box_h);
        }
        translate([0, 0, z_deck_top]) bearing_pocket();
        translate([-cc, 0, z_deck]) motor_cut(deck_t);
        // side windows, open at the bottom edge: cable exit, airflow, collar access
        for (sy = [-1, 1])
        {
            translate([-cc - 20, sy * by - wall - 1, bp_t - 1]) cube([40, 2 * wall + 2, 21]);
            translate([-15, sy * by - wall - 1, bp_t - 1]) cube([30, 2 * wall + 2, 21]);
        }
        for (p = boss_pts) translate([p[0], p[1], bp_t - 1]) cylinder(d = m3_tap, h = 15);
    }
}

module base_bottom()
{
    difference()
    {
        translate([bx0, -by - ear, 0]) cube([bx1 - bx0, 2 * (by + ear), bp_t]);
        translate([0, 0, bp_t - brg_w]) cylinder(d = brg_od + brg_fit, h = 10);
        translate([0, 0, -1]) cylinder(d = 9, h = 10);
        for (p = boss_pts)
        {
            translate([p[0], p[1], -1]) cylinder(d = m3_clear, h = 10);
            translate([p[0], p[1], -1]) cylinder(d = 6.5, h = 4);
        }
        for (x = [bx0 + 12, bx1 - 12], sy = [-1, 1])
            translate([x, sy * (by + ear / 2), -1]) cylinder(d = 4.5, h = 10);
    }
}

module arm_blank()        // in YZ, extruded along +X by arm_t, from floor top
{
    rotate([90, 0, 90]) linear_extrude(arm_t) hull()
    {
        translate([-arm_hw, 0]) square([2 * arm_hw, m_w + 2 * tension + floor_clr + 6]);
        translate([0, H_tilt]) circle(r = 14);
    }
}

module gusset()           // foot along +X, at origin
{
    rotate([90, 0, 0]) linear_extrude(gus_t) polygon([[0, 0], [gus, 0], [0, gus_h]]);
}

module yoke()
{
    difference()
    {
        union()
        {
            translate([-xa - arm_t, -arm_hw, z_yoke])
                cube([2 * xa + 2 * arm_t + gus, 2 * arm_hw, floor_t]);
            translate([xa, 0, z_floor]) arm_blank();                     // arm A (motor side)
            translate([-xa - arm_t, 0, z_floor]) arm_blank();            // arm B
            translate([0, 0, z_floor]) cylinder(d = hub_d, h = hub_h);
            for (sy = [-1, 1])
            {
                translate([xa + arm_t, sy * arm_hw + (sy > 0 ? 0 : gus_t), z_floor]) gusset();
                translate([-xa, sy * arm_hw + (sy > 0 ? 0 : gus_t), z_floor]) gusset();
            }
        }
        // pan shaft bore and set screw (driver access from -Y)
        translate([0, 0, z_yoke - 1]) cylinder(d = shaft_d + shaft_fit, h = floor_t + hub_h + 2);
        translate([0, 0, z_floor + 8]) rotate([0, 0, -90]) setscrew_cut();
        // tilt motor face on arm A, slots vertical
        translate([xa, 0, z_tmotor]) rotate([0, 90, 0]) motor_cut(arm_t);
        // tilt bearings, pockets open to the outside faces
        translate([xa + arm_t, 0, z_tilt]) rotate([0, 90, 0]) bearing_pocket();
        translate([-xa - arm_t, 0, z_tilt]) rotate([0, -90, 0]) bearing_pocket();
    }
}

module cheek()            // +X side, coordinates relative to the tilt axis
{
    difference()
    {
        translate([cr_x - cheek_t, 0, 0]) rotate([90, 0, 90]) linear_extrude(cheek_t) hull()
        {
            circle(d = hub_d);
            translate([-plat_hw, -plat_drop - plat_t]) square([2 * plat_hw, plat_t]);
        }
        translate([cr_x - cheek_t - 1, 0, 0]) rotate([0, 90, 0])
            cylinder(d = shaft_d + shaft_fit, h = cheek_t + 2);
        translate([cr_x - cheek_t / 2, 0, 0]) rotate([0, -90, 0]) setscrew_cut(open = 10);
    }
}

module cradle()           // relative to the tilt axis
{
    for (m = [0, 1]) mirror([m, 0, 0]) cheek();
    difference()
    {
        translate([-cr_x, -plat_hw, -plat_drop - plat_t]) cube([2 * cr_x, 2 * plat_hw, plat_t]);
        translate([0, 0, -plat_drop - plat_t - 1])
        {
            cylinder(d = 6.6, h = plat_t + 2);                           // 1/4" camera screw
            for (sx = [-1, 1], sy = [-1, 1])                             // strap / zip-tie slots
                translate([sx * 22 - 7, sy * 14 - 1.5, 0]) cube([14, 3, plat_t + 2]);
            for (sx = [-1, 1]) translate([sx * 10, 0, 0]) cylinder(d = m3_clear, h = plat_t + 2);
        }
    }
}

module collar()
{
    difference()
    {
        cylinder(d = 18, h = 8);
        translate([0, 0, -1]) cylinder(d = shaft_d + shaft_fit, h = 10);
        translate([0, 0, 4]) setscrew_cut();
    }
}

module spacer()
{
    difference()
    {
        cylinder(d = 8, h = spacer_t);
        translate([0, 0, -1]) cylinder(d = shaft_d + 0.5, h = spacer_t + 2);
    }
}

module small_parts()
{
    collar();
    for (i = [0 : 5]) translate([16 + 11 * (i % 3), 11 * floor(i / 3) - 5, 0]) spacer();
}

// ── Mock-ups for the assembly view ──────────────────────────────
module motor()            // mounting face at z = 0, shaft along +Z, connector towards +Y
{
    color("dimgray") translate([-m_w / 2, -m_w / 2, -m_len]) cube([m_w, m_w, m_len]);
    color("silver") cylinder(d = m_boss_d, h = m_boss_h);
    color("silver") cylinder(d = 5, h = m_shaft_l);
    color("white") translate([-conn_w / 2, m_w / 2, -m_len]) cube([conn_w, conn_out, conn_len]);
}

module drive(z_small, z_big)   // small pulley at origin, big pulley at +X cc, belt plane Z
{
    color("gainsboro") translate([0, 0, z_small]) cylinder(d = p20_fl_d, h = p20_h);
    color("gainsboro") translate([cc, 0, z_big]) cylinder(d = p60_fl_d, h = p60_h);
    color("black") translate([0, 0, z_big + 1]) linear_extrude(belt_w) difference()
    {
        hull() { circle(d = pd_s + 1.5); translate([cc, 0]) circle(d = pd_b + 1.5); }
        hull() { circle(d = pd_s); translate([cc, 0]) circle(d = pd_b); }
    }
}

module assembly()
{
    color("steelblue") base_top();
    color("slategray") base_bottom();
    color("orange") yoke();
    translate([0, 0, z_tilt]) rotate([tilt_deg, 0, 0])
    {
        color("seagreen") cradle();
        color("#333") translate([-35, -12, -plat_drop]) cube([70, 24, 30]);   // camera stand-in
    }
    // pan drive
    translate([-cc, 0, z_deck]) motor();
    translate([-cc, 0, 0]) drive(z_deck_top + 1, z_deck_top + brg_proud + spacer_t);
    color("silver") translate([0, 0, 1.5]) cylinder(d = shaft_d, h = z_floor + hub_h - 1.5);
    color("gold") translate([0, 0, z_deck - 9]) collar();
    // tilt drive
    translate([xa, 0, z_tmotor]) rotate([0, 90, 0]) motor();
    translate([xa + arm_t, 0, z_tmotor]) rotate([0, 90, 0]) rotate([0, 0, 180])
        translate([0, 0, 0]) drive(1, brg_proud + spacer_t);
    color("silver") translate([-xa - arm_t - 2, 0, z_tilt]) rotate([0, 90, 0])
        cylinder(d = shaft_d, h = 2 * xa + 2 * arm_t + 2 + brg_proud + spacer_t + p60_h + 1);
}

// ── Output (parts are placed in print orientation) ──────────────
if (part == "assembly")         assembly();
else if (part == "base_top")    translate([0, 0, z_deck_top]) rotate([180, 0, 0]) base_top();
else if (part == "base_bottom") base_bottom();
else if (part == "yoke")        translate([0, 0, -z_yoke]) yoke();
else if (part == "cradle")      translate([0, 0, plat_drop + plat_t]) cradle();
else if (part == "small_parts") small_parts();

// ── Pulley bore reducer (60T pulleys arrived with 8 mm bore) ────
// Solid sleeve: 5 mm inside, 8 mm outside.
//   pulley_bushing      one sleeve at bush_od
//   pulley_bushing_set  one sleeve of each size in bush_ods, for finding the
//                       best press fit. Rim notches identify the size:
//                       0 notches = first entry, 1 = second, ...
bush_od    = 8.1;      // pulley bore measured 7.99; printed 8.1 slides in, 8.15 and 8.2 will not enter
bush_ods   = [8.0, 8.1, 8.2, 8.3, 8.4, 8.5];
bush_id    = 5.2;
bush_len   = p60_h;    // trim or re-export to match the pulley height

module pulley_bushing(od = bush_od, notches = 0)
{
    difference()
    {
        cylinder(d = od, h = bush_len);
        translate([0, 0, -1]) cylinder(d = bush_id, h = bush_len + 2);
        for (k = [0 : 1 : notches - 1])
            rotate([0, 0, k * 40]) translate([od / 2 - 0.6, -0.5, bush_len - 0.8]) cube([2, 1, 2]);
    }
}

if (part == "pulley_bushing") pulley_bushing();
if (part == "pulley_bushing_set")
    for (i = [0 : len(bush_ods) - 1]) translate([14 * i, 0, 0]) pulley_bushing(bush_ods[i], i);
