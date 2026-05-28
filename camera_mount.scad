// =============================================================================
//  Elfin S20  –  EOF Camera Mount
//  Designed for the end-of-flange (elfin_end_link) of the Elfin S20 robot arm.
//
//  Target camera: Intel RealSense D435 (90 × 25 × 25 mm)
//  Flange: 4× M6 bolts on Ø63 mm PCD  (ISO 9283 / HuaYan S20 spec)
//
//  Print settings:
//    Layer height : 0.2 mm
//    Infill       : 40 %  (PETG or ABS recommended)
//    Supports     : Yes (for camera cradle overhangs)
//    Material     : PETG / ABS  (no PLA – robot vibration + heat)
// =============================================================================

// ---------- Adjustable parameters -------------------------------------------

/* [Flange Interface] */
FLANGE_PCD        = 63;    // mm  – pitch circle diameter for bolt holes
FLANGE_BOLTS      = 4;     // number of M6 bolt holes
BOLT_DIA          = 6.5;   // mm  – clearance for M6
FLANGE_PLATE_DIA  = 80;    // mm  – outer diameter of adapter plate
FLANGE_PLATE_THK  = 6;     // mm  – thickness of adapter plate
CENTER_BORE       = 16;    // mm  – central pilot bore

/* [Arm / Bracket] */
ARM_LENGTH        = 55;    // mm  – distance from flange face to camera CL
ARM_WIDTH         = 18;    // mm  – cross-section width
ARM_HEIGHT        = 12;    // mm  – cross-section height
ARM_TAPER         = 0.7;   // 0=no taper, 1=full taper (aesthetic triangular arm)

/* [Camera Cradle – RealSense D435] */
CAM_W             = 90;    // mm  – camera body width
CAM_H             = 25;    // mm  – camera body height
CAM_D             = 25;    // mm  – camera body depth
CRADLE_WALL       = 3.5;   // mm  – wall thickness around camera
CRADLE_LIP        = 4;     // mm  – lip that retains the camera from the front
M2_DIA            = 2.3;   // mm  – M2 clearance for camera retention screws

/* [Cable Management] */
CABLE_SLOT_W      = 12;    // mm  – slot on arm back for cable routing
CABLE_SLOT_D      = 5;     // mm  – depth of cable slot

/* [General] */
$fn               = 64;
EPS               = 0.01;

// ---------- Derived values --------------------------------------------------

PLATE_R   = FLANGE_PLATE_DIA / 2;
PCD_R     = FLANGE_PCD / 2;

// ---------- Sub-modules -----------------------------------------------------

module bolt_hole_pattern(r, n, dia, depth) {
    for (i = [0 : n-1]) {
        angle = i * 360 / n;
        translate([r * cos(angle), r * sin(angle), -EPS])
            cylinder(d=dia, h=depth + 2*EPS);
    }
}

module chamfer_cylinder(r, h, ch=1) {
    hull() {
        cylinder(r=r, h=h-ch);
        translate([0,0,h-ch]) cylinder(r1=r, r2=r-ch, h=ch);
        translate([0,0,0])    cylinder(r1=r-ch, r2=r, h=ch);
    }
}

// ---------- Flange adapter plate --------------------------------------------
//  Sits directly on the robot EOF flange.

module flange_plate() {
    difference() {
        union() {
            // Main disc
            chamfer_cylinder(r=PLATE_R, h=FLANGE_PLATE_THK, ch=1.5);
            // Raised arm attachment boss on top face
            translate([0, 0, FLANGE_PLATE_THK - EPS])
                chamfer_cylinder(r=ARM_WIDTH/2 + 4, h=4, ch=1);
        }
        // Central pilot bore
        translate([0, 0, -EPS])
            cylinder(d=CENTER_BORE, h=FLANGE_PLATE_THK + 2*EPS);
        // Bolt holes
        bolt_hole_pattern(PCD_R, FLANGE_BOLTS, BOLT_DIA, FLANGE_PLATE_THK);
        // Anti-rotation notch (key)
        translate([-1.5, PLATE_R - 5, -EPS])
            cube([3, 8, FLANGE_PLATE_THK + 2*EPS]);
    }
}

// ---------- Arm / bracket ---------------------------------------------------
//  Connects the flange plate to the camera cradle.

module arm() {
    arm_z_offset = FLANGE_PLATE_THK + 4 - EPS;  // on top of boss

    hull() {
        // Root cross-section (wider)
        translate([0, 0, arm_z_offset])
            cube([ARM_WIDTH, ARM_HEIGHT, EPS], center=true);
        // Tip cross-section (slightly narrower if tapered)
        tip_w = ARM_WIDTH  * (1 - 0.2 * ARM_TAPER);
        tip_h = ARM_HEIGHT * (1 - 0.3 * ARM_TAPER);
        translate([0, ARM_LENGTH, arm_z_offset + ARM_HEIGHT])
            cube([tip_w, tip_h, EPS], center=true);
    }

    // Cable routing slot (subtracted below)
}

module arm_with_slot() {
    slot_y = ARM_LENGTH * 0.2;  // starts 20% along the arm
    difference() {
        arm();
        // Cable slot running along the arm
        translate([-CABLE_SLOT_W/2, slot_y,
                   FLANGE_PLATE_THK + 4 + ARM_HEIGHT/2 - CABLE_SLOT_D])
            cube([CABLE_SLOT_W, ARM_LENGTH * 0.7, CABLE_SLOT_D + EPS]);
    }
}

// ---------- Camera cradle ---------------------------------------------------
//  U-shaped pocket that grips the camera.  Camera slides in from the +Y side.

module camera_cradle() {
    outer_w = CAM_W + 2*CRADLE_WALL;
    outer_h = CAM_H + 2*CRADLE_WALL;
    outer_d = CAM_D + CRADLE_WALL;       // open at front for lens access

    arm_z_offset = FLANGE_PLATE_THK + 4 + ARM_HEIGHT;

    translate([0, ARM_LENGTH, arm_z_offset]) {
        rotate([90, 0, 0]) {   // cradle faces in +Y direction (forward)
            difference() {
                // Outer block
                translate([-outer_w/2, -outer_h/2, 0])
                    cube([outer_w, outer_h, outer_d]);

                // Camera pocket (open top for sliding in)
                translate([-CAM_W/2, -CAM_H/2, CRADLE_WALL])
                    cube([CAM_W, CAM_H + outer_h, CAM_D + EPS]);  // open top

                // Lens opening – keep bottom and side walls only
                translate([-CAM_W/2 + 5, -CAM_H/2 + 5, -EPS])
                    cube([CAM_W - 10, CAM_H - 10, CRADLE_WALL + 2*EPS]);

                // M2 retention screw holes (2× on each side)
                for (side = [-1, 1])
                    translate([side * (outer_w/2 + EPS),
                               0, CRADLE_WALL + CAM_D/2])
                        rotate([0, 90*side, 0])
                            cylinder(d=M2_DIA, h=CRADLE_WALL + 2*EPS);
            }

            // Retaining lip (front) – prevents camera sliding forward
            translate([-outer_w/2, -CRADLE_LIP/2, outer_d - EPS])
                cube([outer_w, CRADLE_LIP, CRADLE_LIP]);

            // Cable strain-relief clip on the back
            translate([-6, outer_h/2 - 3, 0])
                difference() {
                    cube([12, 8, 5]);
                    translate([2, -EPS, 1]) cube([8, 6, 3]);
                }
        }
    }
}

// ---------- Assembly --------------------------------------------------------

module full_assembly() {
    color("#7a7aaa") flange_plate();
    color("#3a3a6a") arm_with_slot();
    color("#2c3e50") camera_cradle();
}

full_assembly();

// ---------- Exploded / print-ready parts ------------------------------------
// Uncomment to export individual parts for printing:

// ── Plate (print flat on bed):
// translate([0, 0, 0]) flange_plate();

// ── Arm + cradle (may need supports):
// translate([120, 0, 0]) {
//     arm_with_slot();
//     camera_cradle();
// }
