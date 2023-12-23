$fn=100;
w = 63.5;
h = 30;
ww = 1.4;

relayW = 46;
espW = 25.5;
thermW = 22.5;
thermPinD = 3.4;
relayPinD = 4.5;
thermPinSpace = 14.5;

difference() {
    union() {
        cube([w, relayW+espW+thermW, ww]);

        translate([(w-47)/2, 22.5, 0]) {
            cylinder(h=h, d=relayPinD);
            translate([47, 0, 0]) {
                cylinder(h=h, d=relayPinD);
            }
        }

        translate([w/2, relayW+espW+((thermW-14)/2), 0]) {
            cylinder(h=h, d=thermPinD);
            translate([0, thermPinSpace, 0]) {
                cylinder(h=h, d=thermPinD);
            }
        }

        linear_extrude(h) {
            difference() {
                offset(ww) {
                    square([w, relayW+espW+thermW]);
                }
                square([w, relayW+espW+thermW]);
            }
        }
    }

    // thermostat cable
    translate([10, relayW+espW+((thermW)/2), 10]) {
        rotate([0, 90, 0]) {
            cylinder(d=8, h=100);
        }
    }

    // esp power cable
    translate([10, relayW+(espW/2), 10]) {
        rotate([0, 90, 0]) {
            cylinder(d=10, h=100);
        }
    }

    // ac load in
    translate([-90, 10, 21.5]) {
        rotate([0, 90, 0]) {
            cylinder(d=7, h=100);
        }
    }

    // ac load out
    translate([-90, (relayW-10), 21.5]) {
        rotate([0, 90, 0]) {
            cylinder(d=7, h=100);
        }
    }
}


translate([w+10, 0, 0]) {
    linear_extrude(ww) {
        offset(ww*2) {
            square([w, relayW+espW+thermW]);
        }
    }

    linear_extrude(6) {
        difference() {
            offset(ww*2) {
                square([w, relayW+espW+thermW]);
            }
            offset(ww) {
                square([w, relayW+espW+thermW]);
            }
        }
    }

    linear_extrude(3) {
        translate([(w-47)/2, 22.5, 0]) {
            difference() {
                offset(ww) circle(d=relayPinD);
                circle(d=relayPinD);
            }
            translate([47, 0, 0]) {
                difference() {
                    offset(ww) circle(d=relayPinD);
                    circle(d=relayPinD);
                }
            }
        }

        translate([w/2, relayW+espW+((thermW-14)/2), 0]) {
            difference() {
                offset(ww) circle(d=thermPinD);
                circle(d=thermPinD);
            }
            translate([0, thermPinSpace, 0]) {
                difference() {
                    offset(ww) circle(d=thermPinD);
                    circle(d=thermPinD);
                }
            }
        }
    }
}
