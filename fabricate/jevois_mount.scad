inch = 25.4;
W = 32;
H = 36;
w = 1 * inch;
h = 1.175 * inch;

T = 4;

difference(){
  translate([-W/2, -H/2 - T, 0])cube([W, H + T, T]);
  translate([-w/2, -h/2, ,-1])cylinder(h=10, r=1.5, $fn=50);
  translate([-w/2,  h/2, ,-1])cylinder(h=10, r=1.5, $fn=50);
  translate([ w/2, -h/2, ,-1])cylinder(h=10, r=1.5, $fn=50);
  translate([ w/2,  h/2, ,-1])cylinder(h=10, r=1.5, $fn=50);
  translate([-w/2, -h/2, ,-7])cylinder(h=10, r=2.5, $fn=50);
  translate([-w/2,  h/2, ,-7])cylinder(h=10, r=2.5, $fn=50);
  translate([ w/2, -h/2, ,-7])cylinder(h=10, r=2.5, $fn=50);
  translate([ w/2,  h/2, ,-7])cylinder(h=10, r=2.5, $fn=50);
  translate([-50, -H/2 + 30 - 1, -18])cube([100, 50, 20]);
}

translate([-8 - 6, -H/2 - T, 0])rotate(a=-90, v=[1, 0, 0])
intersection(){
  difference(){
    cylinder(r=6.2, h=T, $fn=50);
    translate([2.6, 3.1, -1])cylinder(h=100, r=1.25, $fn=50);
    translate([2.6, 3.1,  2])cylinder(h=1, r2=1.25, r1=2, $fn=50);
    translate([2.6, 3.1, -8])cylinder(h=10, r2=2, r1=2, $fn=50);
  }
  cube(100, 100, 100);
    
}

difference(){
  translate([8 + 6, -H/2 - T, 0])rotate(a=-90, v=[1, 0, 0])rotate(a=90, v=[0, 0, 1])
    intersection(){
    difference(){
      cylinder(r=6.2, h=T, $fn=50);
      translate([3.1, 2.6, -1])cylinder(h=100, r=1.25, $fn=50);
      translate([3.1, 2.6, 2])cylinder(h=1.1, r2=1.25, r1=2, $fn=50);
      translate([3.1, 2.6, -8])cylinder(h=10, r2=2, r1=2, $fn=50);
    }
    cube(100, 100, 100);
  }
}

//#translate([-.9*inch/2, -50, -10])cube([.9*inch, 100, 10]);
//#translate([-8, -50, -10])cube([16, 100, 10]);


