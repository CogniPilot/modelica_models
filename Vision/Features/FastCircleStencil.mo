within Vision.Features;
// FAST-9 samples a radius-three circle, not every cell of a square patch.
package FastCircleStencil
  constant Integer radius = 3;
  constant Integer sampleCount = 16;
  // Row/column offsets, clockwise from the top of the radius-three circle.
  constant Integer offsets[sampleCount,2] = [
    -3,0; -3,1; -2,2; -1,3; 0,3; 1,3; 2,2; 3,1;
    3,0; 3,-1; 2,-2; 1,-3; 0,-3; -1,-3; -2,-2; -3,-1];
end FastCircleStencil;
