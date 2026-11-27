within;
package Meshing "Three-dimensional finite-volume meshes for closed triangulated solids"
  annotation(uses(Modelica(version="4.1.0")), Documentation(info="<html>
<p>Use <a href=\"modelica://Meshing.ExternalFlowMesh\">ExternalFlowMesh</a>
to mesh an ASCII STL at initialization and export it for OpenFOAM.
<a href=\"modelica://Meshing.generate\">generate</a> accepts triangle coordinates
directly and returns a <a href=\"modelica://Meshing.PolyMesh\">PolyMesh</a>.</p>
<p>The fluid occupies the Cartesian box outside the closed bodies. Coordinates
are metres. Internal faces point from owner to neighbour and boundary faces
point out of the fluid. Connectivity indices are one-based inside Modelica.</p>
<p>Optional refinement boxes control resolution in the wake.
<a href=\"modelica://Meshing.addWallLayers\">addWallLayers</a> inserts a stack
of cells between the body and the core mesh. The STL entry point checks
positive cell volume and closure before export. Inspect mesh quality and
surface fidelity at the chosen resolution before running a flow solver.</p>
</html>"));
end Meshing;
