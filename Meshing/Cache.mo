within Meshing;
package Cache "Content keys and atomic publication for generated mesh artifacts"
  constant String generatorVersion="Meshing.meshSTL/1" "Increment when a change affects generated mesh values";
  impure function key
    input String fileName,recipe;
    output String digest;
    external "C" digest=mesh_key(fileName,recipe)
      annotation(Include="#include \"meshing_cache.h\"",IncludeDirectory="modelica://Meshing/Resources/Include");
  end key;
  impure function temporary
    input String destination;
    output String path;
    external "C" path=mesh_temp(destination)
      annotation(Include="#include \"meshing_cache.h\"",IncludeDirectory="modelica://Meshing/Resources/Include");
  end temporary;
  impure function publish
    input String temporaryPath,destination;
    external "C" mesh_publish(temporaryPath,destination)
      annotation(Include="#include \"meshing_cache.h\"",IncludeDirectory="modelica://Meshing/Resources/Include");
  end publish;
  impure function verified
    input String path,signature,key;
    output Boolean valid;
    external "C" valid=mesh_verified(path,signature,key)
      annotation(Include="#include \"meshing_cache.h\"",IncludeDirectory="modelica://Meshing/Resources/Include");
  end verified;
  impure function sign
    input String path,signature,key;
    external "C" mesh_sign(path,signature,key)
      annotation(Include="#include \"meshing_cache.h\"",IncludeDirectory="modelica://Meshing/Resources/Include");
  end sign;
end Cache;
