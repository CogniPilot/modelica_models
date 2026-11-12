within Vehicles.Microduck;
package Vision "Image processing of Three.js sensor pixels"
  function redCentroid
    input Real rgb[:,:,3];
    output Real centroid[2];
    output Boolean visible;
  protected
    Real weight;
    Real total;
    Real horizontal;
    Real vertical;
  algorithm
    total:=0;
    horizontal:=0;
    vertical:=0;
    centroid:={0,0};
    visible:=false;
    for row in 1:size(rgb,1) loop
      for column in 1:size(rgb,2) loop
        weight:=max(0,rgb[row,column,1]-max(rgb[row,column,2],rgb[row,column,3]));
        total:=total+weight;
        horizontal:=horizontal+weight*(column-0.5)/size(rgb,2);
        vertical:=vertical+weight*(row-0.5)/size(rgb,1);
      end for;
    end for;
    if total>1e-6 then
      centroid:={2*horizontal/total-1,1-2*vertical/total};
      visible:=true;
    end if;
  end redCentroid;
end Vision;
