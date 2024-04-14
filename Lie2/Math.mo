within Lie2;

package Math
  function trace
    input Real[:, :] a;
    output Real res;
  algorithm
    res := 0;
  for i in 1:min(size(a)) loop
      res := res + a[i, i];
    end for;
  end trace;
  
  function norm2
    input Real[:] a;
    output Real res;
  algorithm
    res := sqrt(a*a);
  end norm2;
  
end Math;