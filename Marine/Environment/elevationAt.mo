within Marine.Environment;
function elevationAt "Periodic bilinear elevation on an east-north grid"
  input Real grid[:,:],east,north,period;
  output Real elevation;
protected
  Integer rows,cols,i,j,ii,jj;
  Real x,y,a,b;
algorithm
  rows:=size(grid,1); cols:=size(grid,2);
  x:=east*cols/period; y:=north*rows/period;
  i:=1+mod(integer(floor(x)),cols); j:=1+mod(integer(floor(y)),rows);
  ii:=1+mod(i,cols); jj:=1+mod(j,rows);
  a:=x-floor(x); b:=y-floor(y);
  elevation:=(1-b)*((1-a)*grid[j,i]+a*grid[j,ii])+b*((1-a)*grid[jj,i]+a*grid[jj,ii]);
end elevationAt;
