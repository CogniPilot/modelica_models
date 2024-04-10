within Lie2.Examples;

model TestSO3
  import Lie2.SO3;
  
  SO3.Dcm.Group.Element g1(r={
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}});
    
  SO3.Dcm.Group.Element g2(r={
    {0, 1, 0},
    {1, 0, 0},
    {0, 0, 1}});
  
  SO3.Dcm.Group.Element g3;
  
equation
  g3 = SO3.Dcm.Group.product(g1, g2);
end TestSO3;