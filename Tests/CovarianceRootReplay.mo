within Tests;

block CovarianceRootReplay
  input Real columns[15, 30];
  output Real root[15, 15](each start=0, each fixed=true);
algorithm
  when sample(0, 0.01) then
    root := LinearAlgebra.covarianceRoot(columns);
  end when;
end CovarianceRootReplay;
