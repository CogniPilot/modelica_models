within SLAM.PoseGraph;
// Full-capacity selected covariance of the FINAL undamped pose-graph linearization.
// Exact-real variational/tree bound; ordinary floating evaluation is explicitly
// not an outward-rounded certificate. No optimizer damping is statistical noise.
package ModelicaPoseGraphCovariance
  import PGCholesky = SLAM.PoseGraph.PGCholesky;
  import PGDot = SLAM.PoseGraph.PGDot;
  import PGEdge = SLAM.PoseGraph.PGEdge;
  import PGNormalProduct = SLAM.PoseGraph.PGNormalProduct;
  import PGSkew = SLAM.PoseGraph.PGSkew;
  import PGSolveBlock = SLAM.PoseGraph.PGSolveBlock;
  import PGValidateGraph = SLAM.PoseGraph.PGValidateGraph;

  constant Integer nodeCapacity=128;
  constant Integer edgeCapacity=256;
  constant Integer poseDimension=6;
  constant Integer selectedDimension=12;
  constant Integer maximumIterations=96;

  record Result
    Real upper[selectedDimension,selectedDimension];
    Real lower[selectedDimension,selectedDimension];
    Real residualUpper[selectedDimension,selectedDimension];
    Boolean accepted;
    Integer status "0 idle;1 numerical result;-1 config/selection;-2 graph;-3 tree/pivot;-4 chart;-5 arithmetic";
    Integer iterations[selectedDimension];
    Boolean converged[selectedDimension];
    Real residualNorm[selectedDimension];
    Integer treeEdges;
    Integer activeNodes;
    Boolean roundoffCertified "Always false: no directed-rounding enclosure in this profile";
  end Result;

  record Tree
    Integer parent[nodeCapacity];
    Integer order[nodeCapacity];
    Integer count;
    Real childInverse[nodeCapacity,poseDimension,poseDimension];
    Real parentJacobian[nodeCapacity,poseDimension,poseDimension];
    Real weightFactor[nodeCapacity,poseDimension,poseDimension];
    Boolean valid;
  end Tree;

  record Coarse
    Real basis[nodeCapacity,poseDimension,poseDimension];
    Real product[nodeCapacity,poseDimension,poseDimension];
    Real factor[poseDimension,poseDimension];
    Boolean valid;
  end Coarse;

  function EmptyCoarse
    output Coarse coarse;
  algorithm
    coarse.basis:=zeros(nodeCapacity,poseDimension,poseDimension);
    coarse.product:=zeros(nodeCapacity,poseDimension,poseDimension);
    coarse.factor:=zeros(poseDimension,poseDimension); coarse.valid:=false;
  end EmptyCoarse;

  function Empty
    output Result result;
  algorithm
    result.upper:=zeros(selectedDimension,selectedDimension);
    result.lower:=zeros(selectedDimension,selectedDimension);
    result.residualUpper:=zeros(selectedDimension,selectedDimension);
    result.accepted:=false; result.status:=0;
    result.iterations:=fill(0,selectedDimension); result.converged:=fill(false,selectedDimension);
    result.residualNorm:=zeros(selectedDimension); result.treeEdges:=0; result.activeNodes:=0;
    result.roundoffCertified:=false;
  end Empty;

  // Checked small general inverse. Child Jacobians are not SPD matrices.
  // The inverse closure is a numerical rank diagnostic, not rounded interval proof.
  function Inverse6
    input Real A[poseDimension,poseDimension];
    output Real inverse[poseDimension,poseDimension];
    output Boolean valid;
  protected
    Real work[poseDimension,poseDimension]; Real scale; Real pivot; Real swap;
    Real largest; Real factor; Real closure; Integer selected;
  algorithm
    inverse:=identity(poseDimension); work:=A; scale:=0.0; valid:=true;
    pivot:=0.0; swap:=0.0; largest:=0.0; factor:=0.0; closure:=0.0; selected:=1;
    for row in 1:poseDimension loop
      for column in 1:poseDimension loop
        valid:=valid and abs(A[row,column])<=1e18;
        scale:=max(scale,abs(A[row,column]));
      end for;
    end for;
    valid:=valid and scale>1e-12;
    for column in 1:poseDimension loop
      if valid then
        selected:=column; largest:=abs(work[column,column]);
        for row in 1:poseDimension loop
          if row>column and abs(work[row,column])>largest then
            selected:=row; largest:=abs(work[row,column]);
          end if;
        end for;
        valid:=largest>1e-12*scale and largest<=1e18;
        if valid then
          for k in 1:poseDimension loop
            swap:=work[column,k]; work[column,k]:=work[selected,k]; work[selected,k]:=swap;
            swap:=inverse[column,k]; inverse[column,k]:=inverse[selected,k]; inverse[selected,k]:=swap;
          end for;
          pivot:=work[column,column];
          for k in 1:poseDimension loop
            work[column,k]:=work[column,k]/pivot; inverse[column,k]:=inverse[column,k]/pivot;
          end for;
          for row in 1:poseDimension loop
            if row<>column then
              factor:=work[row,column];
              for k in 1:poseDimension loop
                work[row,k]:=work[row,k]-factor*work[column,k];
                inverse[row,k]:=inverse[row,k]-factor*inverse[column,k];
              end for;
            end if;
          end for;
        end if;
      end if;
    end for;
    for row in 1:poseDimension loop
      for column in 1:poseDimension loop
        closure:=sum(inverse[row,k]*A[k,column] for k in 1:poseDimension)
          -(if row==column then 1.0 else 0.0);
        valid:=valid and abs(closure)<1e-8 and abs(inverse[row,column])<=1e18;
      end for;
    end for;
    if not valid then inverse:=zeros(poseDimension,poseDimension); end if;
  end Inverse6;

  function BuildTree
    input Real nodeMask[nodeCapacity]; input Real edgeMask[edgeCapacity];
    input Integer source[edgeCapacity]; input Integer target[edgeCapacity];
    input Real Ji[edgeCapacity,poseDimension,poseDimension];
    input Real Jj[edgeCapacity,poseDimension,poseDimension];
    input Real information[edgeCapacity,poseDimension,poseDimension];
    output Tree tree;
  protected
    Boolean reached[nodeCapacity]; Boolean layer[nodeCapacity]; Boolean inverseValid; Boolean weightValid;
    Integer parent; Integer child; Real childJacobian[poseDimension,poseDimension];
    Real parentJacobian[poseDimension,poseDimension]; Real inverse[poseDimension,poseDimension];
    Real weight[poseDimension,poseDimension];
  algorithm
    tree.parent:=fill(0,nodeCapacity); tree.order:=fill(1,nodeCapacity); tree.count:=0;
    tree.childInverse:=zeros(nodeCapacity,poseDimension,poseDimension);
    tree.parentJacobian:=zeros(nodeCapacity,poseDimension,poseDimension);
    tree.weightFactor:=zeros(nodeCapacity,poseDimension,poseDimension); tree.valid:=true;
    reached:=fill(false,nodeCapacity); reached[1]:=true; layer:=reached; parent:=1; child:=1;
    childJacobian:=zeros(poseDimension,poseDimension); parentJacobian:=childJacobian;
    inverse:=childJacobian; weight:=childJacobian; inverseValid:=false; weightValid:=false;
    for pass in 1:nodeCapacity loop
      // Only vertices reached before this pass can parent new vertices. Keeping
      // the discovery layer fixed avoids selecting an entire long chain before
      // any shortcut edge is examined. Original edge order breaks equal-depth
      // ties; every factor remains in the full undamped information operator.
      layer:=reached;
      for edge in 1:edgeCapacity loop
        if tree.valid and edgeMask[edge]==1.0 then
          parent:=1; child:=1;
          if layer[source[edge]] and not reached[target[edge]] then
            parent:=source[edge]; child:=target[edge];
            childJacobian:=Jj[edge,:,:]; parentJacobian:=Ji[edge,:,:];
          elseif layer[target[edge]] and not reached[source[edge]] then
            parent:=target[edge]; child:=source[edge];
            childJacobian:=Ji[edge,:,:]; parentJacobian:=Jj[edge,:,:];
          end if;
          if child>1 then
            (inverse,inverseValid):=Inverse6(childJacobian);
            (weight,weightValid):=PGCholesky(information[edge,:,:],1e-10);
            tree.valid:=inverseValid and weightValid and tree.count<nodeCapacity-1;
            if tree.valid then
              reached[child]:=true; tree.count:=tree.count+1;
              tree.order[tree.count]:=child; tree.parent[child]:=parent;
              tree.childInverse[child,:,:]:=inverse;
              tree.parentJacobian[child,:,:]:=parentJacobian;
              tree.weightFactor[child,:,:]:=weight;
            end if;
          end if;
        end if;
      end for;
    end for;
    for node in 1:nodeCapacity loop tree.valid:=tree.valid and (nodeMask[node]==0.0 or reached[node]); end for;
  end BuildTree;

  // T' q=f, reversed discovery order, retaining each edge's original direction.
  function Backward
    input Real f[nodeCapacity,poseDimension]; input Tree tree;
    output Real q[nodeCapacity,poseDimension];
  protected
    Real pending[nodeCapacity,poseDimension]; Integer node; Integer parent;
  algorithm
    q:=zeros(nodeCapacity,poseDimension); pending:=f; pending[1,:]:=zeros(poseDimension);
    node:=1; parent:=1;
    for reverseIndex in 1:nodeCapacity loop
      if reverseIndex<=tree.count then
        node:=tree.order[tree.count+1-reverseIndex]; parent:=tree.parent[node];
        q[node,:]:=transpose(tree.childInverse[node,:,:])*pending[node,:];
        if parent>1 then pending[parent,:]:=pending[parent,:]-transpose(tree.parentJacobian[node,:,:])*q[node,:]; end if;
      end if;
    end for;
  end Backward;

  function TreeSolve
    input Real f[nodeCapacity,poseDimension]; input Tree tree;
    output Real x[nodeCapacity,poseDimension];
  protected
    Real q[nodeCapacity,poseDimension]; Real w[poseDimension]; Integer node;
  algorithm
    q:=Backward(f,tree); x:=zeros(nodeCapacity,poseDimension); w:=zeros(poseDimension); node:=1;
    for index in 1:nodeCapacity loop
      if index<=tree.count then
        node:=tree.order[index]; w:=PGSolveBlock(tree.weightFactor[node,:,:],q[node,:]);
        x[node,:]:=tree.childInverse[node,:,:]*(w-tree.parentJacobian[node,:,:]*x[tree.parent[node],:]);
      end if;
    end for;
  end TreeSolve;

  // Six global rigid-motion directions in the graph's world-p/right-local-angle
  // chart. The root and inactive rows stay zero; column normalization changes
  // the basis, not its span. H is the same full undamped operator used by PCG.
  function BuildCoarse
    input Real position[nodeCapacity,3]; input Real rotation[nodeCapacity,3,3];
    input Real nodeMask[nodeCapacity]; input Real edgeMask[edgeCapacity];
    input Integer source[edgeCapacity]; input Integer target[edgeCapacity];
    input Real Ji[edgeCapacity,poseDimension,poseDimension];
    input Real Jj[edgeCapacity,poseDimension,poseDimension];
    input Real information[edgeCapacity,poseDimension,poseDimension];
    output Coarse coarse;
  protected
    Real skew[3,3]; Real square; Real E[poseDimension,poseDimension]; Real value;
    Boolean factorValid;
  algorithm
    coarse:=EmptyCoarse(); skew:=zeros(3,3); square:=0.0;
    E:=zeros(poseDimension,poseDimension); value:=0.0; factorValid:=false; coarse.valid:=true;
    for node in 1:nodeCapacity loop
      if node>1 and nodeMask[node]==1.0 then
        skew:=PGSkew(position[node,:]-position[1,:]);
        for axis in 1:3 loop
          coarse.basis[node,axis,axis]:=1.0;
          for column in 1:3 loop
            coarse.basis[node,axis,column+3]:=-skew[axis,column];
            coarse.basis[node,axis+3,column+3]:=rotation[node,column,axis];
          end for;
        end for;
      end if;
    end for;
    for column in 1:poseDimension loop
      square:=PGDot(coarse.basis[:,:,column],coarse.basis[:,:,column]);
      coarse.valid:=coarse.valid and square>1e-24 and square<=1e100;
      if coarse.valid then coarse.basis[:,:,column]:=coarse.basis[:,:,column]/sqrt(square); end if;
    end for;
    if coarse.valid then
      for column in 1:poseDimension loop
        coarse.product[:,:,column]:=PGNormalProduct(coarse.basis[:,:,column],nodeMask,edgeMask,
          source,target,Ji,Jj,information,zeros(nodeCapacity,poseDimension),0.0);
      end for;
      for row in 1:poseDimension loop
        for column in 1:poseDimension loop
          value:=0.5*(PGDot(coarse.basis[:,:,row],coarse.product[:,:,column])
            +PGDot(coarse.basis[:,:,column],coarse.product[:,:,row]));
          E[row,column]:=value; coarse.valid:=coarse.valid and abs(value)<=1e100;
        end for;
      end for;
      if coarse.valid then
        (coarse.factor,factorValid):=PGCholesky(E,1e-12); coarse.valid:=factorValid;
      end if;
    end if;
    if not coarse.valid then coarse:=EmptyCoarse(); end if;
  end BuildCoarse;

  // C=ZKZ'+(I-ZKZ'H) TreeInverse (I-HZKZ'), K=(Z'HZ)^-1.
  // HZ is precomputed; no full H application occurs in this helper. The same
  // fixed factor is used on both sides, preserving the symmetric SPD contract.
  function BalancedSolve
    input Real f[nodeCapacity,poseDimension]; input Tree tree; input Coarse coarse;
    output Real x[nodeCapacity,poseDimension];
  protected
    Real coefficients[poseDimension]; Real correction[poseDimension];
    Real pending[nodeCapacity,poseDimension]; Real work[nodeCapacity,poseDimension];
  algorithm
    coefficients:=zeros(poseDimension); correction:=zeros(poseDimension);
    pending:=f; work:=zeros(nodeCapacity,poseDimension); x:=work;
    for column in 1:poseDimension loop coefficients[column]:=PGDot(coarse.basis[:,:,column],f); end for;
    coefficients:=PGSolveBlock(coarse.factor,coefficients);
    for column in 1:poseDimension loop pending:=pending-coarse.product[:,:,column]*coefficients[column]; end for;
    work:=TreeSolve(pending,tree);
    for column in 1:poseDimension loop correction[column]:=PGDot(coarse.product[:,:,column],work); end for;
    correction:=PGSolveBlock(coarse.factor,correction);
    x:=work;
    for column in 1:poseDimension loop x:=x+coarse.basis[:,:,column]*(coefficients[column]-correction[column]); end for;
  end BalancedSolve;

  constant Integer pairDimension = 2*poseDimension;
  constant Integer pairCapacity = div(nodeCapacity,2);
  record Pairs
    Real factor[pairCapacity,pairDimension,pairDimension];
    Boolean active[nodeCapacity];
    Boolean valid;
  end Pairs;
  function EmptyPairs
    output Pairs pairs;
  algorithm
    pairs.factor := zeros(pairCapacity,pairDimension,pairDimension);
    pairs.active := fill(false,nodeCapacity); pairs.valid := false;
  end EmptyPairs;
  function FactorPair
    input Real A[pairDimension,pairDimension];
    output Real L[pairDimension,pairDimension]; output Boolean valid;
  protected Real scale; Real value;
  algorithm
    L := zeros(pairDimension,pairDimension); scale := 0.0; value := 0.0; valid := true;
    for i in 1:pairDimension loop scale := max(scale,abs(A[i,i])); end for;
    valid := scale > 1e-12 and scale <= 1e18;
    for i in 1:pairDimension loop
      for j in 1:pairDimension loop
        valid := valid and abs(A[i,j]) <= 1e18 and abs(A[i,j]-A[j,i]) <= 1e-10*max(1.0,scale);
        value := A[i,j];
        for k in 1:pairDimension loop
          if j <= i and k < j then value := value-L[i,k]*L[j,k]; end if;
        end for;
        if j <= i then
          if i == j then
            valid := valid and value > 1e-12*scale and value <= 1e18;
            L[i,j] := sqrt(if value > 1e-12*scale and value <= 1e18 then value else 1.0);
          else L[i,j] := value/L[j,j]; end if;
        end if;
      end for;
    end for;
    if not valid then L := zeros(pairDimension,pairDimension); end if;
  end FactorPair;
  // Principal full-H blocks on free rows (2,3),(4,5),...,(128,padding).
  // Every active factor contributes diagonal terms; within-pair factors also
  // contribute both cross blocks. This changes only the numerical preconditioner.
  function BuildPairs
    input Real nodeMask[nodeCapacity]; input Real edgeMask[edgeCapacity];
    input Integer source[edgeCapacity]; input Integer target[edgeCapacity];
    input Real Ji[edgeCapacity,poseDimension,poseDimension];
    input Real Jj[edgeCapacity,poseDimension,poseDimension];
    input Real information[edgeCapacity,poseDimension,poseDimension];
    output Pairs pairs;
  protected
    Real blocks[pairCapacity,pairDimension,pairDimension]; Real normalBlock[poseDimension,poseDimension];
    Real left[poseDimension,poseDimension]; Real right[poseDimension,poseDimension];
    Real L[pairDimension,pairDimension]; Real paddingScale; Boolean factorValid;
    Integer a; Integer b; Integer pa; Integer pb; Integer oa; Integer ob; Integer node;
  algorithm
    pairs := EmptyPairs(); pairs.valid := true;
    blocks := zeros(pairCapacity,pairDimension,pairDimension); normalBlock := zeros(poseDimension,poseDimension);
    left := normalBlock; right := normalBlock; L := zeros(pairDimension,pairDimension); factorValid := false;
    paddingScale := 1.0; a := 1; b := 1; pa := 1; pb := 1; oa := 0; ob := 0; node := 1;
    for i in 1:nodeCapacity loop
      pairs.valid := pairs.valid and (nodeMask[i] == 0.0 or nodeMask[i] == 1.0);
      pairs.active[i] := i > 1 and nodeMask[i] == 1.0;
    end for;
    for edge in 1:edgeCapacity loop
      pairs.valid := pairs.valid and (edgeMask[edge] == 0.0 or edgeMask[edge] == 1.0);
      if pairs.valid and edgeMask[edge] == 1.0 then
        a := source[edge]; b := target[edge];
        pairs.valid := a >= 1 and a <= nodeCapacity and b >= 1 and b <= nodeCapacity and a <> b;
        if pairs.valid then pairs.valid := nodeMask[a] == 1.0 and nodeMask[b] == 1.0; end if;
        if pairs.valid then
          for sideA in 1:2 loop
            for sideB in 1:2 loop
              a := if sideA == 1 then source[edge] else target[edge];
              b := if sideB == 1 then source[edge] else target[edge];
              if a > 1 and b > 1 then
                pa := div(a-2,2)+1; pb := div(b-2,2)+1;
                if pa == pb then
                  oa := mod(a-2,2)*poseDimension; ob := mod(b-2,2)*poseDimension;
                  left := if sideA == 1 then Ji[edge,:,:] else Jj[edge,:,:];
                  right := if sideB == 1 then Ji[edge,:,:] else Jj[edge,:,:];
                  normalBlock := transpose(left)*information[edge,:,:]*right;
                  for i in 1:poseDimension loop for j in 1:poseDimension loop
                    blocks[pa,oa+i,ob+j] := blocks[pa,oa+i,ob+j]+normalBlock[i,j];
                  end for; end for;
                end if;
              end if;
            end for;
          end for;
        end if;
      end if;
    end for;
    if pairs.valid then
      for pair in 1:pairCapacity loop
        paddingScale := 0.0;
        for row in 1:pairDimension loop
          node := 2+2*(pair-1)+div(row-1,poseDimension);
          if node <= nodeCapacity then
            if pairs.active[node] then paddingScale := max(paddingScale,abs(blocks[pair,row,row])); end if;
          end if;
        end for;
        if paddingScale == 0.0 then paddingScale := 1.0; end if;
        for row in 1:pairDimension loop
          node := 2+2*(pair-1)+div(row-1,poseDimension);
          // Inactive and out-of-range padding never read numerical input cells.
          if node > nodeCapacity then blocks[pair,row,row] := paddingScale;
          elseif not pairs.active[node] then blocks[pair,row,row] := paddingScale; end if;
        end for;
        (L,factorValid) := FactorPair(blocks[pair,:,:]); pairs.valid := pairs.valid and factorValid;
        pairs.factor[pair,:,:] := L;
      end for;
    end if;
    if not pairs.valid then pairs := EmptyPairs(); end if;
  end BuildPairs;
  function PairSolve
    input Real f[nodeCapacity,poseDimension]; input Pairs pairs;
    output Real x[nodeCapacity,poseDimension];
  protected Real z[pairDimension]; Real w[pairDimension]; Real value; Integer node; Integer axis; Integer row;
  algorithm
    x := zeros(nodeCapacity,poseDimension); z := zeros(pairDimension); w := z;
    value := 0.0; node := 1; axis := 1; row := 1;
    if pairs.valid then
      for pair in 1:pairCapacity loop
        z := zeros(pairDimension); w := z;
        for i in 1:pairDimension loop
          node := 2+2*(pair-1)+div(i-1,poseDimension); axis := mod(i-1,poseDimension)+1;
          value := 0.0;
          if node <= nodeCapacity then if pairs.active[node] then value := f[node,axis]; end if; end if;
          for k in 1:pairDimension loop if k < i then value := value-pairs.factor[pair,i,k]*z[k]; end if; end for;
          z[i] := value/pairs.factor[pair,i,i];
        end for;
        for i in 1:pairDimension loop
          row := pairDimension+1-i; value := z[row];
          for k in 1:pairDimension loop if k > row then value := value-pairs.factor[pair,k,row]*w[k]; end if; end for;
          w[row] := value/pairs.factor[pair,row,row];
        end for;
        for i in 1:pairDimension loop
          node := 2+2*(pair-1)+div(i-1,poseDimension); axis := mod(i-1,poseDimension)+1;
          if node <= nodeCapacity then if pairs.active[node] then x[node,axis] := w[i]; end if; end if;
        end for;
      end for;
    end if;
  end PairSolve;
  function BalancedPairSolve
    input Real f[nodeCapacity,poseDimension]; input Pairs pairs; input Coarse coarse;
    output Real x[nodeCapacity,poseDimension];
  protected Real coefficients[poseDimension]; Real correction[poseDimension];
    Real pending[nodeCapacity,poseDimension]; Real work[nodeCapacity,poseDimension];
  algorithm
    coefficients := zeros(poseDimension); correction := zeros(poseDimension);
    pending := f; work := zeros(nodeCapacity,poseDimension); x := work;
    for column in 1:poseDimension loop coefficients[column] := PGDot(coarse.basis[:,:,column],f); end for;
    coefficients := PGSolveBlock(coarse.factor,coefficients);
    for column in 1:poseDimension loop pending := pending-coarse.product[:,:,column]*coefficients[column]; end for;
    work := PairSolve(pending,pairs);
    for column in 1:poseDimension loop correction[column] := PGDot(coarse.product[:,:,column],work); end for;
    correction := PGSolveBlock(coarse.factor,correction);
    x := work;
    for column in 1:poseDimension loop x := x+coarse.basis[:,:,column]*(coefficients[column]-correction[column]); end for;
  end BalancedPairSolve;

  function Select
    input Real position[nodeCapacity,3]; input Real rotation[nodeCapacity,3,3];
    input Real nodeMask[nodeCapacity]; input Real edgeMask[edgeCapacity];
    input Real fromNode[edgeCapacity]; input Real toNode[edgeCapacity];
    input Real translation[edgeCapacity,3]; input Real measuredRotation[edgeCapacity,3,3];
    input Real information[edgeCapacity,poseDimension,poseDimension] "Already scaled by PrepareProblem; no second inflation";
    input Integer currentNode; input Integer referenceNode;
    input Integer maximumPCG=48; input Real tolerance=1e-10; input Boolean requested=true;
    output Result result;
  protected
    Integer source[edgeCapacity]; Integer target[edgeCapacity]; Integer selected[selectedDimension];
    Real Ji[edgeCapacity,poseDimension,poseDimension]; Real Jj[edgeCapacity,poseDimension,poseDimension];
    Real B[nodeCapacity,poseDimension,selectedDimension]; Real X[nodeCapacity,poseDimension,selectedDimension];
    Real HX[nodeCapacity,poseDimension,selectedDimension]; Real residual[nodeCapacity,poseDimension,selectedDimension];
    Real whitened[nodeCapacity,poseDimension,selectedDimension];
    Real r[nodeCapacity,poseDimension]; Real z[nodeCapacity,poseDimension];
    Real direction[nodeCapacity,poseDimension]; Real product[nodeCapacity,poseDimension];
    Real q[nodeCapacity,poseDimension]; Real edgeResidual[poseDimension];
    Real edgeJi[poseDimension,poseDimension]; Real edgeJj[poseDimension,poseDimension];
    Real rho; Real nextRho; Real curvature; Real alpha; Real beta; Real initialNorm;
    Real norm; Real value; Real graphStatus; Real activeNodes; Real activeEdges;
    Boolean valid; Boolean chartValid; Boolean running; Boolean coarseEnabled;
    Tree tree; Coarse coarse; Pairs pairs;
  algorithm
    result:=Empty(); source:=fill(1,edgeCapacity); target:=source; selected:=fill(1,selectedDimension);
    Ji:=zeros(edgeCapacity,poseDimension,poseDimension); Jj:=Ji;
    B:=zeros(nodeCapacity,poseDimension,selectedDimension); X:=B; HX:=B; residual:=B; whitened:=B;
    r:=zeros(nodeCapacity,poseDimension); z:=r; direction:=r; product:=r; q:=r;
    edgeJi:=zeros(poseDimension,poseDimension); edgeJj:=edgeJi; edgeResidual:=zeros(poseDimension);
    rho:=0.0; nextRho:=0.0; curvature:=0.0; alpha:=0.0; beta:=0.0; initialNorm:=0.0;
    norm:=0.0; value:=0.0; graphStatus:=0.0; activeNodes:=0.0; activeEdges:=0.0;
    valid:=false; chartValid:=false; running:=false; coarseEnabled:=false; coarse:=EmptyCoarse(); pairs:=EmptyPairs();
    if requested then
      result.status:=-1;
      valid:=currentNode>=1 and currentNode<=nodeCapacity and referenceNode>=1 and referenceNode<=nodeCapacity
        and maximumPCG>=0 and maximumPCG<=maximumIterations and tolerance>=1e-14 and tolerance<=1e-2;
      if valid then valid:=nodeMask[currentNode]==1.0 and nodeMask[referenceNode]==1.0; end if;
      if valid then
        (source,target,graphStatus,activeNodes,activeEdges):=PGValidateGraph(position,rotation,nodeMask,
          edgeMask,fromNode,toNode,translation,measuredRotation,information);
        result.status:=-2; valid:=graphStatus==1.0;
      end if;
      if valid then
        result.status:=-4;
        for edge in 1:edgeCapacity loop
          if edgeMask[edge]==1.0 then
            (edgeResidual,edgeJi,edgeJj,chartValid):=PGEdge(position[source[edge],:],rotation[source[edge],:,:],
              position[target[edge],:],rotation[target[edge],:,:],translation[edge,:],measuredRotation[edge,:,:]);
            Ji[edge,:,:]:=edgeJi; Jj[edge,:,:]:=edgeJj;
            valid:=valid and chartValid;
            for row in 1:poseDimension loop
              for column in 1:poseDimension loop
                valid:=valid and information[edge,row,column]==information[edge,column,row]
                  and abs(Ji[edge,row,column])<=1e18 and abs(Jj[edge,row,column])<=1e18;
              end for;
            end for;
          end if;
        end for;
      end if;
      if valid then
        result.status:=-3; tree:=BuildTree(nodeMask,edgeMask,source,target,Ji,Jj,information); valid:=tree.valid;
      end if;
      if valid then
        // Preserve X=0/tree-only majorant when no PCG step is requested. With
        // only the root active there is no free subspace or coarse rank to test.
        coarseEnabled:=maximumPCG>0 and activeNodes>1.0;
        if coarseEnabled then
          coarse:=BuildCoarse(position,rotation,nodeMask,edgeMask,source,target,Ji,Jj,information); valid:=coarse.valid;
          if valid then pairs:=BuildPairs(nodeMask,edgeMask,source,target,Ji,Jj,information); valid:=pairs.valid; end if;
        end if;
      end if;
      if valid then
        result.status:=-5;
        for column in 1:selectedDimension loop
          selected[column]:=if column<=poseDimension then currentNode else referenceNode;
          if selected[column]>1 then B[selected[column],mod(column-1,poseDimension)+1,column]:=1.0; end if;
          r:=B[:,:,column]; z:=if coarseEnabled then BalancedPairSolve(r,pairs,coarse) else TreeSolve(r,tree); direction:=z;
          rho:=PGDot(r,z); initialNorm:=PGDot(r,r);
          valid:=valid and rho>=0.0 and rho<=1e100;
          running:=valid and initialNorm>0.0;
          for iteration in 1:maximumIterations loop
            if running and iteration<=maximumPCG then
              product:=PGNormalProduct(direction,nodeMask,edgeMask,source,target,Ji,Jj,information,zeros(nodeCapacity,poseDimension),0.0);
              curvature:=PGDot(direction,product); valid:=curvature>0.0 and curvature<=1e100 and rho>0.0;
              if valid then
                alpha:=rho/curvature; X[:,:,column]:=X[:,:,column]+alpha*direction;
                r:=r-alpha*product; norm:=PGDot(r,r); result.iterations[column]:=iteration;
                valid:=norm>=0.0 and norm<=1e100;
                running:=valid and norm>tolerance*tolerance*initialNorm;
                if running then
                  z:=if coarseEnabled then BalancedPairSolve(r,pairs,coarse) else TreeSolve(r,tree); nextRho:=PGDot(r,z); valid:=nextRho>0.0 and nextRho<=1e100;
                  if valid then beta:=nextRho/rho; direction:=z+beta*direction; rho:=nextRho; else running:=false; end if;
                end if;
              else running:=false;
              end if;
            end if;
          end for;
          HX[:,:,column]:=PGNormalProduct(X[:,:,column],nodeMask,edgeMask,source,target,Ji,Jj,information,zeros(nodeCapacity,poseDimension),0.0);
          residual[:,:,column]:=B[:,:,column]-HX[:,:,column];
          norm:=PGDot(residual[:,:,column],residual[:,:,column]);
          result.residualNorm[column]:=sqrt(max(0.0,norm));
          result.converged[column]:=norm<=tolerance*tolerance*initialNorm;
          q:=Backward(residual[:,:,column],tree);
          for node in 1:nodeCapacity loop
            if node>1 and nodeMask[node]==1.0 then
              // Only L_W^-1 q is needed for the full residual Gram upper bound.
              for axis in 1:poseDimension loop
                value:=q[node,axis];
                for k in 1:poseDimension loop
                  if k<axis then value:=value-tree.weightFactor[node,axis,k]*whitened[node,k,column]; end if;
                end for;
                whitened[node,axis,column]:=value/tree.weightFactor[node,axis,axis];
              end for;
            end if;
          end for;
        end for;
        for row in 1:selectedDimension loop
          for column in 1:selectedDimension loop
            result.lower[row,column]:=PGDot(B[:,:,row],X[:,:,column])+PGDot(X[:,:,row],B[:,:,column])
              -0.5*(PGDot(X[:,:,row],HX[:,:,column])+PGDot(HX[:,:,row],X[:,:,column]));
            result.residualUpper[row,column]:=PGDot(whitened[:,:,row],whitened[:,:,column]);
            result.upper[row,column]:=result.lower[row,column]+result.residualUpper[row,column];
            valid:=valid and abs(result.lower[row,column])<=1e100
              and abs(result.residualUpper[row,column])<=1e100 and abs(result.upper[row,column])<=1e100;
          end for;
        end for;
        if valid then
          result.accepted:=true; result.status:=1; result.treeEdges:=tree.count; result.activeNodes:=integer(activeNodes);
        end if;
      end if;
      if not valid then
        graphStatus:=result.status; result:=Empty(); result.status:=integer(graphStatus);
      end if;
    end if;
  end Select;
end ModelicaPoseGraphCovariance;
