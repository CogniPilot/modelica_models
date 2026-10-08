Rumoca 0.10.2 zero-row function padding
======================================

  rumoca sim EmptyPadding.mo --model EmptyPadding --t-end 0.02

fails with EL005: invalid Solve IR contract: pure-call argument or slot
interface is invalid. The padded block is zeros(0, 3) for this wide input.

  rumoca sim MaxWorkspace.mo --model MaxWorkspace --t-end 0.02

passes. This control retains the same max-dependent workspace declaration,
so max in the declaration alone does not explain the failure.

LinearAlgebra.covarianceRoot previously used the failing padding expression.
It now operates on the transposed input directly and returns missing triangular
columns as zeros for narrow inputs. Tests.CovarianceRootTests covers wide,
narrow rank-deficient, and zero inputs in the regular CI simulation list.
The dated 2026-10-08 review records the reproduction and numerical validation.
