within;
package PackageRecordConstant
  "A member of a package-level record constant cannot be referenced"

  record Fundamental
    Real R_s;
    Real h_off;
  end Fundamental;

  package Basic
    constant Fundamental Modified(R_s = 287.0, h_off = 1589557.6);
    constant Fundamental Constructed = Fundamental(R_s = 287.0, h_off = 1.0);
    constant Real Scalar = 287.0;
  end Basic;

  model ReadsScalarConstant "Compiles: a scalar package constant resolves"
    Real y;
  equation
    y = Basic.Scalar;
  end ReadsScalarConstant;

  model ReadsModifiedRecord "Refused: ED008 on `Basic.Modified.h_off`"
    Real y;
  equation
    y = Basic.Modified.h_off;
  end ReadsModifiedRecord;

  model ReadsConstructedRecord "Refused: ED008 on `Basic.Constructed.h_off`"
    Real y;
  equation
    y = Basic.Constructed.h_off;
  end ReadsConstructedRecord;

  annotation(Documentation(info="<html>
    <p>Every member access on a package-level constant of record type is
    reported as an unresolved Flat reference, however the value is supplied:
    by a modification on the declaration, or by a record constructor binding.
    A scalar constant in the same package resolves.</p>
  </html>"));
end PackageRecordConstant;
