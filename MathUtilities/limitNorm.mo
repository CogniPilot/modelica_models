within MathUtilities;
function limitNorm "Scale a vector so its Euclidean norm respects a bound"
  input Real value[:];
  input Real maximumNorm(min = 0.0);
  output Real result[size(value, 1)];
protected
  Real magnitude;
algorithm
  magnitude := sqrt(value * value);
  result := if magnitude > maximumNorm then
      (maximumNorm / magnitude) * value
    else
      value;
  annotation(
    Inline = true,
    Documentation(info = "<html>
      <p>Limits magnitude without rotating the vector, so a diagonal command
      keeps its direction instead of being squared off by an element-wise
      clip.</p>
    </html>"));
end limitNorm;
