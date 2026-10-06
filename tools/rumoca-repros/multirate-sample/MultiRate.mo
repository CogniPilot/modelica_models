within;
model MultiRate
  "A fast sampled consumer reads a slower sampled producer's held value"
  discrete Real source[2](each start = 0.0, each fixed = true);
  discrete Real held[2](each start = 0.0, each fixed = true);
algorithm
  when sample(0.0, 0.00125) then
    held := source;
  end when;
algorithm
  when sample(0.0, 0.01) then
    source := pre(source) + {1.0, 2.0};
  end when;
end MultiRate;
