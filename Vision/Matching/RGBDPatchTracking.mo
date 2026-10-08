within Vision.Matching;
// Staged photometric primitive: no production caller or source-manifest entry.
// Forward-additive translation only; no depth, correspondence independence,
// full-image forward/backward check, or uncertainty certificate is implied.
package RGBDPatchTracking
  constant Integer patchWidth = 7;
  constant Integer patchSize = patchWidth*patchWidth;
  constant Integer pixelDimension = 2;

  record Settings
    Integer maximumIterations = 20;
    Integer maximumBacktracks = 8;
    Real searchRadius = 4.0 "Maximum distance from supplied seed, in pixels";
    Real maximumStep = 1.0;
    Real convergenceTolerance = 0.001;
    Real minimumContrast = 0.001 "Raw grayscale standard deviation; admitted range [1e-12,1]";
    Real minimumEigenvalue = 1e-6;
    Real minimumEigenvalueRatio = 0.001;
    Real maximumSsd = 0.04 "Sum of squared unit-normalized descriptor residuals";
    Integer interpolationMethod = 0 "0 bilinear; 1 C1 Catmull-Rom with complete4x4 raw support";
  end Settings;

  record Diagnostics
    Integer stopDetail = 0 "0 disabled; 1 accepted; 2 config; 3 reference; 4 sample; 5 Hessian; 6 step; 7 backtracking; 8 search; 9 iterations; 10 SSD";
    Boolean evaluated = false;
    Real lastEvaluatedPixel[pixelDimension] = zeros(pixelDimension);
    Boolean acceptedTrial = false;
    Real lastAcceptedTrialPixel[pixelDimension] = zeros(pixelDimension);
    Real initialSsd = 0;
    Boolean stepValid = false;
    Real lastRawStep[pixelDimension] = zeros(pixelDimension);
    Real lastRawStepNorm = 0;
    Integer lastBacktrack = 0;
    Real lastTrialScale = 0;
    Real lastTrialPixel[pixelDimension] = zeros(pixelDimension);
    Boolean lastTrialInSearch = false;
    Boolean lastTrialValid = false;
    Real lastTrialSsd = 0;
    Integer windowRejects = 0;
    Integer sampleRejects = 0;
    Integer nonDecreaseRejects = 0;
    Real lastEnergy = 0;
    Real maximumEigenvalue = 0;
    Real lastGradient[pixelDimension] = zeros(pixelDimension);
  end Diagnostics;

  function SmoothBinomial
    input Real gray[:,:];
    output Real filtered[size(gray,1),size(gray,2)] "-1 where complete valid support is unavailable";
    output Boolean valid[size(gray,1),size(gray,2)];
  protected
    constant Integer tapCount = 5;
    constant Real weights[tapCount] = {1,4,6,4,1};
    constant Real weightSum = 16;
    Real horizontal[size(gray,1),size(gray,2)]; Real values[tapCount]; Boolean supportValid;
  algorithm
    filtered := fill(-1.0,size(gray,1),size(gray,2));
    valid := fill(false,size(gray,1),size(gray,2));
    horizontal := fill(-1.0,size(gray,1),size(gray,2));
    // All taps have positive weight: invalid samples are never renormalized away.
    // The outer two-pixel border has no complete support; no padding is invented.
    if size(gray,1) < tapCount or size(gray,2) < tapCount then return; end if;
    for row in 1:size(gray,1) loop
      for column in 3:size(gray,2)-2 loop
        supportValid := true;
        for tap in 1:tapCount loop
          values[tap] := gray[row,column+tap-3];
          supportValid := supportValid and values[tap] >= 0 and values[tap] <= 1;
        end for;
        if supportValid then horizontal[row,column] := weights*values/weightSum; end if;
      end for;
    end for;
    for row in 3:size(gray,1)-2 loop
      for column in 3:size(gray,2)-2 loop
        supportValid := true;
        for tap in 1:tapCount loop
          values[tap] := horizontal[row+tap-3,column];
          supportValid := supportValid and values[tap] >= 0 and values[tap] <= 1;
        end for;
        if supportValid then
          filtered[row,column] := weights*values/weightSum; valid[row,column] := true;
        end if;
      end for;
    end for;
  end SmoothBinomial;

  function CatmullRomWeights
    input Real fraction "Admitted sample fraction in [0,1)";
    output Real weights[4];
    output Real derivative[4];
  protected
    Real squared; Real cubed;
  algorithm
    squared := fraction*fraction; cubed := squared*fraction;
    weights := {-0.5*fraction+squared-0.5*cubed,1-2.5*squared+1.5*cubed,
      0.5*fraction+2*squared-1.5*cubed,-0.5*squared+0.5*cubed};
    derivative := {-0.5+2*fraction-1.5*squared,-5*fraction+4.5*squared,
      0.5+4*fraction-4.5*squared,-fraction+1.5*squared};
  end CatmullRomWeights;

  function Sample
    input Real gray[:,:] "Raw samples use the existing [0,1] gray convention";
    input Real pixel[pixelDimension] "Zero-based RGB coordinates";
    input Real minimumContrast = 0.001;
    input Integer interpolationMethod = 0 "0 bilinear; 1 C1 Catmull-Rom";
    output Boolean valid;
    output Real descriptor[patchSize];
    output Real jacobian[patchSize,pixelDimension];
    output Real energy;
  protected
    Real samples[patchSize]; Real gradient[patchSize,pixelDimension];
    Real centered[patchSize]; Real meanSample; Real meanGradient[pixelDimension];
    Real projection[pixelDimension]; Real x; Real y; Real fx; Real fy;
    Real a; Real b; Real c; Real d; Real denominator;
    Real weightsX[4]; Real weightsY[4]; Real derivativeX[4]; Real derivativeY[4];
    Real support[4,4];
    Integer x0; Integer y0; Integer slot;
  algorithm
    valid := false; descriptor := zeros(patchSize);
    jacobian := zeros(patchSize,pixelDimension); energy := 0.0;
    // The upper bound leaves a complete derivative cell even at integer pixels.
    // Validate before floor/Integer conversion or any indexed read.
    if interpolationMethod == 0 then
      if not (size(gray,1) >= patchWidth+1 and size(gray,2) >= patchWidth+1
        and pixel[1] >= 3 and pixel[1] < size(gray,2)-4
        and pixel[2] >= 3 and pixel[2] < size(gray,1)-4
        and minimumContrast >= 1e-12 and minimumContrast <= 1) then return; end if;
    elseif interpolationMethod == 1 then
      // A cubic sample needs floor(x)-1..floor(x)+2 on each axis, including
      // zero-weight taps at knots. No padded or clamped boundary is invented.
      if not (size(gray,1) >= patchWidth+3 and size(gray,2) >= patchWidth+3
        and pixel[1] >= 4 and pixel[1] < size(gray,2)-5
        and pixel[2] >= 4 and pixel[2] < size(gray,1)-5
        and minimumContrast >= 1e-12 and minimumContrast <= 1) then return; end if;
    else return; end if;
    for row in 1:patchWidth loop
      for column in 1:patchWidth loop
        slot := (row-1)*patchWidth+column;
        x := pixel[1]+column-4; y := pixel[2]+row-4;
        x0 := integer(floor(x)); y0 := integer(floor(y));
        fx := x-x0; fy := y-y0;
        if interpolationMethod == 0 then
          a := gray[y0+1,x0+1]; b := gray[y0+1,x0+2];
          c := gray[y0+2,x0+1]; d := gray[y0+2,x0+2];
          if not (a >= 0 and a <= 1 and b >= 0 and b <= 1
            and c >= 0 and c <= 1 and d >= 0 and d <= 1) then return; end if;
          samples[slot] := (1-fy)*((1-fx)*a+fx*b)+fy*((1-fx)*c+fx*d);
          gradient[slot,1] := (1-fy)*(b-a)+fy*(d-c);
          gradient[slot,2] := (1-fx)*(c-a)+fx*(d-b);
        else
          (weightsX,derivativeX) := CatmullRomWeights(fx);
          (weightsY,derivativeY) := CatmullRomWeights(fy);
          for v in 1:4 loop
            for u in 1:4 loop
              support[v,u] := gray[y0+v-1,x0+u-1];
              if not (support[v,u] >= 0 and support[v,u] <= 1) then return; end if;
            end for;
          end for;
          // Cubic overshoot is retained. The centered-energy admission below
          // bounds the normalized result without clipping interpolated samples.
          samples[slot] := weightsY*support*weightsX;
          gradient[slot,1] := weightsY*support*derivativeX;
          gradient[slot,2] := derivativeY*support*weightsX;
        end if;
      end for;
    end for;
    meanSample := sum(samples)/patchSize;
    centered := samples-fill(meanSample,patchSize); energy := centered*centered;
    if not (energy > 0 and energy >= patchSize*minimumContrast*minimumContrast and energy <= patchSize) then return; end if;
    denominator := sqrt(energy); descriptor := centered/denominator;
    for axis in 1:pixelDimension loop
      meanGradient[axis] := sum(gradient[:,axis])/patchSize;
      projection[axis] := centered*gradient[:,axis]/energy;
      jacobian[:,axis] := (gradient[:,axis]-fill(meanGradient[axis],patchSize)
        -centered*projection[axis])/denominator;
    end for;
    valid := true;
  end Sample;

  function TrackDetailed
    input Real gray[:,:];
    input Real referenceDescriptor[patchSize];
    input Real seed[pixelDimension];
    input Boolean enabled = true;
    input Settings settings = Settings();
    output Boolean accepted;
    output Real pixel[pixelDimension];
    output Integer reason "0 accepted/disabled; 1 config; 2 reference; 3 sample/window; 4 aperture; 5 search; 6 descent/iterations; 7 SSD";
    output Integer iterations;
    output Real ssd;
    output Real minimumEigenvalue;
    output Diagnostics diagnostics;
  protected
    Real descriptor[patchSize]; Real jacobian[patchSize,pixelDimension]; Real energy;
    Real trialDescriptor[patchSize]; Real trialJacobian[patchSize,pixelDimension]; Real trialEnergy;
    Real residual[patchSize]; Real normal[pixelDimension,pixelDimension]; Real rhs[pixelDimension];
    Real step[pixelDimension]; Real trial[pixelDimension]; Real current[pixelDimension];
    Real determinant; Real traceValue; Real discriminant; Real maximumEigenvalue;
    Real normStep; Real scale; Real trialCost; Real referenceEnergy;
    Boolean valid; Boolean trialValid; Boolean moved; Boolean converged;
  algorithm
    accepted := false; pixel := zeros(pixelDimension); reason := 0;
    iterations := 0; ssd := 0; minimumEigenvalue := 0;
    diagnostics := Diagnostics();
    // Disabled padding is opaque, including seed/template poison.
    if not enabled then return; end if;
    reason := 1; diagnostics.stopDetail := 2;
    if not (settings.maximumIterations >= 1 and settings.maximumIterations <= 40
      and (settings.interpolationMethod == 0 or settings.interpolationMethod == 1)
      and settings.maximumBacktracks >= 1 and settings.maximumBacktracks <= 12
      and settings.searchRadius > 0 and settings.searchRadius <= 16
      and settings.maximumStep > 0 and settings.maximumStep <= settings.searchRadius
      and settings.convergenceTolerance > 0 and settings.convergenceTolerance <= settings.maximumStep
      and settings.minimumContrast >= 1e-12 and settings.minimumContrast <= 1
      and settings.minimumEigenvalue > 0 and settings.minimumEigenvalue <= 1
      and settings.minimumEigenvalueRatio > 0 and settings.minimumEigenvalueRatio <= 1
      and settings.maximumSsd > 0 and settings.maximumSsd <= 4) then return; end if;
    reason := 2; diagnostics.stopDetail := 3;
    for slot in 1:patchSize loop
      if not (referenceDescriptor[slot] >= -1 and referenceDescriptor[slot] <= 1) then return; end if;
    end for;
    referenceEnergy := referenceDescriptor*referenceDescriptor;
    if not (abs(referenceEnergy-1) <= 1e-8 and abs(sum(referenceDescriptor)) <= 1e-8) then return; end if;
    current := seed; converged := false;
    for iteration in 1:settings.maximumIterations loop
      iterations := iteration; reason := 3; diagnostics.stopDetail := 4;
      (valid,descriptor,jacobian,energy) := Sample(gray,current,settings.minimumContrast,settings.interpolationMethod);
      if not valid then return; end if;
      diagnostics.evaluated := true; diagnostics.lastEvaluatedPixel := current;
      diagnostics.lastEnergy := energy;
      residual := descriptor-referenceDescriptor; ssd := residual*residual;
      if iteration == 1 then diagnostics.initialSsd := ssd; end if;
      normal := transpose(jacobian)*jacobian; rhs := transpose(jacobian)*residual;
      diagnostics.lastGradient := rhs;
      traceValue := normal[1,1]+normal[2,2];
      discriminant := sqrt(max(0,(normal[1,1]-normal[2,2])^2+4*normal[1,2]^2));
      minimumEigenvalue := (traceValue-discriminant)/2;
      maximumEigenvalue := (traceValue+discriminant)/2; reason := 4;
      diagnostics.maximumEigenvalue := maximumEigenvalue; diagnostics.stopDetail := 5;
      if not (minimumEigenvalue >= settings.minimumEigenvalue
        and minimumEigenvalue >= settings.minimumEigenvalueRatio*maximumEigenvalue) then return; end if;
      determinant := normal[1,1]*normal[2,2]-normal[1,2]*normal[2,1];
      if not (determinant > 0) then return; end if;
      step := {normal[1,2]*rhs[2]-normal[2,2]*rhs[1],
        normal[2,1]*rhs[1]-normal[1,1]*rhs[2]}/determinant;
      normStep := sqrt(step*step);
      if not (normStep >= 0 and normStep <= 1e6) then reason := 6; diagnostics.stopDetail := 6; return; end if;
      diagnostics.stepValid := true; diagnostics.lastRawStep := step; diagnostics.lastRawStepNorm := normStep;
      if normStep <= settings.convergenceTolerance then converged := true; break; end if;
      if normStep > settings.maximumStep then step := step*(settings.maximumStep/normStep); end if;
      scale := 1; moved := false; reason := 6;
      for backtrack in 1:settings.maximumBacktracks loop
        trial := current+scale*step;
        diagnostics.lastBacktrack := backtrack; diagnostics.lastTrialScale := scale;
        diagnostics.lastTrialPixel := trial; diagnostics.lastTrialInSearch := false;
        diagnostics.lastTrialValid := false; diagnostics.lastTrialSsd := 0;
        if (trial-seed)*(trial-seed) <= settings.searchRadius*settings.searchRadius then
          diagnostics.lastTrialInSearch := true;
          (trialValid,trialDescriptor,trialJacobian,trialEnergy) := Sample(gray,trial,settings.minimumContrast,settings.interpolationMethod);
          if trialValid then
            residual := trialDescriptor-referenceDescriptor; trialCost := residual*residual;
            diagnostics.lastTrialValid := true; diagnostics.lastTrialSsd := trialCost;
            if trialCost < ssd then
              diagnostics.acceptedTrial := true; diagnostics.lastAcceptedTrialPixel := trial;
              current := trial; moved := true; break;
            end if;
            diagnostics.nonDecreaseRejects := diagnostics.nonDecreaseRejects+1;
          else
            diagnostics.sampleRejects := diagnostics.sampleRejects+1;
          end if;
        else
          diagnostics.windowRejects := diagnostics.windowRejects+1;
        end if;
        scale := scale/2;
      end for;
      if not moved then
        diagnostics.stopDetail := 7;
        if (current+step-seed)*(current+step-seed) > settings.searchRadius*settings.searchRadius then reason := 5; diagnostics.stopDetail := 8; end if;
        return;
      end if;
    end for;
    reason := 6; diagnostics.stopDetail := 9; if not converged then return; end if;
    reason := 7; diagnostics.stopDetail := 10; if not (ssd <= settings.maximumSsd) then return; end if;
    accepted := true; pixel := current; reason := 0;
    diagnostics.stopDetail := 1;
  end TrackDetailed;

  function Track
    input Real gray[:,:];
    input Real referenceDescriptor[patchSize];
    input Real seed[pixelDimension];
    input Boolean enabled = true;
    input Settings settings = Settings();
    output Boolean accepted;
    output Real pixel[pixelDimension];
    output Integer reason "0 accepted/disabled; 1 config; 2 reference; 3 sample/window; 4 aperture; 5 search; 6 descent/iterations; 7 SSD";
    output Integer iterations;
    output Real ssd;
    output Real minimumEigenvalue;
  protected
    Diagnostics diagnostics;
  algorithm
    (accepted,pixel,reason,iterations,ssd,minimumEigenvalue,diagnostics)
      := TrackDetailed(gray,referenceDescriptor,seed,enabled,settings);
  end Track;

  function CheckReverse
    input Real referenceGray[:,:]; input Real currentGray[:,:];
    input Real referencePixel[pixelDimension]; input Real trackedPixel[pixelDimension];
    input Boolean forwardAccepted;
    input Settings settings = Settings();
    input Real maximumCycleError = 0.25 "Pixels; independent of photometric and registration limits";
    output Boolean accepted;
    output Integer reason "0 accepted/disabled; 1 configuration; 2 current template; 3 reverse tracking; 4 cycle";
    output Real returnedPixel[pixelDimension]; output Real cycleError;
    output Boolean reverseAccepted; output Integer reverseReason; output Integer reverseIterations;
    output Real reverseSsd; output Real reverseEigenvalue;
    output Diagnostics reverseDiagnostics;
  protected
    Real descriptor[patchSize]; Real jacobian[patchSize,pixelDimension]; Real energy;
    Real difference[pixelDimension]; Boolean valid;
  algorithm
    accepted := false; reason := 0; returnedPixel := zeros(pixelDimension); cycleError := 0;
    reverseAccepted := false; reverseReason := 0; reverseIterations := 0;
    reverseSsd := 0; reverseEigenvalue := 0; reverseDiagnostics := Diagnostics();
    // Failed/disabled forward slots are opaque, including image and setting poison.
    if not forwardAccepted then return; end if;
    reason := 1;
    if not (size(referenceGray,1) == size(currentGray,1)
      and size(referenceGray,2) == size(currentGray,2)
      and maximumCycleError > 0 and maximumCycleError <= 16
      and referencePixel[1] >= 0 and referencePixel[1] < size(referenceGray,2)
      and referencePixel[2] >= 0 and referencePixel[2] < size(referenceGray,1)) then return; end if;
    reason := 2;
    (valid,descriptor,jacobian,energy) := Sample(currentGray,trackedPixel,
      settings.minimumContrast,settings.interpolationMethod);
    if not valid then return; end if;
    reason := 3;
    (reverseAccepted,returnedPixel,reverseReason,reverseIterations,reverseSsd,reverseEigenvalue,reverseDiagnostics)
      := TrackDetailed(referenceGray,descriptor,referencePixel,true,settings);
    if not reverseAccepted then return; end if;
    difference := returnedPixel-referencePixel; cycleError := sqrt(difference*difference);
    reason := 4;
    if not (cycleError <= maximumCycleError) then return; end if;
    accepted := true; reason := 0;
  end CheckReverse;

  function TrackSet
    input Real gray[:,:]; input Real referenceDescriptor[:,patchSize];
    input Real seeds[size(referenceDescriptor,1),pixelDimension];
    input Boolean enabled[size(referenceDescriptor,1)];
    input Settings settings = Settings();
    output Boolean accepted[size(referenceDescriptor,1)];
    output Real pixels[size(referenceDescriptor,1),pixelDimension];
    output Integer reasons[size(referenceDescriptor,1)];
    output Integer iterations[size(referenceDescriptor,1)];
    output Real ssd[size(referenceDescriptor,1)];
    output Real minimumEigenvalue[size(referenceDescriptor,1)];
  protected
    Boolean slotAccepted; Real slotPixel[pixelDimension]; Integer slotReason;
    Integer slotIterations; Real slotSsd; Real slotEigenvalue;
  algorithm
    // Slot identity and sparse enabled mask are retained across the entire extent.
    for slot in 1:size(referenceDescriptor,1) loop
      (slotAccepted,slotPixel,slotReason,slotIterations,slotSsd,slotEigenvalue)
        := Track(gray,referenceDescriptor[slot,:],seeds[slot,:],enabled[slot],settings);
      accepted[slot] := slotAccepted; pixels[slot,:] := slotPixel; reasons[slot] := slotReason;
      iterations[slot] := slotIterations; ssd[slot] := slotSsd; minimumEigenvalue[slot] := slotEigenvalue;
    end for;
  end TrackSet;
end RGBDPatchTracking;
