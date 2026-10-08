within SLAM.Localization;
// Source-owned transport from one accepted localization image to the catalog
// measurement API. This is not another visual update or an independence claim.
// Dependencies: RGBDKeyframes, RGBDRegistrationUncertainty, SLAMExactRealEqual.
package RGBDLocalizationFrame
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDUncertaintyInverse6 = SLAM.Localization.RGBDUncertaintyInverse6;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import SLAMExactRealEqual = SLAM.Inertial.SLAMExactRealEqual;

  // Current errors are five3-vectors: world dp,dv, right-local dtheta,dba,dbg.
  constant Integer currentDimension = 5*RGBDKeyframes.dimension;
  constant Integer poseIndices[RGBDKeyframes.poseDimension] = {1,2,3,7,8,9};

  function Build
    input Integer generation "Checked outer catalog/reset generation";
    input Integer id "Checked catalog nextId proposal, never a ring slot";
    input Integer imageEpoch "Acquisition identity, independent of imageTime";
    input Real coreEpoch "Must exactly equal the Integer acquisition identity";
    input Real imageTime;
    input Real currentCount "Selected domain extent, not enabled or match count";
    input Real currentDescriptor[RGBDKeyframes.featureCapacity,RGBDKeyframes.descriptorSize];
    input Real currentPoint[RGBDKeyframes.featureCapacity,RGBDKeyframes.dimension];
    input Real currentEnabled[RGBDKeyframes.featureCapacity];
    input Real pixels[RGBDKeyframes.featureCapacity,2] "Zero-based RGB pixels";
    input Integer rgbSize[2] "Actual height,width";
    input Integer depthSize[2];
    input Real rgbCalibration[4];
    input Real depthCalibration[4];
    input Real opticalToBody[RGBDKeyframes.dimension,RGBDKeyframes.dimension];
    input Real cameraOriginBody[RGBDKeyframes.dimension];
    input Real disparityNoise;
    input Real noiseReferenceFx;
    input Real baseline;
    input Real bodyPosition[RGBDKeyframes.dimension];
    input Real bodyRotation[RGBDKeyframes.dimension,RGBDKeyframes.dimension];
    input Real currentCovariance[currentDimension,currentDimension]
      "Already accepted producer covariance: world dp,dv; right-local angle; body biases";
    input Integer vocabularyVersion;
    input Real poseAccepted;
    input Boolean requested;
    output RGBDKeyframes.Frame frame;
    output Boolean accepted;
    output Integer rejectionReason "0 accepted; 1 idle; 2 metadata; 3 domain/mask; 4 calibration; 5 pose; 6 pose covariance; 7 feature payload";
  protected
    RGBDKeyframes.Frame candidate;
    Boolean valid;
    Boolean covarianceValid;
    Real inverseCovariance[RGBDKeyframes.poseDimension,RGBDKeyframes.poseDimension];
    Real scaledPivot;
    Real descriptorNorm;
    Real descriptorMean;
  algorithm
    frame := RGBDKeyframes.EmptyFrame();
    accepted := false;
    rejectionReason := 1;
    // The whole idle payload is opaque. No conversions or helper calls below
    // are demanded until requested and their preceding domains are certified.
    if requested then
      rejectionReason := 2;
      valid := generation >= 1 and generation <= RGBDKeyframes.identifierLimit
        and id >= 1 and id < RGBDKeyframes.identifierLimit
        and imageEpoch >= 0 and imageEpoch <= RGBDKeyframes.identifierLimit
        and SLAMExactRealEqual(coreEpoch,imageEpoch)
        and imageTime >= 0.0 and imageTime <= 1e9
        and vocabularyVersion >= 1 and vocabularyVersion <= RGBDKeyframes.identifierLimit;
      if valid then
        rejectionReason := 3;
        valid := currentCount >= 0.0 and currentCount <= RGBDKeyframes.featureCapacity
          and SLAMExactRealEqual(currentCount,floor(currentCount));
        if valid then
          // Mask validation never reads descriptor/point/pixel payload.
          for feature in 1:RGBDKeyframes.featureCapacity loop
            valid := valid and (SLAMExactRealEqual(currentEnabled[feature],0.0)
              or SLAMExactRealEqual(currentEnabled[feature],1.0));
            if SLAMExactRealEqual(currentEnabled[feature],1.0) then
              valid := valid and feature <= currentCount;
            end if;
          end for;
        end if;
        if valid then
          rejectionReason := 4;
          valid := RGBDKeyframes.ValidCalibration(rgbCalibration,rgbSize)
            and RGBDKeyframes.ValidCalibration(depthCalibration,depthSize)
            and disparityNoise > 0.0 and disparityNoise <= 1.0
            and noiseReferenceFx >= 1e-6 and noiseReferenceFx <= 1e6
            and baseline >= 1e-6 and baseline <= 1.0
            and RGBDUncertaintyProper(opticalToBody);
          for axis in 1:RGBDKeyframes.dimension loop
            valid := valid and abs(cameraOriginBody[axis]) <= 10.0;
          end for;
        end if;
        if valid then
          rejectionReason := 5;
          valid := SLAMExactRealEqual(poseAccepted,1.0)
            and RGBDUncertaintyProper(bodyRotation);
          for axis in 1:RGBDKeyframes.dimension loop
            valid := valid and abs(bodyPosition[axis]) <= 1e6;
          end for;
        end if;
        if valid then
          candidate := RGBDKeyframes.EmptyFrame();
          // Preserve the complete position/right-local-angle marginal, both
          // cross blocks included. This does not certify the full21-state prior.
          for row in 1:RGBDKeyframes.poseDimension loop
            for column in 1:RGBDKeyframes.poseDimension loop
              candidate.poseCovariance[row,column] :=
                currentCovariance[poseIndices[row],poseIndices[column]];
            end for;
          end for;
          rejectionReason := 6;
          (inverseCovariance,covarianceValid,scaledPivot) :=
            RGBDUncertaintyInverse6(candidate.poseCovariance,1e-10);
          valid := covarianceValid;
          if valid then
            rejectionReason := 7;
            for feature in 1:RGBDKeyframes.featureCapacity loop
              if SLAMExactRealEqual(currentEnabled[feature],1.0) then
                valid := valid and currentPoint[feature,3] > 0.0;
                for axis in 1:RGBDKeyframes.dimension loop
                  valid := valid and abs(currentPoint[feature,axis]) <= 1e6;
                end for;
                for coordinate in 1:2 loop
                  valid := valid and pixels[feature,coordinate] >= 0.0
                    and pixels[feature,coordinate] < rgbSize[3-coordinate]
                    and SLAMExactRealEqual(pixels[feature,coordinate],floor(pixels[feature,coordinate]));
                end for;
                descriptorNorm := 0.0;
                descriptorMean := 0.0;
                for sample in 1:RGBDKeyframes.descriptorSize loop
                  valid := valid and abs(currentDescriptor[feature,sample]) <= 1.000001;
                  descriptorNorm := descriptorNorm+currentDescriptor[feature,sample]^2;
                  descriptorMean := descriptorMean+currentDescriptor[feature,sample];
                end for;
                valid := valid and abs(descriptorNorm-1.0) <= 1e-6
                  and abs(descriptorMean) <= 1e-6;
              end if;
            end for;
            if valid then
              // Conversions follow complete domain validation. Disabled slots
              // retain canonical empty payload; even a poisoned slot is unread.
              candidate.generation := generation; candidate.id := id;
              candidate.epoch := imageEpoch; candidate.imageTime := imageTime;
              candidate.count := integer(currentCount);
              candidate.rgbSize := rgbSize; candidate.depthSize := depthSize;
              candidate.rgbCalibration := rgbCalibration;
              candidate.depthCalibration := depthCalibration;
              candidate.opticalToBody := opticalToBody;
              candidate.cameraOriginBody := cameraOriginBody;
              candidate.disparityNoise := disparityNoise;
              candidate.noiseReferenceFx := noiseReferenceFx; candidate.baseline := baseline;
              candidate.bodyPosition := bodyPosition; candidate.bodyRotation := bodyRotation;
              candidate.vocabularyVersion := vocabularyVersion;
              // Histogram stays canonical zero; PrepareCapture alone owns BoW.
              for feature in 1:RGBDKeyframes.featureCapacity loop
                if SLAMExactRealEqual(currentEnabled[feature],1.0) then
                  candidate.enabled[feature] := true;
                  candidate.descriptor[feature,:] := currentDescriptor[feature,:];
                  candidate.opticalPoint[feature,:] := currentPoint[feature,:];
                  for coordinate in 1:2 loop
                    candidate.pixels[feature,coordinate] := integer(pixels[feature,coordinate]);
                  end for;
                end if;
              end for;
              frame := candidate; accepted := true; rejectionReason := 0;
            end if;
          end if;
        end if;
      end if;
    end if;
  end Build;
end RGBDLocalizationFrame;
