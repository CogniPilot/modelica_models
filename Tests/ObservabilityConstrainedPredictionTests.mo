within Tests;
model ObservabilityConstrainedPredictionTests "Held-interval prediction keeps the carried global-yaw direction unobservable across corrections"
  function nullspaceDirection "Global-yaw nullspace direction (e_z x p, e_z x v, R' e_z, 0, 0) of the 15-state error"
    input Real position[3]; input Real velocity[3]; input Real rotation[3,3];
    output Real direction[15];
  algorithm
    direction := cat(1,cross({0.0,0.0,1.0},position),cross({0.0,0.0,1.0},velocity),transpose(rotation)*{0.0,0.0,1.0},zeros(6));
  end nullspaceDirection;

  function run
    output Boolean passed;
  protected
    import ES15PredictHeldInterval = SLAM.Inertial.ES15PredictHeldInterval;
    import SchmidtCorrectRelativePose = SLAM.Inertial.SchmidtCorrectRelativePose;
    constant Integer triples = 3;
    constant Real substepInterval = 0.02 "One substep of the held interval";
    constant Real gravity[3] = {0.0,0.0,-9.81};
    constant Real density[12] = {0.06,0.06,0.06,0.006,0.006,0.006,0.002,0.002,0.002,0.0002,0.0002,0.0002};
    constant Real covariance0[15,15] = diagonal({0.25,0.25,0.25,0.04,0.04,0.04,0.01,0.01,0.01,0.0004,0.0004,0.0004,0.000025,0.000025,0.000025});
    // Three update-capture-update triples of the 600 s hallway replay: the prior and
    // posterior of the first update, then the four held IMU intervals that lead to the
    // capture and to the second update (two per camera frame).
    Real priorPosition[triples,3]; Real priorVelocity[triples,3]; Real priorRotation[triples,3,3];
    Real position[triples,3]; Real velocity[triples,3]; Real rotation[triples,3,3];
    Real accelBias[triples,3]; Real gyroBias[triples,3];
    Real accel[triples,4,3]; Real gyro[triples,4,3]; Real h[triples,4];
    // The second update of each triple: its exported prior, its posterior, and the two
    // held intervals of the capture frame that follows it.
    Real priorPosition2[triples,3]; Real priorVelocity2[triples,3]; Real priorRotation2[triples,3,3];
    Real position2[triples,3]; Real velocity2[triples,3]; Real rotation2[triples,3,3];
    Real accelBias2[triples,3]; Real gyroBias2[triples,3];
    Real accelNext[triples,2,3]; Real gyroNext[triples,2,3]; Real hNext[triples,2];
    Real stale[9]; Real staleTransition[15,15]; Real directionNext[15]; Real chainNext[15]; Real staleChain[15];
    Real u[15]; Real carry[9]; Real carryOut[9]; Real zeroOut[9]; Real unitNorm;
    Real accepted; Real transition[15,15]; Real noise[15,15]; Integer substeps;
    Real p[3]; Real v[3]; Real R[3,3]; Real P[15,15]; Real X[15,6];
    Real acceptedPlain; Real plain[15,15]; Real noisePlain[15,15]; Integer substepsPlain;
    Real pPlain[3]; Real vPlain[3]; Real RPlain[3,3]; Real PPlain[15,15]; Real XPlain[15,6];
    Real target[15]; Real residual[15]; Real complement[15]; Real targetNorm; Real plainLeak;
    Real chain[15]; Real plainChain[15]; Real clone[6]; Real plainClone[6];
    Real referencePosition[3]; Real referenceRotation[3,3];
    Real chainResidual[6]; Real plainResidual[6]; Real jacobianNorm;
    Real cAccepted; Real cNis; Real cInnovation[6]; Real H[6,21]; Real cS[6,6];
    Real cP[3]; Real cV[3]; Real cR[3,3]; Real cBa[3]; Real cBg[3];
    Real cCov[15,15]; Real cCross[15,6]; Real cRef[6,6];
  algorithm
    // triple 1: leg "init accel", update row 35 (t 7.0 s), capture row 36, update row 37
    priorPosition[1,:] := {0.00781910766870885,-0.025123290653926017,-0.07285552886523368}; priorVelocity[1,:] := {0.04015288499468982,0.0011633184945807533,0.0007835790855421285};
    priorRotation[1,:,:] := {{0.9999838727776938,-0.005278524623313646,0.002095557760061799},{0.0052765263241931525,0.999985620174161,0.000957974343007462},{-0.002100584317464418,-0.0009469016278570445,0.9999973454578928}};
    position[1,:] := {-0.006326299813207929,-0.03135503897824777,-0.0764206483091017}; velocity[1,:] := {0.008626010588263948,-0.0075796512977512245,-0.0007503429523202723};
    rotation[1,:,:] := {{0.9999820218139182,-0.005965274152034955,-0.0006095516709297968},{0.00596558117544688,0.9999820791968664,0.0005031166377002247},{0.0006065395185999704,-0.0005067439225492523,0.9999996876601559}};
    accelBias[1,:] := {0.00029365138205164707,-2.9234417022779226e-05,-0.0032922816000094494}; gyroBias[1,:] := {0.00024156445512190625,0.0003652944819199388,-7.652144154470783e-05};
    accel[1,1,:] := {0.018350881957207674,-0.009337385902331753,9.83115808273719}; gyro[1,1,:] := {0.00012374402525199837,0.002913159242473422,0.0014108489083182354}; h[1,1] := 0.09999999999999964;
    accel[1,2,:] := {0.003389883434862377,-0.013618676329125983,9.822918832666637}; gyro[1,2,:] := {0.00019109054205881078,0.001228462947512882,0.0007378119070771555}; h[1,2] := 0.10000000000000053;
    accel[1,3,:] := {0.02842647050532982,-0.019096795890508403,9.876487758150642}; gyro[1,3,:] := {2.792639280105487e-05,0.001086852112580724,0.0013818029962388892}; h[1,3] := 0.09999999999999964;
    accel[1,4,:] := {0.01736057222997064,0.00031828999135758915,9.82826218576053}; gyro[1,4,:] := {-0.0001110838168690502,0.001087796841627621,0.0016369439744257917}; h[1,4] := 0.10000000000000053;
    priorPosition2[1,:] := {-0.0019420794497919576,-0.0349201987140202,-0.07430449848807072}; priorVelocity2[1,:] := {0.014118848709658991,-0.009609632055727003,0.012452553219790052};
    priorRotation2[1,:,:] := {{0.9999787850766472,-0.00651259493635032,-0.00012451436347182224},{0.006512666055098006,0.9999786244170431,0.0005795600458524821},{0.00012073726208453613,-0.0005803586709984973,0.9999998243031475}};
    position2[1,:] := {0.03138361824848336,-0.02515316841698446,-0.07891705040688267}; velocity2[1,:] := {0.08056897844046455,0.002162222219133503,-0.004304987900671336};
    rotation2[1,:,:] := {{0.9999813755136748,-0.0045884992282530694,0.00402421428494809},{0.004583847484233647,0.9999888161937952,0.0011644009504156712},{-0.00402951213177776,-0.0011459328795200166,0.9999912248964066}};
    accelBias2[1,:] := {0.00022692001249528066,-3.377725796620859e-05,-0.002174138351523289}; gyroBias2[1,:] := {0.0003144949640769428,-0.00015804259023487405,0.00013838052329749888};
    accelNext[1,1,:] := {0.0122611088539873,-0.015574195738299573,9.806314586307385}; gyroNext[1,1,:] := {0.0007583386366950112,0.0009266539841584855,0.0008512756320717466}; hNext[1,1] := 0.09999999999999964;
    accelNext[1,2,:] := {0.021094719499019893,-0.007931407440321126,9.81425816279956}; gyroNext[1,2,:] := {-4.7492034662739854e-05,0.0017818387929889328,-0.00010938211957466353}; hNext[1,2] := 0.09999999999999964;
    // triple 2: leg "+x late", update row 1001 (t 200.2 s), capture row 1002, update row 1003
    priorPosition[2,:] := {92.85110502464975,10.009560168795685,-0.3327047279054938}; priorVelocity[2,:] := {0.5051477134387146,0.09727887193181453,0.010826233487337097};
    priorRotation[2,:,:] := {{0.9833196436292162,-0.18185081170647932,0.003572217042067322},{0.1818421097664371,0.9833242277427856,0.0026287353878839646},{-0.003990685228273855,-0.0019353076613333956,0.9999901644594293}};
    position[2,:] := {92.85143804585091,9.98127246880564,-0.33609850863672086}; velocity[2,:] := {0.4883224092464187,0.09302808218532448,0.003988629426495951};
    rotation[2,:,:] := {{0.9837690336225591,-0.17940967857472406,0.003264309888850352},{0.17940404638963692,0.9837735523326093,0.001945731200270754},{-0.0035604247445021634,-0.0013285196998501629,0.9999927791794528}};
    accelBias[2,:] := {-0.002679843527626695,-0.00023833324162599734,0.020166646162859157}; gyroBias[2,:] := {0.00041430322535738394,-0.00037901251086500283,-2.657594185986617e-05};
    accel[2,1,:] := {-0.003068711123941758,0.002807641930102193,9.846313197302107}; gyro[2,1,:] := {0.0004686602942214356,-0.0004341177590940063,0.0014113812610604102}; h[2,1] := 0.10000000000002274;
    accel[2,2,:] := {-0.03249609127968902,0.01534999703697898,9.850811013321948}; gyro[2,2,:] := {-0.00018147432404159894,-0.00021809237915588732,0.0006750051229962128}; h[2,2] := 0.09999999999999432;
    accel[2,3,:] := {-0.035016461898575754,-0.02573707936754436,9.833169986946507}; gyro[2,3,:] := {0.00027707927998085454,-7.884168129635761e-05,-0.00014110094392065595}; h[2,3] := 0.09999999999999432;
    accel[2,4,:] := {-0.00418057959186287,-0.01764063222219384,9.848209144425196}; gyro[2,4,:] := {-0.0006324677554474586,-0.0003829692511096608,0.00019865922773505554}; h[2,4] := 0.09999999999999432;
    priorPosition2[2,:] := {93.04809849903215,10.019833900688049,-0.3332878173806701}; priorVelocity2[2,:] := {0.4953140883598708,0.09734465969883146,0.009769872388000277};
    priorRotation2[2,:,:] := {{0.9837285054337048,-0.17963161172667558,0.0032728681053553325},{0.1796253348045959,0.9837328058438339,0.0021226876708665228},{-0.003600929531953306,-0.0015002583407654133,0.9999923912367279}};
    position2[2,:] := {93.05454514869545,9.995727289752221,-0.3346314960085368}; velocity2[2,:] := {0.5069718461656815,0.09786946572045,0.005234310158915604};
    rotation2[2,:,:] := {{0.9838928761497694,-0.17870392292584664,0.00444029195523052},{0.17869651150035637,0.9839021752765971,0.0020164978510755246},{-0.00472916899017322,-0.0011905531880017368,0.9999881087011829}};
    accelBias2[2,:] := {-0.002671462341872563,-0.00023991738355141632,0.020315459930812612}; gyroBias2[2,:] := {0.00040410464150703143,-0.0004153085313965147,-1.2671457209026879e-05};
    accelNext[2,1,:] := {-0.022915881415295148,-0.014175622198114343,9.81720777152679}; gyroNext[2,1,:] := {0.0006372309593132335,-0.0007640392902931012,0.0015555573220903426}; hNext[2,1] := 0.09999999999999432;
    accelNext[2,2,:] := {-0.03800741084204734,-0.0017062138388048535,9.811063039475794}; gyroNext[2,2,:] := {0.0005732752405558819,-0.0008802664230690973,-0.0001742679117747155}; hNext[2,2] := 0.10000000000002274;
    // triple 3: leg "-x late", update row 2600 (t 520.0 s), capture row 2601, update row 2602
    priorPosition[3,:] := {33.56144145786162,22.43598379067177,-0.2785814983457169}; priorVelocity[3,:] := {-0.4781766141730449,0.1267734584558977,0.011665526103673335};
    priorRotation[3,:,:] := {{0.9786212358611315,0.20562468872099404,-0.004354780136010384},{-0.20560981906615944,0.9786273210255633,0.003628890446132748},{0.005007896286546882,-0.0026559236973602482,0.9999839333929371}};
    position[3,:] := {33.5372988503587,22.132857794442923,-0.2716422024700478}; velocity[3,:] := {-0.4523367589177588,0.09418917567720297,0.021027770417258354};
    rotation[3,:,:] := {{0.980239717123773,0.19780502597771313,-0.0018079466424042673},{-0.19780199977423343,0.9802404788472417,0.0017240982119299281},{0.00211325777407955,-0.0013324140822031205,0.9999968794022779}};
    accelBias[3,:] := {-0.003027113883600639,-0.0008691415901718979,0.01995323858027323}; gyroBias[3,:] := {0.00041926910823009036,-0.00040159264694435463,0.0016849857254332145};
    accel[3,1,:] := {0.0586947207537982,-0.017522344711147504,9.846030655348747}; gyro[3,1,:] := {0.0005010536276265475,0.00022448251266427239,0.0005062648397502975}; h[3,1] := 0.10000000000002274;
    accel[3,2,:] := {0.05479723985977672,0.01716870775539326,9.828709151848333}; gyro[3,2,:] := {-0.00037188128862752684,0.00038192014069136384,0.0001057028227442441}; h[3,2] := 0.10000000000002274;
    accel[3,3,:] := {0.03928592200011454,-0.0020027081188960655,9.816066155433194}; gyro[3,3,:] := {0.0004305454006968764,-0.0009154852698177365,0.0008887380555256078}; h[3,3] := 0.09999999999990905;
    accel[3,4,:] := {0.03532631685890819,-0.0023433933023633714,9.826844415096764}; gyro[3,4,:] := {0.0006968227687274636,-0.00043965353478704085,-0.0005464477274946261}; h[3,4] := 0.10000000000002274;
    priorPosition2[3,:] := {33.359317390273524,22.17086399040525,-0.26291643450875696}; priorVelocity2[3,:] := {-0.4394772937653299,0.09695182059241503,0.020840569665121628};
    priorRotation2[3,:,:] := {{0.9801252610459902,0.19837219946451887,-0.00171555796969305},{-0.1983692483943824,0.9801258003672044,0.0017483552139190143},{0.0020282876973514185,-0.0013732931654177635,0.9999970000529066}};
    position2[3,:] := {33.278145645455844,22.140403117109905,-0.2839286666568024}; velocity2[3,:] := {-0.49314851960072364,0.1372958978961521,-0.013125782041828092};
    rotation2[3,:,:] := {{0.9805117986304934,0.19637619831595896,-0.005744691581444642},{-0.19635527427719005,0.9805246194261018,0.004009608812296285},{0.0064202032619092506,-0.0028034682572368797,0.9999754604768091}};
    accelBias2[3,:] := {-0.003215292619476548,-0.0010661977559699514,0.02106892634065644}; gyroBias2[3,:] := {0.00046389389818471817,-0.0002623828526167764,0.0017506918046436967};
    accelNext[3,1,:] := {0.033401394356021,0.004035451436308091,9.835111711201556}; gyroNext[3,1,:] := {0.0006442817640121882,-0.0002697800755434023,0.0017858411884679238}; hNext[3,1] := 0.10000000000002274;
    accelNext[3,2,:] := {0.04378141346379334,-0.0172403617047211,9.83258420843408}; gyroNext[3,2,:] := {2.8381531813808823e-05,-0.0010389467066800305,0.0009415298864916796}; hNext[3,2] := 0.10000000000002274;

    for n in 1:triples loop
      // The direction the first update's Jacobian was linearized at (its prior) is the one
      // carried into the prediction that starts from the corrected state.
      u := nullspaceDirection(priorPosition[n,:],priorVelocity[n,:],priorRotation[n,:,:]);
      carry := u[1:9];
      unitNorm := sqrt(u*u);

      // 1. One substep: the constrained transition maps the carried direction onto the
      //    direction at the predicted nominal, changes nothing orthogonal to it, keeps
      //    the nominal prediction and the covariance symmetric. The unconstrained call
      //    (zero carry) is the control and must leak, otherwise the test proves nothing.
      (accepted,transition,noise,p,v,R,P,X,substeps,carryOut) := ES15PredictHeldInterval(
        position[n,:],velocity[n,:],rotation[n,:,:],accelBias[n,:],gyroBias[n,:],covariance0,zeros(15,6),
        identity(6),zeros(3),identity(3),0.0,accel[n,1,:],gyro[n,1,:],gravity,substepInterval,density,carry);
      (acceptedPlain,plain,noisePlain,pPlain,vPlain,RPlain,PPlain,XPlain,substepsPlain,zeroOut) := ES15PredictHeldInterval(
        position[n,:],velocity[n,:],rotation[n,:,:],accelBias[n,:],gyroBias[n,:],covariance0,zeros(15,6),
        identity(6),zeros(3),identity(3),0.0,accel[n,1,:],gyro[n,1,:],gravity,substepInterval,density,zeros(9));
      assert(accepted > 0.5 and acceptedPlain > 0.5 and substeps == 1 and substepsPlain == 1,
        "Both substep predictions are accepted as one substep");
      assert(max(abs(p-pPlain)) == 0.0 and max(abs(v-vPlain)) == 0.0 and max(abs(R-RPlain)) == 0.0,
        "The constraint leaves the nominal prediction untouched");
      assert(max(abs(noise-noisePlain)) == 0.0, "The constraint leaves the process noise untouched");
      assert(max(abs(zeroOut)) == 0.0, "No carried direction leaves an interval that none entered");
      target := nullspaceDirection(p,v,R);
      targetNorm := sqrt(target*target);
      assert(max(abs(carryOut-target[1:9])) == 0.0, "The carry out is the direction at the predicted nominal");
      residual := transition*u-target;
      assert(sqrt(residual*residual) <= 1e-12*targetNorm, "Phi* u = N(x_end) for the carried direction u");
      residual := plain*u-target;
      plainLeak := sqrt(residual*residual)/targetNorm;
      assert(plainLeak > 1e-6, "The unconstrained transition moves the carried direction (control)");
      for i in 1:15 loop
        complement := zeros(15); complement[i] := 1.0;
        complement := complement-u*(u[i]/(u*u));
        residual := (transition-plain)*complement;
        assert(sqrt(residual*residual) <= 1e-12*(1.0+sqrt(sum(plain.^2))),
          "(Phi* - Phi) v = 0 for v orthogonal to the carried direction");
      end for;
      assert(max(abs(P-transpose(P))) <= 1e-15*max(abs(P)), "The predicted covariance stays symmetric");
      for i in 1:15 loop
        assert(P[i,i] > 0.0, "The predicted covariance keeps a positive diagonal");
        for j in 1:15 loop
          assert(abs(P[i,j]) <= sqrt(P[i,i]*P[j,j])*(1.0+1e-12), "The predicted covariance keeps every 2x2 minor non-negative");
        end for;
      end for;

      // 2. The chain through capture and second update: with Phi* on every interval the
      //    stacked [Phi*_(k+2) Phi*_(k+1) u ; S Phi*_(k+1) u] lies in the nullspace of the
      //    relative-pose Jacobian H_(k+2) at the chain's own nominal; with the unconstrained
      //    transitions it does not.
      carry := u[1:9]; chain := u; plainChain := u;
      p := position[n,:]; v := velocity[n,:]; R := rotation[n,:,:];
      pPlain := p; vPlain := v; RPlain := R;
      for i in 1:4 loop
        (accepted,transition,noise,p,v,R,P,X,substeps,carry) := ES15PredictHeldInterval(
          p,v,R,accelBias[n,:],gyroBias[n,:],covariance0,zeros(15,6),identity(6),zeros(3),identity(3),0.0,
          accel[n,i,:],gyro[n,i,:],gravity,h[n,i],density,carry);
        (acceptedPlain,plain,noisePlain,pPlain,vPlain,RPlain,PPlain,XPlain,substepsPlain,zeroOut) := ES15PredictHeldInterval(
          pPlain,vPlain,RPlain,accelBias[n,:],gyroBias[n,:],covariance0,zeros(15,6),identity(6),zeros(3),identity(3),0.0,
          accel[n,i,:],gyro[n,i,:],gravity,h[n,i],density,zeros(9));
        assert(accepted > 0.5 and acceptedPlain > 0.5 and substeps == substepsPlain and substeps > 1,
          "Both held-interval predictions are accepted with the same substep count");
        assert(max(abs(p-pPlain)) == 0.0 and max(abs(v-vPlain)) == 0.0 and max(abs(R-RPlain)) == 0.0,
          "The constraint leaves the held-interval nominal untouched");
        chain := transition*chain; plainChain := plain*plainChain;
        target := nullspaceDirection(p,v,R);
        residual := chain-target;
        assert(sqrt(residual*residual) <= 1e-12*sqrt(target*target),
          "The composed constrained transitions carry u onto the direction at the latest nominal");
        assert(max(abs(carry-target[1:9])) == 0.0, "The held interval carries out the direction at its predicted nominal");
        if i == 2 then
          // The reference clone copies the pose rows (1:3, 7:9) of the error at the capture.
          clone := cat(1,chain[1:3],chain[7:9]); plainClone := cat(1,plainChain[1:3],plainChain[7:9]);
          referencePosition := p; referenceRotation := R;
        end if;
      end for;
      (cAccepted,cNis,cInnovation,H,cS,cP,cV,cR,cBa,cBg,cCov,cCross,cRef) := SchmidtCorrectRelativePose(
        position=p,velocity=v,rotation=R,accelBias=accelBias[n,:],gyroBias=gyroBias[n,:],covariance=covariance0,
        referencePosition=referencePosition,referenceRotation=referenceRotation,referenceCovariance=identity(6),
        crossCovariance=zeros(15,6),measurementEnabled=0.0);
      jacobianNorm := sqrt(sum(H.^2));
      chainResidual := H*cat(1,chain,clone);
      plainResidual := H*cat(1,plainChain,plainClone);
      assert(sqrt(chainResidual*chainResidual) <= 1e-9*jacobianNorm,
        "H_(k+2) annihilates the constrained chain of the carried direction");
      assert(sqrt(plainResidual*plainResidual) > 1e-7*jacobianNorm,
        "H_(k+2) does not annihilate the unconstrained chain (control)");

      // 4. Two accepted updates in a row (k and k+2). The direction the state carries
      //    into the second update is the direction at that update's own prior, as the
      //    replay exported it, and not the first update's; the correction leaves it in
      //    place, so the prediction after the second update starts from it. A prediction
      //    built on the first update's direction instead would move the second's
      //    direction (control), which is the leak the carry rule prevents.
      target := nullspaceDirection(priorPosition2[n,:],priorVelocity2[n,:],priorRotation2[n,:,:]);
      directionNext := cat(1,carry,zeros(6));
      residual := directionNext-target;
      assert(sqrt(residual*residual) <= 1e-9*sqrt(target*target),
        "The direction entering the second update is the direction at its own exported prior");
      residual := directionNext-u;
      assert(sqrt(residual*residual) > 1e-4*unitNorm,
        "The direction entering the second update is not the first update's");
      stale := u[1:9]; chainNext := directionNext; staleChain := directionNext;
      p := position2[n,:]; v := velocity2[n,:]; R := rotation2[n,:,:];
      pPlain := p; vPlain := v; RPlain := R;
      for i in 1:2 loop
        (accepted,transition,noise,p,v,R,P,X,substeps,carry) := ES15PredictHeldInterval(
          p,v,R,accelBias2[n,:],gyroBias2[n,:],covariance0,zeros(15,6),identity(6),zeros(3),identity(3),0.0,
          accelNext[n,i,:],gyroNext[n,i,:],gravity,hNext[n,i],density,carry);
        (acceptedPlain,staleTransition,noisePlain,pPlain,vPlain,RPlain,PPlain,XPlain,substepsPlain,stale) := ES15PredictHeldInterval(
          pPlain,vPlain,RPlain,accelBias2[n,:],gyroBias2[n,:],covariance0,zeros(15,6),identity(6),zeros(3),identity(3),0.0,
          accelNext[n,i,:],gyroNext[n,i,:],gravity,hNext[n,i],density,stale);
        assert(accepted > 0.5 and acceptedPlain > 0.5, "Both predictions after the second update are accepted");
        chainNext := transition*chainNext; staleChain := staleTransition*staleChain;
      end for;
      target := nullspaceDirection(p,v,R);
      residual := chainNext-target;
      assert(sqrt(residual*residual) <= 1e-12*sqrt(target*target),
        "After the second update the prediction carries that update's direction onto the predicted nominal");
      residual := staleChain-target;
      assert(sqrt(residual*residual) > 1e-7*sqrt(target*target),
        "A prediction built on the first update's direction would move the second's (control)");
    end for;
    passed := true;
  end run;
  parameter Boolean passed = run();
equation
  assert(passed,"Observability-constrained prediction tests did not complete");
end ObservabilityConstrainedPredictionTests;
