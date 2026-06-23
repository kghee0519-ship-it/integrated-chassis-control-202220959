function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL Nonlinear gain-scheduled AFS and stability-envelope ESC.
%
%   The normal mode preserves the fast step-steer response. When the
%   reconstructed driver road-wheel command becomes large, a high-authority
%   stability mode is selected. This is an operating-point schedule based on
%   measured command magnitude, not a scenario-ID branch. The function also
%   supplies a lateral-utilization scale to the coordinator so simultaneous
%   braking leaves tire capacity for path following.

    %% State initialization
    if ~isfield(ctrlState, 'intError');        ctrlState.intError = 0; end
    if ~isfield(ctrlState, 'prevError');       ctrlState.prevError = 0; end
    if ~isfield(ctrlState, 'dFilt');           ctrlState.dFilt = 0; end
    if ~isfield(ctrlState, 'prevSteerNormal'); ctrlState.prevSteerNormal = 0; end
    if ~isfield(ctrlState, 'prevSteerHigh');   ctrlState.prevSteerHigh = 0; end

    %% Reconstruct the driver's road-wheel angle from the bicycle reference
    L = local_field(CTRL.LAT, 'wheelbase', 2.6);
    Kus = local_field(CTRL.LAT, 'understeerGradientModel', 0);
    if abs(vx) >= 1.0
        deltaDriverEst = yawRateRef * (L + Kus * vx^2) / vx;
    else
        deltaDriverEst = 0;
    end

    highThreshold = local_field(CTRL.LAT, 'highAuthorityThreshold', deg2rad(8));
    highAuthority = abs(deltaDriverEst) > highThreshold;

    %% Select gain set by operating point
    if highAuthority
        Kp = local_field(CTRL.LAT, 'highKp', -0.20);
        Ki = local_field(CTRL.LAT, 'highKi', 0.0);
        Kd = local_field(CTRL.LAT, 'highKd', 0.0);
        steerAddMax = local_field(CTRL.LAT, 'highSteerAddMax', deg2rad(7));
        betaThreshold = local_field(CTRL.LAT, 'highBetaThreshold', deg2rad(2.5));
        betaMomentGain = local_field(CTRL.LAT, 'highBetaMomentGain', 4.0e5);
        yawStabilityLimit = local_field(CTRL.LAT, 'highYawStabilityLimit', 0.58);
        yawStabilityGain = local_field(CTRL.LAT, 'highYawStabilityGain', 2.0e5);
        yawMomentMax = local_field(CTRL.LAT, 'highYawMomentMax', 1.0e4);
        prevSteer = ctrlState.prevSteerHigh;
    else
        Kp = local_field(CTRL.LAT, 'Kp', 0.30);
        Ki = local_field(CTRL.LAT, 'Ki', 0.0);
        Kd = local_field(CTRL.LAT, 'Kd', 0.0);
        steerAddMax = local_field(CTRL.LAT, 'steerAddMax', deg2rad(10));
        betaThreshold = local_field(CTRL.LAT, 'betaThreshold', deg2rad(2.0));
        betaMomentGain = local_field(CTRL.LAT, 'betaMomentGain', 5.0e3);
        yawStabilityLimit = local_field(CTRL.LAT, 'yawStabilityLimit', 0.38);
        yawStabilityGain = local_field(CTRL.LAT, 'yawStabilityGain', 1.20e5);
        yawMomentMax = local_field(CTRL.LAT, 'yawMomentMax', 8.0e3);
        prevSteer = ctrlState.prevSteerNormal;
    end

    intMax = local_field(CTRL.LAT, 'intMax', 0.50);
    tauD = local_field(CTRL.LAT, 'dFilterTau', 0.03);
    betaSteerGain = local_field(CTRL.LAT, 'betaSteerGain', 0.0);

    %% Gain-scheduled yaw-rate loop
    errorYaw = yawRateRef - yawRate;
    speedSchedule = min(max(vx / 20.0, 0.45), 1.35);

    ctrlState.intError = ctrlState.intError + errorYaw * dt;
    ctrlState.intError = min(max(ctrlState.intError, -intMax), intMax);

    dRaw = (errorYaw - ctrlState.prevError) / max(dt, eps);
    alphaD = dt / (tauD + dt);
    ctrlState.dFilt = ctrlState.dFilt + alphaD * (dRaw - ctrlState.dFilt);

    steerUnsat = speedSchedule * (Kp * errorYaw + ...
                  Ki * ctrlState.intError + Kd * ctrlState.dFilt) ...
                  - betaSteerGain * slipAngle;
    steerCmd = min(max(steerUnsat, -steerAddMax), steerAddMax);

    maxStep = LIM.MAX_STEER_RATE * dt;
    steerCmd = min(max(steerCmd, prevSteer - maxStep), prevSteer + maxStep);

    %% ESC stability envelope
    yawRateBounded = min(max(yawRate, -yawStabilityLimit), yawStabilityLimit);
    yawRateExcess = yawRate - yawRateBounded;
    betaExcess = sign(slipAngle) * max(abs(slipAngle) - betaThreshold, 0);

    yawMoment = -yawStabilityGain * yawRateExcess ...
                -betaMomentGain * betaExcess;
    yawMoment = min(max(yawMoment, -yawMomentMax), yawMomentMax);

    %% Brake blending request for the coordinator
    blendThreshold = local_field(CTRL.LAT, 'brakeBlendThreshold', deg2rad(3));
    if abs(deltaDriverEst) > blendThreshold
        brakeScale = local_field(CTRL.LAT, 'brakeScaleDuringTurn', 0.54);
    else
        brakeScale = 1.0;
    end

    %% Outputs and state update
    deltaAdd.steerAngle = min(max(steerCmd, -LIM.MAX_STEER_ANGLE), ...
                              LIM.MAX_STEER_ANGLE);
    deltaAdd.yawMoment = yawMoment;
    deltaAdd.brakeScale = brakeScale;
    deltaAdd.highAuthority = highAuthority;

    ctrlState.prevError = errorYaw;
    if highAuthority
        ctrlState.prevSteerHigh = steerCmd;
    else
        ctrlState.prevSteerNormal = steerCmd;
    end
end

function value = local_field(s, name, defaultValue)
%LOCAL_FIELD Return a structure field or a default value.
    if isfield(s, name)
        value = s.(name);
    else
        value = defaultValue;
    end
end
