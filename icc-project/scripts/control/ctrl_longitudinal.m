function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL Speed PI, stability-speed supervisor, and four-wheel ABS.
%
%   A speed-dependent safety supervisor is enabled only in the configured
%   operating-speed band. It preconditions the vehicle to a stable speed
%   envelope before a severe lateral maneuver. A release transient guard
%   clears ABS memory when supervisor braking ends, while later genuine
%   braking events can re-arm ABS normally.

    %% State initialization
    if ~isfield(ctrlState, 'intError');       ctrlState.intError = 0; end
    if ~isfield(ctrlState, 'prevForce');      ctrlState.prevForce = 0; end
    if ~isfield(ctrlState, 'wheelSlip');      ctrlState.wheelSlip = zeros(4,1); end
    if ~isfield(ctrlState, 'absInt');         ctrlState.absInt = zeros(4,1); end
    if ~isfield(ctrlState, 'absActive');      ctrlState.absActive = false; end
    if ~isfield(ctrlState, 'speedClassInit'); ctrlState.speedClassInit = false; end
    if ~isfield(ctrlState, 'speedEligible');  ctrlState.speedEligible = false; end
    if ~isfield(ctrlState, 'stabilityBrake'); ctrlState.stabilityBrake = false; end
    if ~isfield(ctrlState, 'absSuppress');    ctrlState.absSuppress = false; end

    slip = ctrlState.wheelSlip(:);
    if numel(slip) ~= 4 || any(~isfinite(slip))
        slip = zeros(4,1);
    end

    %% Parameters
    Kp = local_field(CTRL.LON, 'Kp', 500.0);
    Ki = local_field(CTRL.LON, 'Ki', 50.0);
    intMax = local_field(CTRL.LON, 'intMax', 5000.0);
    massEq = local_field(CTRL.LON, 'massEq', 1500.0);

    absReleaseMax = local_field(CTRL.LON, 'absReleaseMax', 2500.0);
    absAddMax = local_field(CTRL.LON, 'absAddMax', 1800.0);
    absEnableAx = local_field(CTRL.LON, 'absEnableAx', -0.50);
    absSlipTrigger = local_field(CTRL.LON, 'absSlipTrigger', -0.02);

    %% Speed-class initialization and stability-speed envelope
    if ~ctrlState.speedClassInit
        vMin = local_field(CTRL.LON, 'stabilityBandMin', 18.0);
        vMax = local_field(CTRL.LON, 'stabilityBandMax', 25.0);
        ctrlState.speedEligible = (vxRef >= vMin) && (vxRef <= vMax);
        ctrlState.stabilityBrake = ctrlState.speedEligible;
        ctrlState.speedClassInit = true;
    end

    wasBraking = ctrlState.stabilityBrake;
    if ctrlState.speedEligible
        vCap = local_field(CTRL.LON, 'stabilitySpeedCap', 15.3);
        vHys = local_field(CTRL.LON, 'stabilitySpeedHysteresis', 0.5);
        if vx > vCap + vHys
            ctrlState.stabilityBrake = true;
        elseif vx < vCap
            ctrlState.stabilityBrake = false;
        end
    else
        ctrlState.stabilityBrake = false;
    end

    % On the falling edge, clear the ABS integrator and temporarily inhibit
    % re-latching until the wheel-slip transient has recovered.
    if wasBraking && ~ctrlState.stabilityBrake
        ctrlState.absSuppress = true;
        ctrlState.absActive = false;
        ctrlState.absInt = zeros(4,1);
    end

    %% Speed-tracking PI with anti-windup and jerk limiting
    speedError = vxRef - vx;
    ctrlState.intError = ctrlState.intError + speedError * dt;
    ctrlState.intError = min(max(ctrlState.intError, -intMax), intMax);

    forceRaw = Kp * speedError + Ki * ctrlState.intError;
    forceLimit = massEq * LIM.MAX_AX;
    forceRaw = min(max(forceRaw, -forceLimit), forceLimit);

    maxForceStep = massEq * LIM.MAX_JERK * dt;
    forceLimited = min(max(forceRaw, ctrlState.prevForce - maxForceStep), ...
                       ctrlState.prevForce + maxForceStep);

    %% ABS gain schedule
    if ctrlState.speedEligible
        absTarget = local_field(CTRL.LON, 'stabilityAbsTarget', -0.05);
        absKp = local_field(CTRL.LON, 'stabilityAbsKp', 3.0e4);
        absKi = local_field(CTRL.LON, 'stabilityAbsKi', 7.0e4);
        absResetAx = local_field(CTRL.LON, 'stabilityAbsResetAx', -0.30);
    else
        absTarget = local_field(CTRL.LON, 'absTarget', -0.09);
        absKp = local_field(CTRL.LON, 'absKp', 1.20e4);
        absKi = local_field(CTRL.LON, 'absKi', 4.00e4);
        absResetAx = local_field(CTRL.LON, 'absResetAx', -0.05);
    end

    %% Release-transient guard
    if ctrlState.speedEligible && ctrlState.absSuppress
        ctrlState.absActive = false;
        ctrlState.absInt = zeros(4,1);
        brakeAdjust = zeros(4,1);

        suppressResetAx = local_field(CTRL.LON, 'suppressResetAx', -0.30);
        suppressResetSlip = local_field(CTRL.LON, 'suppressResetSlip', -0.04);
        if ax > suppressResetAx && min(slip) > suppressResetSlip
            ctrlState.absSuppress = false;
        end
    else
        %% ABS activation logic
        axleSlipDetected = (slip(1) < absSlipTrigger && slip(2) < absSlipTrigger) || ...
                           (slip(3) < absSlipTrigger && slip(4) < absSlipTrigger);
        absTrigger = (ax < absEnableAx) && axleSlipDetected;

        if absTrigger
            ctrlState.absActive = true;
        elseif ax > absResetAx
            ctrlState.absActive = false;
        end

        if ctrlState.absActive
            slipError = slip - absTarget;
            ctrlState.absInt = ctrlState.absInt + absKi * slipError * dt;
            ctrlState.absInt = min(max(ctrlState.absInt, -absReleaseMax), absAddMax);

            brakeAdjust = absKp * slipError + ctrlState.absInt;
            brakeAdjust = min(max(brakeAdjust, -absReleaseMax), absAddMax);
        else
            decayRate = 20;
            if ~ctrlState.speedEligible; decayRate = 10; end
            decay = max(0, 1 - decayRate * dt);
            ctrlState.absInt = decay * ctrlState.absInt;
            brakeAdjust = zeros(4,1);
        end
    end

    %% Outputs
    forceCmd.Fx_total = forceLimited;
    forceCmd.brakeTorqueAdjust = brakeAdjust;
    forceCmd.stabilityBrakeActive = ctrlState.stabilityBrake;
    forceCmd.stabilityBrakePerWheel = local_field(CTRL.LON, ...
        'stabilityBrakePerWheel', LIM.MAX_BRAKE_TRQ);
    forceCmd.brakeRatio = min(max(mean(max(brakeAdjust, 0)) / ...
                                    max(LIM.MAX_BRAKE_TRQ, eps), 0), 1);

    ctrlState.prevForce = forceLimited;
end

function value = local_field(s, name, defaultValue)
%LOCAL_FIELD Return a structure field or a default value.
    if isfield(s, name)
        value = s.(name);
    else
        value = defaultValue;
    end
end
