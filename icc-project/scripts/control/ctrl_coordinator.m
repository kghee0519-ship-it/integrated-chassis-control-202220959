function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Integrated steering, braking, and damping allocation.
%
%   Longitudinal braking is front/rear biased, ESC yaw moment is realized by
%   one-sided differential braking, and signed ABS corrections are added per
%   wheel. During high-speed stability preconditioning, the lateral channel
%   reduces common brake demand while steering is large so tire force is
%   reserved for path following.

    %#ok<INUSD> vx is retained by the required project interface.

    frontBrakeBias = local_field(CTRL.COORD, 'frontBrakeBias', 0.58);
    escFrontRatio = local_field(CTRL.COORD, 'escFrontRatio', 0.90);

    %% AFS steering command
    steerCmd = min(max(latCmd.steerAngle, -LIM.MAX_STEER_ANGLE), ...
                   LIM.MAX_STEER_ANGLE);

    %% Conventional longitudinal-force allocation
    brakeLon = zeros(4,1);
    if isfield(lonCmd, 'Fx_total') && lonCmd.Fx_total < 0
        totalBrakeTorque = -lonCmd.Fx_total * VEH.rw;
        frontEach = totalBrakeTorque * frontBrakeBias / 2;
        rearEach = totalBrakeTorque * (1 - frontBrakeBias) / 2;
        brakeLon = [frontEach; frontEach; rearEach; rearEach];
    end

    %% Stability-speed supervisor allocation
    brakeStability = zeros(4,1);
    if isfield(lonCmd, 'stabilityBrakeActive') && lonCmd.stabilityBrakeActive
        nominalEach = local_field(lonCmd, 'stabilityBrakePerWheel', ...
                                  LIM.MAX_BRAKE_TRQ);
        brakeScale = local_field(latCmd, 'brakeScale', 1.0);
        totalTorque = 4 * nominalEach * brakeScale;
        frontEach = totalTorque * frontBrakeBias / 2;
        rearEach = totalTorque * (1 - frontBrakeBias) / 2;
        brakeStability = [frontEach; frontEach; rearEach; rearEach];
    end

    %% Per-wheel ABS signed correction
    if isfield(lonCmd, 'brakeTorqueAdjust') && ...
            numel(lonCmd.brakeTorqueAdjust) == 4
        brakeABS = lonCmd.brakeTorqueAdjust(:);
    else
        brakeABS = zeros(4,1);
    end

    %% ESC yaw-moment allocation
    yawMoment = local_field(latCmd, 'yawMoment', 0);
    htf = max(VEH.track_f / 2, 0.1);
    htr = max(VEH.track_r / 2, 0.1);

    frontTorque = abs(yawMoment) * escFrontRatio * VEH.rw / htf;
    rearTorque = abs(yawMoment) * (1 - escFrontRatio) * VEH.rw / htr;

    if yawMoment >= 0
        brakeESC = [frontTorque; 0; rearTorque; 0];
    else
        brakeESC = [0; frontTorque; 0; rearTorque];
    end

    %% Combined actuator command
    brakeCmd = brakeLon + brakeStability + brakeABS + brakeESC;
    brakeCmd = min(max(brakeCmd, -LIM.MAX_BRAKE_TRQ), LIM.MAX_BRAKE_TRQ);

    if isempty(verCmd) || numel(verCmd) ~= 4
        dampingCmd = 1500 * ones(4,1);
    else
        dampingCmd = verCmd(:);
    end
    dampingCmd = min(max(dampingCmd, CTRL.VER.cMin), CTRL.VER.cMax);

    actuatorCmd.steerAngle = steerCmd;
    actuatorCmd.brakeTorque = brakeCmd;
    actuatorCmd.dampingCoeff = dampingCmd;
end

function value = local_field(s, name, defaultValue)
%LOCAL_FIELD Return a structure field or a default value.
    if isfield(s, name)
        value = s.(name);
    else
        value = defaultValue;
    end
end
