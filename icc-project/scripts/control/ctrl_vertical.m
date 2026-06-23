function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL Semi-active on-off skyhook damping controller.
%
%   High damping is selected when the damper can dissipate sprung-mass
%   motion; otherwise low damping is used. A short first-order filter avoids
%   discontinuous coefficient switching while retaining the semi-active
%   cMin/cMax limits.

    cMin = CTRL.VER.cMin;
    cMax = CTRL.VER.cMax;
    skyGain = CTRL.VER.skyGain;
    tau = local_field(CTRL.VER, 'filterTau', 0.02);

    if ~isfield(suspState, 'zs_dot') || ~isfield(suspState, 'zu_dot') || ...
            numel(suspState.zs_dot) ~= 4 || numel(suspState.zu_dot) ~= 4
        dampingCmd = 1500 * ones(4,1);
        ctrlState.prevDamping = dampingCmd;
        return;
    end

    zsDot = suspState.zs_dot(:);
    zuDot = suspState.zu_dot(:);
    relVel = zsDot - zuDot;

    dampingTarget = cMin * ones(4,1);
    for i = 1:4
        % Force-form skyhook. The requested damping force must remain
        % dissipative for a semi-active damper.
        if zsDot(i) * relVel(i) > 0
            cForce = skyGain * abs(zsDot(i)) / max(abs(relVel(i)), 1e-4);
            dampingTarget(i) = min(max(cForce, cMin), cMax);
        end
    end

    if ~isfield(ctrlState, 'prevDamping') || numel(ctrlState.prevDamping) ~= 4
        ctrlState.prevDamping = dampingTarget;
    end

    alpha = dt / (tau + dt);
    dampingCmd = ctrlState.prevDamping + alpha * ...
                 (dampingTarget - ctrlState.prevDamping);
    dampingCmd = min(max(dampingCmd, cMin), cMax);

    ctrlState.prevDamping = dampingCmd;
end

function value = local_field(s, name, defaultValue)
%LOCAL_FIELD Return a structure field or a default value.
    if isfield(s, name)
        value = s.(name);
    else
        value = defaultValue;
    end
end
