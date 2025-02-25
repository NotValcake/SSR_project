function [t1, t3, T] = minactime(ds, amax, dmax, vmax, lom)
% MINACTTIME Computes the minimum actuation time for a motion profile.
%
%   [t1, t3, T] = minactime(ds, amax, dmax, vmax)
%
%   INPUTS:
%       ds   - distance to be traversed
%       amax - maximum acceleration (positive)
%       dmax - maximum deceleration (positive, used as -dmax in deceleration)
%       vmax - maximum velocity allowed
%
%   OUTPUTS:
%       t1   - time duration of the acceleration phase
%       t3   - time duration of the deceleration phase
%       T    - total actuation time
%
%   The function computes the minimum time required to traverse a given
%   distance under acceleration, deceleration, and velocity constraints.
%   Depending on the distance and constraints, the motion follows one of
%   two possible profiles:
%   - **Trapezoidal profile** (acceleration → cruise → deceleration) if
%     the required displacement allows reaching vmax.
%   - **Triangular profile** (acceleration → deceleration) if vmax
%     cannot be reached within the given displacement.
%
%   Example:
%       [t1, t3, T] = minactime(10, 2, 2, 3);

if ~exist('lom','var')
    lom = "ts"; % default to a three steps law of motion
end

if lom == "ts"
    % Candidate times if vmax is reached
    t1 = vmax / amax;         % time to accelerate to vmax
    t3 = vmax / dmax;         % time to decelerate from vmax

    % Distances during acceleration and deceleration if vmax is reached
    accd = 0.5 * amax * t1^2;
    decd = 0.5 * vmax^2 / dmax;

    % Check if there is enough distance for a constant velocity phase.
    if ds > (accd + decd)
        % --- Trapezoidal profile ---
        % Compute cruise duration so that total displacement is ds:
        t2 = (ds - (accd + decd)) / vmax;
        T = t1 + t2 + t3;
    else
        % --- Triangular profile (no cruise) ---
        % Compute the peak velocity v_peak that can be reached:
        t1 = sqrt(dmax/amax * 2*ds/(amax+dmax));
        t3 = sqrt(amax/dmax * 2*ds/(amax+dmax));
        T = t1 + t3;
    end
elseif lom == "c" % if cycloidal law of motion
    T = max([2*ds/vmax, sqrt(2*pi*ds/amax), sqrt(2*pi*ds/dmax)]);
    t1 = -1;
    t3 = -1;
end

end

