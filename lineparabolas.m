function [q, qd, qdd, tt, T, t] = lineparabolas(theta, A, V, dt)
%LINEPARABOLAS Generate multi trajectory with parabolic blends through via points
%   This function creates a time-optimal multi-dimensional trajectory consisting
%   of linear segments connected by parabolic blends that passes exactly through
%   the first and last via points, and near intermediate via points. The
%   trajectory respects specified maximum acceleration and velocity constraints.
%
%   Inputs:
%       theta : Vector of via points (dxn)
%       A     : Maximum allowable acceleration (dx1, positive)
%       V     : Maximum allowable velocity (dx1, positive)
%       dt    : Time step
%
%   Outputs:
%       q     : Position values along trajectory (d×N)
%       qd    : Velocity values along trajectory (d×N)
%       qdd   : Acceleration values along trajectory (d×N)
%       tt    : Time vector corresponding to trajectory points (1×N)
%       T     : Time instants when passing near via points (1×n)
%       t     : Duration of each parabolic blend segment (1×n)
%
%   The algorithm follows these steps:
%   1. Calculate minimum time segments between points considering constraints
%   2. Compute parabolic blend durations to connect linear segments
%   3. Verify and enforce velocity/acceleration constraints iteratively
%   4. Generate trajectory samples for each segment with proper blends
%
%   Note: Intermediate via points may not be exactly reached due to blending

if nargin < 4
    dt = 1/100;
end

% Number of via points
[d,n] = size(theta);

% Check via points validity
for j = 1:n-1
    if abs(theta(:,j+1)-theta(:,j)) < 1e-15
        error("Points %d and %d are equal. Adjacent via points must be different.", j, j+1)
    end
end

% Initialize timing parameters
tau = zeros(1,n);  % Minimum actuation time for each segment
T = zeros(1,n);    % Absolute time for each via point
t = zeros(1,n);    % Duration of each parabolic blend

% Initialize kinematic parameters
thetad = zeros(d,n);   % Velocity at via points
thetadd = zeros(d,n);  % Acceleration during blends

% Output arrays initialization
q = [];
qd = [];
qdd = [];
tt = [];

% Pre-allocate candidate times
taucandidate = zeros(1,d);
taunextcandidate = zeros(1,d);

% Main loop through all via point segments
for j = 1:n-1
    % Segment completion flag (for constraint checking)
    segcomp = false;

    % Iterate until segment meets all constraints
    while ~segcomp
        if j == 1
            %% First segment handling %%
            for k=1:d
                % Compute minimum time for first two segments
                [~,~,taucandidate(k)] = minactime(theta(k,j+1)-theta(k,j), A(k), A(k), V(k), "ts");
                [~,~,taunextcandidate(k)] = minactime(theta(k,j+2)-theta(k,j+1), A(k), A(k), V(k), "ts");

                % Enforce velocity constraints
                taucandidate(k) = max(taucandidate(k), abs(theta(k,j+1)-theta(k,j))/V(k));
                taunextcandidate(k) = max(taunextcandidate(k), abs(theta(k,j+2)-theta(k,j+1))/V(k));
            end

            % Select the maximum actuation time among actuators
            tau(j) = max(taucandidate);
            tau(j+1) = max(taunextcandidate);

            % Calculate absolute timing for via points
            T(j+1) = T(j) + tau(j);
            T(j+2) = T(j+1) + tau(j+1);

            % Calculate initial acceleration and blend time
            thetadd(:,j) = sign(theta(:,j+1)-theta(:,j)).*A;
            t(j) = max(tau(j) - sqrt(tau(j)^2 - 2*(theta(:,j+1)-theta(:,j))./thetadd(:,j)));
            thetad(:,j) = (theta(:,j+1)-theta(:,j))/(tau(j)-t(j)/2);

            % Compute actual blend accelerations
            thetadd(:,j) = (thetad(:,j))./t(j);

            % Initialize next segment parameters
            thetad(:,j+1) = (theta(:,j+2)-theta(:,j+1))./tau(j+1);
            t(j+1) = max(abs(thetad(:,j+1)-thetad(:,j))./A);

            % Compute actual blend accelerations
            if abs(t(j+1)) > 1e-15
                thetadd(:,j+1) = (thetad(:,j+1)-thetad(:,j))./t(j+1);
            else
                % If t(j+1) = 0, points are aligned and acceleration is
                % null
                thetadd(:,j+1) = zeros(d,1);
            end
        elseif j < n-1
            %% Intermediate segments handling %%
            for k=1:d
                % Compute minimum time
                [~,~,taunextcandidate(k)] = minactime(theta(k,j+2)-theta(k,j+1), A(k), A(k), V(k), "ts");

                % Enforce velocity constraints
                taunextcandidate(k) = max(taunextcandidate(k), abs(theta(k,j+2)-theta(k,j+1))/V(k));
            end

            % Select the maximum actuation time among actuators
            tau(j+1) = max(taunextcandidate);

            % Update absolute timing
            T(j+2) = T(j+1) + tau(j+1);

            % Calculate current segment parameters
            thetad(:,j+1) = (theta(:,j+2)-theta(:,j+1))/tau(j+1);

            % Handle final segment differently
            if j == n-2
                tau(j+2) = max(taunextcandidate);

                % Final segment acceleration setup
                % Compute auxiliary blend acceleration
                thetadd(:,j+2) = -sign(theta(:,j+2)-theta(:,j+1)).*A;
                t(j+2) = max(tau(j+1) - sqrt(tau(j+1)^2 - 2*(theta(:,j+2)-theta(:,j+1))./-thetadd(:,j+2)));
                thetad(:,j+1) = (theta(:,j+2)-theta(:,j+1))/(tau(j+1)-t(j+2)/2);
                t(j+1) = max(abs(thetad(:,j+1)-thetad(:,j))./A);

                % Compute actual blend accelerations
                if abs(t(j+1)) > 1e-15
                    thetadd(:,j+1) = (thetad(:,j+1)-thetad(:,j))./t(j+1);
                else
                    % If t(j+1) = 0, points are aligned and acceleration is
                    % null
                    thetadd(:,j+1) = zeros(d,1);
                end
                thetadd(:,j+2) = (thetad(:,j+2)-thetad(:,j+1))./t(j+2);
            else
                % Regular intermediate blend time
                t(j+1) = max(abs(thetad(:,j+1)-thetad(:,j))./A);

                % Compute actual blend accelerations
                if abs(t(j+1)) > 1e-15
                    thetadd(:,j+1) = (thetad(:,j+1)-thetad(:,j))./t(j+1);
                else
                    % If t(j+1) = 0, points are aligned and acceleration is
                    % null
                    thetadd(:,j+1) = zeros(d,1);
                end
            end
        end
        % taucandidate = taunextcandidate;

        %% Acceleration constraint verification %%
        if tau(j) >= (t(j) + t(j+1))/2
            segcomp = true;
        else
            % Adjust time segment to meet acceleration constraint
            tau(j) = (t(j) + t(j+1))/2;
        end
    end

    %% Trajectory segment generation %%
    % Create time vector for current segment
    ttseg = T(j):dt:T(j+1);

    % Segment generation parameters
    qseg = zeros(d, length(ttseg));  % Position samples for current segment
    qdseg = zeros(d, length(ttseg)); % Velocity samples for current segment
    qddseg = zeros(d, length(ttseg));% Acceleration samples for current segment

    % Generate samples for current segment
    for i = 1:length(ttseg)
        if j == 1
            %% First segment trajectory construction %%
            if ttseg(i) < T(j) + t(j)
                % Initial parabolic blend
                qddseg(:,i) = thetadd(:,j);
                qdseg(:,i) = thetadd(:,j) .* ttseg(i);
                qseg(:,i) = theta(:,j) + 0.5 * thetadd(:,j) * ttseg(i)^2;
            elseif ttseg(i) < T(j+1) - t(j+1)/2
                % Linear segment
                qseg(:,i) = theta(:,j) + thetad(:,j) * (ttseg(i) - (T(j) + t(j)/2));
                qdseg(:,i) = thetad(:,j);
                qddseg(:,i) = zeros(d,1);
            else
                % Final parabolic blend of first segment
                tp = ttseg(i) - (T(j+1) - t(j+1)/2);
                qddseg(:,i) = thetadd(:,j+1);
                qdseg(:,i) = thetad(:,j) + thetadd(:,j+1) * tp;
                qseg(:,i) = theta(:,j+1) - thetad(:,j)*t(j+1)/2 + thetad(:,j)*tp + 0.5*thetadd(:,j+1)*tp^2;
            end
        elseif j < n-1
            %% Intermediate segments trajectory construction %%
            if ttseg(i) < T(j) + t(j)/2
                % Initial parabolic blend
                tp = ttseg(i) - (T(j) - t(j)/2);
                qddseg(:,i) = thetadd(:,j);
                qdseg(:,i) = thetad(:,j-1) + thetadd(:,j) * tp;
                qseg(:,i) = theta(:,j) - thetad(:,j-1)*t(j)/2 + thetad(:,j-1)*tp + 0.5*thetadd(:,j)*tp^2;
            elseif ttseg(i) < T(j+1) - t(j+1)/2
                % Linear segment
                qseg(:,i) = theta(:,j) + thetad(:,j) * (ttseg(i) - T(j));
                qdseg(:,i) = thetad(:,j);
                qddseg(:,i) = zeros(d,1);
            else
                % Final parabolic blend
                tp = ttseg(i) - (T(j+1) - t(j+1)/2);
                qddseg(:,i) = thetadd(:,j+1);
                qdseg(:,i) = thetad(:,j) + thetadd(:,j+1) * tp;
                qseg(:,i) = theta(:,j+1) - thetad(:,j)*t(j+1)/2 + thetad(:,j)*tp + 0.5*thetadd(:,j+1)*tp^2;
            end
        else
            %% Final segment trajectory construction %%
            if ttseg(i) < T(j) + t(j)/2
                % Initial parabolic blend
                tp = ttseg(i) - (T(j) - t(j)/2);
                qddseg(:,i) = thetadd(:,j);
                qdseg(:,i) = thetad(:,j-1) + thetadd(:,j) * tp;
                qseg(:,i) = theta(:,j) - thetad(:,j-1)*t(j)/2 + thetad(:,j-1)*tp + 0.5*thetadd(:,j)*tp^2;
            elseif ttseg(i) < T(j+1) - t(j+1)
                % Linear segment
                qseg(:,i) = theta(:,j) + thetad(:,j) * (ttseg(i) - T(j));
                qdseg(:,i) = thetad(:,j);
                qddseg(:,i) = zeros(d,1);
            else
                % Final parabolic blend to last point
                tp = ttseg(i) - (T(j+1) - t(j+1));
                qddseg(:,i) = thetadd(:,j+1);
                qdseg(:,i) = thetad(:,j) + thetadd(:,j+1) * tp;
                qseg(:,i) = theta(:,j+1) - thetad(:,j)*t(j+1)/2 + thetad(:,j)*tp + 0.5*thetadd(:,j+1)*tp^2;
            end
        end
    end

    %% Append current segment to full trajectory %%
    q = [q, qseg];
    qd = [qd, qdseg];
    qdd = [qdd, qddseg];
    tt = [tt, ttseg];
end
end