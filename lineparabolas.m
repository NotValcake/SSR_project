function [q, qd, qdd, tt, T, t] = lineparabolas(theta, A, V)
%LINEPARABOLAS Generate 1D trajectory with parabolic blends through via points
%   This function creates a time-optimal one-dimensional trajectory consisting
%   of linear segments connected by parabolic blends that passes exactly through
%   the first and last via points, and near intermediate via points. The
%   trajectory respects specified maximum acceleration and velocity constraints.
%
%   Inputs:
%       theta : Vector of via points (1×n or n×1)
%       A     : Maximum allowable acceleration (scalar, positive)
%       V     : Maximum allowable velocity (scalar, positive)
%
%   Outputs:
%       q     : Position values along trajectory (1×N)
%       qd    : Velocity values along trajectory (1×N)
%       qdd   : Acceleration values along trajectory (1×N)
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

    % Number of via points
    n = length(theta);

    % Initialize timing parameters
    tau = zeros(n,1);  % Minimum actuation time for each segment
    T = zeros(n,1);    % Absolute time for each via point
    t = zeros(n,1);    % Duration of each parabolic blend

    % Initialize kinematic parameters
    thetad = zeros(n,1);   % Velocity at via points
    thetadd = zeros(n,1);  % Acceleration during blends

    % Segment generation parameters
    nsamples = 50;         % Number of samples per trajectory segment
    qseg = zeros(nsamples, 1);  % Position samples for current segment
    qdseg = zeros(nsamples, 1); % Velocity samples for current segment
    qddseg = zeros(nsamples, 1);% Acceleration samples for current segment

    % Output arrays initialization
    q = [];
    qd = [];
    qdd = [];
    tt = [];

    % Main loop through all via point segments
    for j = 1:n-1
        % Segment completion flag (for constraint checking)
        segcomp = false;

        % Iterate until segment meets all constraints
        while ~segcomp
            if j == 1
                %% First segment handling %%
                % Compute minimum time for first two segments
                [~,~,tau(j)] = minactime(theta(j+1)-theta(j), A, A, V, "ts");
                [~,~,tau(j+1)] = minactime(theta(j+2)-theta(j+1), A, A, V, "ts");

                % Enforce velocity constraints
                tau(j) = max(tau(j), abs(theta(j+1)-theta(j))/V);
                tau(j+1) = max(tau(j+1), abs(theta(j+2)-theta(j+1))/V);

                % Calculate absolute timing for via points
                T(j+1) = T(j) + tau(j);
                T(j+2) = T(j+1) + tau(j+1);

                % Calculate initial acceleration and blend time
                thetadd(j) = sign(theta(j+1)-theta(j))*A;
                t(j) = tau(j) - sqrt(tau(j)^2 - 2*(theta(j+1)-theta(j))/thetadd(j));
                thetad(j) = thetadd(j)*t(j);

                % Initialize next segment parameters
                thetadd(j+1) = sign(theta(j+2)-theta(j+1))*A;
                thetad(j+1) = (theta(j+2)-theta(j+1))/tau(j+1);
                t(j+1) = abs(thetad(j+1)-thetad(j))/A;

            elseif j < n-1
                %% Intermediate segments handling %%
                % Compute minimum time for next segment
                [~,~,tau(j+1)] = minactime(theta(j+2)-theta(j+1), A, A, V, "ts");

                % Enforce velocity constraint
                tau(j+1) = max(tau(j+1), abs(theta(j+2)-theta(j+1))/V);

                % Update absolute timing
                T(j+2) = T(j+1) + tau(j+1);

                % Calculate current segment parameters
                thetad(j+1) = (theta(j+2)-theta(j+1))/tau(j+1);
                thetadd(j+1) = sign(theta(j+2)-theta(j+1))*A;

                % Handle final segment differently
                if j == n-2
                    % Final segment acceleration setup
                    thetadd(j+2) = -sign(theta(j+2)-theta(j+1))*A;
                    t(j+2) = tau(j+1) - sqrt(tau(j+1)^2 - 2*(theta(j+2)-theta(j+1))/-thetadd(j+2));
                    thetad(j+1) = -thetadd(j+2)*t(j+2);
                    t(j+1) = abs(thetad(j+1)-thetad(j))/A;
                else
                    % Regular intermediate blend time
                    t(j+1) = abs(thetad(j+1)-thetad(j))/A;
                end
            end

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
        ttseg = linspace(T(j), T(j+1), nsamples);

        % Generate samples for current segment
        for i = 1:length(ttseg)
            if j == 1
                %% First segment trajectory construction %%
                if ttseg(i) < T(j) + t(j)
                    % Initial parabolic blend
                    qddseg(i) = thetadd(j);
                    qdseg(i) = thetadd(j) * ttseg(i);
                    qseg(i) = theta(j) + 0.5 * thetadd(j) * ttseg(i)^2;
                elseif ttseg(i) < T(j+1) - t(j+1)/2
                    % Linear segment
                    qseg(i) = theta(j) + thetad(j) * (ttseg(i) - (T(j) + t(j)/2));
                    qdseg(i) = thetad(j);
                    qddseg(i) = 0;
                else
                    % Final parabolic blend of first segment
                    tp = ttseg(i) - (T(j+1) - t(j+1)/2);
                    qddseg(i) = thetadd(j+1);
                    qdseg(i) = thetad(j) + thetadd(j+1) * tp;
                    qseg(i) = theta(j+1) - thetad(j)*t(j+1)/2 + thetad(j)*tp + 0.5*thetadd(j+1)*tp^2;
                end
            elseif j < n-1
                %% Intermediate segments trajectory construction %%
                if ttseg(i) < T(j) + t(j)/2
                    % Initial parabolic blend
                    tp = ttseg(i) - (T(j) - t(j)/2);
                    qddseg(i) = thetadd(j);
                    qdseg(i) = thetad(j-1) + thetadd(j) * tp;
                    qseg(i) = theta(j) - thetad(j-1)*t(j)/2 + thetad(j-1)*tp + 0.5*thetadd(j)*tp^2;
                elseif ttseg(i) < T(j+1) - t(j+1)/2
                    % Linear segment
                    qseg(i) = theta(j) + thetad(j) * (ttseg(i) - T(j));
                    qdseg(i) = thetad(j);
                    qddseg(i) = 0;
                else
                    % Final parabolic blend
                    tp = ttseg(i) - (T(j+1) - t(j+1)/2);
                    qddseg(i) = thetadd(j+1);
                    qdseg(i) = thetad(j) + thetadd(j+1) * tp;
                    qseg(i) = theta(j+1) - thetad(j)*t(j+1)/2 + thetad(j)*tp + 0.5*thetadd(j+1)*tp^2;
                end
            else
                %% Final segment trajectory construction %%
                if ttseg(i) < T(j) + t(j)/2
                    % Initial parabolic blend
                    tp = ttseg(i) - (T(j) - t(j)/2);
                    qddseg(i) = thetadd(j);
                    qdseg(i) = thetad(j-1) + thetadd(j) * tp;
                    qseg(i) = theta(j) - thetad(j-1)*t(j)/2 + thetad(j-1)*tp + 0.5*thetadd(j)*tp^2;
                elseif ttseg(i) < T(j+1) - t(j+1)
                    % Linear segment
                    qseg(i) = theta(j) + thetad(j) * (ttseg(i) - T(j));
                    qdseg(i) = thetad(j);
                    qddseg(i) = 0;
                else
                    % Final parabolic blend to last point
                    tp = ttseg(i) - (T(j+1) - t(j+1));
                    qddseg(i) = thetadd(j+1);
                    qdseg(i) = thetad(j) + thetadd(j+1) * tp;
                    qseg(i) = theta(j+1) - thetad(j)*t(j+1)/2 + thetad(j)*tp + 0.5*thetadd(j+1)*tp^2;
                end
            end
        end

        %% Append current segment to full trajectory %%
        q = [q, qseg'];
        qd = [qd, qdseg'];
        qdd = [qdd, qddseg'];
        tt = [tt, ttseg];
    end
end