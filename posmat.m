function T = posmat(angle, axis, len)
% POSMAT computes the position matrix matrix for a rotation around a 
% specified axis (x, y, or z) by a given angle,
%
% INPUTS:
%   angle    - Rotation angle in radians.
%   axis     - A character ('x', 'y', or 'z') specifying the axis of rotation.
%   len      - A scalar specifying the length of the link.
%
% OUTPUT:
%   T        - A 4x4 homogeneous transformation matrix representing the
%              rotation around the specified axis by the given angle.
%
% EXAMPLE USAGE:
%   T = posmat(pi/4, 'x', 5); % 45 degrees around x-axis
%
%   T = posmat(-pi/2, 'z', 10); % -90 degrees around z-axis
%
% The homogeneous transformation matrix has the form:
%      [ R  t ]
%      [ 0  1 ]
% where R is the 3x3 rotation matrix, t is the 1x3 translation vector, and
% 0 is a 1x3 zero vector.
%
% Author: RV

    % Initialize the 4x4 identity matrix
    T = eye(4);
    
    % Initialize the rotation matrix and translation vector
    R = eye(3);
    t = [0, 0, 0];
    
    % Check which axis was selected and create the corresponding rotation matrix
    switch lower(axis)
        case 'x'
            % Rotation matrix around the x-axis
            R = [1, 0, 0;
                 0, cos(angle), -sin(angle);
                 0, sin(angle), cos(angle)];
            % Compute translation along the rotated y and z axes
            t = [0, len * cos(angle), len * sin(angle)];
            
        case 'y'
            % Rotation matrix around the y-axis
            R = [cos(angle), 0, sin(angle);
                 0, 1, 0;
                 -sin(angle), 0, cos(angle)];
            % Compute translation along the rotated x and z axes
            t = [len * cos(angle), 0, -len * sin(angle)];
            
        case 'z'
            % Rotation matrix around the z-axis
            R = [cos(angle), -sin(angle), 0;
                 sin(angle), cos(angle), 0;
                 0, 0, 1];
            % Compute translation along the rotated x and y axes
            t = [len * cos(angle), len * sin(angle), 0];
            
        otherwise
            error('Invalid axis input. Choose x, y, or z.');
    end
    
    % Insert the 3x3 rotation matrix into the top-left corner of the 4x4 matrix
    T(1:3, 1:3) = R;
    
    % Insert the computed translation vector into the top-right corner of the 4x4 matrix
    T(1:3, 4) = t(:);

end
