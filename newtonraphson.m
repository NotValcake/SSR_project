function [Qi, i] = newtonraphson(F, J, Q0, S, tol, imax)
    % NEWTONRAPHSON Finds a zero of a nonlinear system using the Newton-Raphson method.
    %
    % Syntax:
    %   [Qi, i] = newtonraphson(F, J, Q0, S, tol, imax)
    %
    % Inputs:
    %   F    - Anonymous function representing the nonlinear system.
    %          Must return an (n×1) vector.
    %   J    - Anonymous function representing the Jacobian matrix.
    %   Q0   - Initial guess (n×1 vector).
    %   S    - Target vector for convergence testing.
    %   tol  - Convergence tolerance.
    %   imax - Maximum number of iterations.
    %
    % Outputs:
    %   Qi   - Approximate solution vector where F(Qi) is close to S.
    %   i    - Number of iterations required to find the solution.
    %
    % Note:
    %   The use of pinv (pseudoinverse) instead of the standard inverse
    %   helps to eliminate the problem of singularity in the Jacobian matrix.
    %   This ensures the method can proceed even when J is near-singular.
    
    % Initialization
    Qi = Q0;
    Si = F(Qi);
    i = 0;

    % Newton-Raphson iteration
    while i < imax
        JQi = J(Qi);  % Compute Jacobian at Qi

        % Check if the Jacobian is singular (or near-singular)
        % if abs(det(JQi)) < 1e-12
        %     error('Jacobian near singularity at iteration %d. Unable to proceed.', i);
        % end

        % Update Qi using the pseudoinverse to handle singular cases
        Qi = Qi + pinv(JQi) * (S - Si);
        Si = F(Qi);

        % Convergence test
        if norm(S - Si, inf) < tol
            fprintf('Solution found in %d iterations.\n', i + 1);
            return;
        end

        i = i + 1;  % Increment iteration count
    end

    % If max iterations are reached without convergence
    warning('Max iterations reached without finding a solution.');
end
