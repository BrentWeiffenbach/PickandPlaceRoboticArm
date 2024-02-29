

% Plans a cubic trajectory in joint/task space.

classdef Traj_Planner
    
    methods
  
        % solves for a cubic (3rd order) polynomial trajectory between two via-points
        % Input:
        % takes in desired t0, tf (initial and final time in seconds), and v0, vf 
        % (initial and final velocity), p0, pf (Starting and end positions)
        % 
        % Output:
        % 4-by-1 array containing the coefficients of the polynomial.
        function coefficients = cubic_traj(self, t0, tf, v0, vf, p0, pf)
            % Generic Cubic function coefficient matrix
            A = [1 t0 t0^2 t0^3;
                 0 1 2*t0 3*t0^2;
                 1 tf tf^2 tf^3;
                 0 1 2*tf 3*tf^2];
            % coefficient matrix rows correspond to: p0, v0, pf, vf
            B = [p0; v0; pf; vf]; 
            % linsolve returns column vector coefficents
            coefficients = linsolve(A, B);
        end
        
        % solves for a quintic polynomial trajectory between two via-points
        % INPUT:
        % Starting and ending times t0 and tf (in seconds), starting and ending 
        % velocities, and starting and ending positions, starting and ending 
        % accelerations
        % 
        % OUTPUT:
        % 6-by-1 array of coefficients for the quintic polynomial.
        function coefficients = quintic_traj(self, t0, tf, a0, af, v0, vf, p0, pf)
             % Generic Quintic function coefficient matrix
            A = [1 t0 t0^2 t0^3 t0^4 t0^5;
                 0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
                 0 0 2 6*t0 12*t0^2 20*t0^3;
                 1 tf tf^2 tf^3 tf^4 tf^5;
                 0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
                 0 0 2 6*tf 12*tf^2 20*tf^3];
            % coefficient matrix rows correspond to: p0, v0, a0, pf, vf, af
            B = [p0; v0; a0; pf; vf; af];
            % linsolve returns column vector coefficents
            coefficients = linsolve(A, B);
        end
    end
end