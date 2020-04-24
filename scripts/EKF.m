classdef EKF < handle
    properties
        mu;         % pose = [x;v;theta] - 9*1
        Sigma;
        cov_g;      %gyro noise
        cov_a;      %acc noise
        V;          %observation noise of position
    end
    
    methods
        function obj = EKF(theta0, p0, v0)
            if nargin == 0
                theta0 = zeros(3,1);
                p0 = zeros(3,1);
                v0 = zeros(3,1);
            end
            
            obj.mu = [p0; v0; theta0];
            obj.Sigma = eye(9);
            
            [obj.cov_g, obj.cov_a, obj.V] = deal(eye(3));
        end
        
        function rotm = rot_mat(euler_angle)
            r = euler_angle(1);
            p = euler_angle(2);
            y = euler_angle(3);
            
            % http://planning.cs.uiuc.edu/node102.html
            rx = [1, 0, 0; 0, cos(r), -sin(r); 0, sin(r), cos(r)];
            ry = [cos(p), 0, sin(p); 0, 1, 0; -sin(p), 0, cos(p)];
            rz = [cos(y), -sin(y), 0; sin(y), cos(y), 0; 0, 0, 1];
            rotm = rz * ry * rx;
        end
        
        function prediction(obj, w, a, dt)
          
            %Propoagate mean through non-linear dynamics
            
            %a = a + [0; 0; -9.81];
            x = obj.mu(1:3) + rot_mat(obj.mu(4:6)) * dt + ...
                1 / 2 * rot_mat(obj.mu(4:6)) * a * dt;
            v = obj.mu(4:6) + rot_mat(obj.mu(4:6)) * a * dt;
            theta = obj.mu(7:9) + w;
            obj.mu = [x; v; theta];
            
            %Update covariance with G,V and M(u)
            obj.Sigma_pred = 0; %TODO
        end
        
        function correction(obj, z)
            % TODO
        end
    end
end

