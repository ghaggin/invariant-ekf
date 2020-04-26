classdef EKF < handle
    properties
        mu;         % pose = [x;v;theta] - 9*1
        Sigma;
        Q_w_mat;    % gyro noise
        Q_a_mat;    % acc noise
        mu_sym;
        F_sym;
        W_sym;
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
            
            [obj.Q_w_mat, obj.Q_a_mat, obj.V] = deal(eye(3) .* 0.01);
            
            syms x [3, 1]
            syms v [3, 1]
            syms t [3, 1]
            syms w [3, 1]
            syms a [3, 1]
            syms dt

            rotm = obj.rot_mat(t);
            
            obj.mu_sym = [x + rotm * v * dt; v + rotm * a * dt; t + w * dt];
            obj.F_sym = jacobian(obj.mu_sym, [x; v; t]);
            obj.W_sym = jacobian(obj.mu_sym, [w; a]);
            
        end
        
        %------------------------------------------------------------------
        
        function prediction(obj, w, a, dt)
            F_mat = obj.eval_F(obj.mu, w, a, dt);
            W_mat = obj.eval_W(obj.mu, w, a, dt);
            
            obj.Sigma = F_mat * obj.Sigma * F_mat' + ...
                W_mat * blkdiag(obj.Q_w_mat, obj.Q_a_mat) * W_mat';
            
            %Propoagate mean through non-linear dynamics
            obj.mu = obj.eval_mu(obj.mu, w, a, dt);
        end
        
        %------------------------------------------------------------------
        
        function correction(obj, z)
            % TODO
        end
        
        %------------------------------------------------------------------
        
        function rotm = rot_mat(~, euler_angle)
            r = euler_angle(1);
            p = euler_angle(2);
            y = euler_angle(3);
            
            % http://planning.cs.uiuc.edu/node102.html
            rx = [1, 0, 0; 0, cos(r), -sin(r); 0, sin(r), cos(r)];
            ry = [cos(p), 0, sin(p); 0, 1, 0; -sin(p), 0, cos(p)];
            rz = [cos(y), -sin(y), 0; sin(y), cos(y), 0; 0, 0, 1];
            rotm = rz * ry * rx;
        end
        
        %------------------------------------------------------------------
        
        function out = eval_mu(obj, mu, w, a, dt) %#ok<INUSD>
            x1 = mu(1); x2 = mu(2); x3 = mu(3);   %#ok<NASGU>
            v1 = mu(4); v2 = mu(5); v3 = mu(6);   %#ok<NASGU>
            t1 = mu(7); t2 = mu(8); t3 = mu(9);   %#ok<NASGU>
            w1 = w(1); w2 = w(2); w3 = w(3);      %#ok<NASGU>
            a1 = a(1); a2 = a(2); a3 = a(3);      %#ok<NASGU>
            
            out = subs(obj.mu);
        end
        
        function out = eval_F(obj, mu, w, a, dt) %#ok<INUSD>
            x1 = mu(1); x2 = mu(2); x3 = mu(3);  %#ok<NASGU>
            v1 = mu(4); v2 = mu(5); v3 = mu(6);  %#ok<NASGU>
            t1 = mu(7); t2 = mu(8); t3 = mu(9);  %#ok<NASGU>
            w1 = w(1); w2 = w(2); w3 = w(3);     %#ok<NASGU>
            a1 = a(1); a2 = a(2); a3 = a(3);     %#ok<NASGU>
            
            out = subs(obj.F_sym);
        end
        
        function out = eval_W(obj, mu, w, a, dt) %#ok<INUSD>
            x1 = mu(1); x2 = mu(2); x3 = mu(3);  %#ok<NASGU>
            v1 = mu(4); v2 = mu(5); v3 = mu(6);  %#ok<NASGU>
            t1 = mu(7); t2 = mu(8); t3 = mu(9);  %#ok<NASGU>
            w1 = w(1); w2 = w(2); w3 = w(3);     %#ok<NASGU>
            a1 = a(1); a2 = a(2); a3 = a(3);     %#ok<NASGU>
            
            out = subs(obj.W_sym);
        end
    end
end

