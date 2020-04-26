classdef EKF < handle
    properties
        mu;         % pose = [x;v;theta] - 9*1
        Sigma;
        Q_w_mat;    % gyro noise
        Q_a_mat;    % acc noise
        %mu_sym;
        %F_sym;
        %W_sym;
        V;          %observation noise of position
        mu_lam;
        F_lam;
        W_lam;
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
            
            %{
            syms x [3, 1]
            syms v [3, 1]
            syms t [3, 1]
            syms w [3, 1]
            syms a [3, 1]
            syms dt

            rotm = obj.rot_mat(t);
            
            obj.mu_sym = [x + rotm * v * dt; v + rotm * a * dt + [0;0;-9.81] * dt; t + w * dt];
            obj.F_sym = jacobian(obj.mu_sym, [x; v; t]);
            obj.W_sym = jacobian(obj.mu_sym, [w; a]);
            %}
            
            obj.mu_lam = @(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt) [ ...
                x1 + dt*(v3*(sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2)) - v2*(cos(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2)) + v1*cos(t2)*cos(t3));
                x2 + dt*(v2*(cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3)) - v3*(cos(t3)*sin(t1) - cos(t1)*sin(t2)*sin(t3)) + v1*cos(t2)*sin(t3));
                x3 + dt*(v3*cos(t1)*cos(t2) - v1*sin(t2) + v2*cos(t2)*sin(t1));
                v1 + dt*(a3*(sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2)) - a2*(cos(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2)) + a1*cos(t2)*cos(t3));
                v2 + dt*(a2*(cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3)) - a3*(cos(t3)*sin(t1) - cos(t1)*sin(t2)*sin(t3)) + a1*cos(t2)*sin(t3));
                v3 - (981*dt)/100 + dt*(a3*cos(t1)*cos(t2) - a1*sin(t2) + a2*cos(t2)*sin(t1));
                t1 + dt*w1;
                t2 + dt*w2;
                t3 + dt*w3];
            
            obj.F_lam = @(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt) [ ...
                1, 0, 0, dt*cos(t2)*cos(t3), -dt*(cos(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2)),  dt*(sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2)),  dt*(v2*(sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2)) + v3*(cos(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2))), dt*(v3*cos(t1)*cos(t2)*cos(t3) - v1*cos(t3)*sin(t2) + v2*cos(t2)*cos(t3)*sin(t1)), -dt*(v2*(cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3)) - v3*(cos(t3)*sin(t1) - cos(t1)*sin(t2)*sin(t3)) + v1*cos(t2)*sin(t3));
                0, 1, 0, dt*cos(t2)*sin(t3),  dt*(cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3)), -dt*(cos(t3)*sin(t1) - cos(t1)*sin(t2)*sin(t3)), -dt*(v2*(cos(t3)*sin(t1) - cos(t1)*sin(t2)*sin(t3)) + v3*(cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3))), dt*(v3*cos(t1)*cos(t2)*sin(t3) - v1*sin(t2)*sin(t3) + v2*cos(t2)*sin(t1)*sin(t3)),  dt*(v3*(sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2)) - v2*(cos(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2)) + v1*cos(t2)*cos(t3));
                0, 0, 1,        -dt*sin(t2),                              dt*cos(t2)*sin(t1),                              dt*cos(t1)*cos(t2),                                                          dt*(v2*cos(t1)*cos(t2) - v3*cos(t2)*sin(t1)),                        -dt*(v1*cos(t2) + v3*cos(t1)*sin(t2) + v2*sin(t1)*sin(t2)),                                                                                                                          0;
                0, 0, 0,                  1,                                               0,                                               0,  dt*(a2*(sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2)) + a3*(cos(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2))), dt*(a3*cos(t1)*cos(t2)*cos(t3) - a1*cos(t3)*sin(t2) + a2*cos(t2)*cos(t3)*sin(t1)), -dt*(a2*(cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3)) - a3*(cos(t3)*sin(t1) - cos(t1)*sin(t2)*sin(t3)) + a1*cos(t2)*sin(t3));
                0, 0, 0,                  0,                                               1,                                               0, -dt*(a2*(cos(t3)*sin(t1) - cos(t1)*sin(t2)*sin(t3)) + a3*(cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3))), dt*(a3*cos(t1)*cos(t2)*sin(t3) - a1*sin(t2)*sin(t3) + a2*cos(t2)*sin(t1)*sin(t3)),  dt*(a3*(sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2)) - a2*(cos(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2)) + a1*cos(t2)*cos(t3));
                0, 0, 0,                  0,                                               0,                                               1,                                                          dt*(a2*cos(t1)*cos(t2) - a3*cos(t2)*sin(t1)),                        -dt*(a1*cos(t2) + a3*cos(t1)*sin(t2) + a2*sin(t1)*sin(t2)),                                                                                                                          0;
                0, 0, 0,                  0,                                               0,                                               0,                                                                                                     1,                                                                                 0,                                                                                                                          0;
                0, 0, 0,                  0,                                               0,                                               0,                                                                                                     0,                                                                                 1,                                                                                                                          0;
                0, 0, 0,                  0,                                               0,                                               0,                                                                                                     0,                                                                                 0,                                                                                                                          1];
            
            obj.W_lam = @(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt) [ ...
                 0,  0,  0,                  0,                                               0,                                               0;
                 0,  0,  0,                  0,                                               0,                                               0;
                 0,  0,  0,                  0,                                               0,                                               0;
                 0,  0,  0, dt*cos(t2)*cos(t3), -dt*(cos(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2)),  dt*(sin(t1)*sin(t3) + cos(t1)*cos(t3)*sin(t2));
                 0,  0,  0, dt*cos(t2)*sin(t3),  dt*(cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3)), -dt*(cos(t3)*sin(t1) - cos(t1)*sin(t2)*sin(t3));
                 0,  0,  0,        -dt*sin(t2),                              dt*cos(t2)*sin(t1),                              dt*cos(t1)*cos(t2);
                dt,  0,  0,                  0,                                               0,                                               0;
                 0, dt,  0,                  0,                                               0,                                               0;
                 0,  0, dt,                  0,                                               0,                                               0];

        end
        
        %------------------------------------------------------------------
        
        function prediction(obj, w, a, dt)
            F_mat = obj.eval_F(obj.mu, w, a, dt);
            W_mat = obj.eval_W(obj.mu, w, a, dt);
            
            obj.Sigma = F_mat * obj.Sigma * F_mat' + ...
                W_mat * blkdiag(obj.Q_w_mat, obj.Q_a_mat) * W_mat';
            
            %Propoagate mean through non-linear dynamics
            obj.mu = obj.eval_mu(obj.mu, w, a, dt);
            obj.mu(7:9) = wrapToPi(obj.mu(7:9)); % so sad
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
        
        function out = eval_mu(obj, mu, w, a, dt)
            x1 = mu(1); x2 = mu(2); x3 = mu(3);
            v1 = mu(4); v2 = mu(5); v3 = mu(6);
            t1 = mu(7); t2 = mu(8); t3 = mu(9);
            w1 = w(1); w2 = w(2); w3 = w(3);
            a1 = a(1); a2 = a(2); a3 = a(3);
            
            out = obj.mu_lam(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt);
            %double(subs(obj.mu_sym));
        end
        
        function out = eval_F(obj, mu, w, a, dt)
            x1 = mu(1); x2 = mu(2); x3 = mu(3);
            v1 = mu(4); v2 = mu(5); v3 = mu(6);
            t1 = mu(7); t2 = mu(8); t3 = mu(9);
            w1 = w(1); w2 = w(2); w3 = w(3);
            a1 = a(1); a2 = a(2); a3 = a(3);
            
            out = obj.F_lam(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt);
            %double(subs(obj.F_sym));
        end
        
        function out = eval_W(obj, mu, w, a, dt)
            x1 = mu(1); x2 = mu(2); x3 = mu(3);
            v1 = mu(4); v2 = mu(5); v3 = mu(6);
            t1 = mu(7); t2 = mu(8); t3 = mu(9);
            w1 = w(1); w2 = w(2); w3 = w(3);
            a1 = a(1); a2 = a(2); a3 = a(3);
            
            out = obj.W_lam(x1, x2, x3, v1, v2, v3, t1, t2, t3, w1, w2, w3, a1, a2, a3, dt);
            %double(subs(obj.W_sym));
        end
    end
end

