%% Essentially the "main" function, runs the left-InEKF filter on the Zurich Urban dataset

%Clear all variables and close all figures
clear; close all;

%Initialize filter and data
filter = LIEKF();
[omega,accel,gps,gps_sig,gt] = getZurichData();
N = length(accel.x);

PATH_FIG = 1;
% X = cell(length(omega.x));
X = zeros(3,length(omega.x));
plot_progress = 0; %If you want incremental plotting turn this on
skip = 100; %Number of steps to skip - set to 1 to see every step
for i = 2:N-1

    % Prediction step
    w = [omega.x(i); omega.y(i); omega.z(i)];
    a = [accel.x(i); accel.y(i); accel.z(i)];
    dt = omega.t(i)-omega.t(i-1);
    filter.prediction(w,a,dt);
    
    % Correction step
    obs = [gps.x(i), gps.y(i), gps.z(i)];
    filter.correction(obs);
    
    % 
    if(mod(i,skip)==0)
        fprintf('iteration: %d/%d\n',i,N);
        if(plot_progress)
            p = filter.mu(1:3,5);
            plot3(p(1),p(2),p(3),'r.'); hold on;
            plot3(gt.x(i), gt.y(i), gt.z(i),'.b') %'Color', [0.5,0.5,0.5]);
            plot3(gps.x(i), gps.y(i), gps.z(i),'.k')
        end
    end
    
    %Save results
    X(:,i) = filter.mu(1:3,5);
end

%% Plot Results
figure(2);
subplot(3,1,1)
plot(omega.t,[gps.x;X(1,:);gt.x]); hold on;
legend('GPS','Estimate', 'GT');

subplot(3,1,2)
plot(omega.t,[gps.y;X(2,:);gt.y]); hold on;
legend('GPS','Estimate', 'GT');

subplot(3,1,3)
plot(omega.t,[gps.z;X(3,:);gt.z]); hold on;
legend('GPS','Estimate', 'GT');

figure(3);
hold on;
plot3(gps.x,gps.y,gps.z, 'g.'); 
plot3(X(1,:),X(2,:),X(3,:), 'k.');
plot3(gt.x, gt.y, gt.z, 'r.');
legend('GPS','Estimate', 'GT');
axis('equal');