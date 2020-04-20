function [omega, accel, gps, sig, gt] = getZurichData()
%% Reads in the Zurich Urban IMU, Gyro, and GPS data interpolating time stamps

    % general comments, this data set has some issues with units.
    % Their ground truth
    data_root = 'AGZ_subset';
    GPSfile =  fullfile(data_root, 'Log Files/OnboardGPS.csv');
    IMUfile = fullfile(data_root, 'Log Files/RawAccel.csv');
    Gyrofile = fullfile(data_root, 'Log Files/RawGyro.csv');
    GTfile = fullfile(data_root, 'Log Files/GroundTruthAGL.csv');

    
    %%    
    % The following (commented) section should give us the xyz coordinates of
    % the latitude/longitude/altitude but doesn't quite line up with the "true
    % pose" gps data - so we use that instead.

    % d1 = readmatrix(GPSfile);
    % tstart = d1(1,1);
    % t1 = (d1(:,1)-tstart)/10^6;
    % x1 = d1(:,3); y1 = d1(:,4); z1 = d1(:,5);
    % r = 6.3781*10^6;
    % y = r*sin(x1*pi/180).*cos(y1*pi/180); y = y-y(1);
    % x = r*sin(x1*pi/180).*sin(y1*pi/180); x = x-x(1);
    % z = z - z(1);

    %% Get Gyro data
    d2 = readmatrix(Gyrofile);
    t = d2(:,1); tstart = t(1);
    t = (t-tstart)/10^6;
    omega = makeDataStruct(t,d2,[3,4,5],tstart);

    %% Get IMU data
    d3 = readmatrix(IMUfile);
    accel = makeDataStruct(t,d3,[3,4,5],tstart);

    %% Get Ground truth
    d4 = readmatrix(GTfile);
    xgt = d4(:,2); xgt = xgt-xgt(1);
    ygt = d4(:,3); ygt = ygt-ygt(1);
    zgt = d4(:,4); zgt = zgt-zgt(1);
    % Set our ground truth to reference from 0,0,0

    
    t4 = linspace(0,t(end),length(d4));
    x4 = interp1(t4, xgt, t)';
    y4 = interp1(t4, ygt, t)';
    z4 = interp1(t4, zgt, t)';
    %The z ground truth starts a
    gt.x = x4; gt.y = y4; gt.z = z4;

    xgps = d4(:,8) - d4(1,8);
    ygps = d4(:,9) - d4(1,9);
    zgps = d4(:,10)- d4(1,10);
    % Set our ground truth to reference from 0,0,0
    
    % interpolate gps data for simplicity so we have IMU,Gyro, and GPS
    % readings at the same time steps. We may want to change this later to
    % update only when there's a new measurement, but this seems like a
    % good place to start
    xgps = interp1(t4, xgps, t)';
    ygps = interp1(t4, ygps, t)';
    zgps = interp1(t4, zgps, t)';
    gps.x = xgps; gps.y = ygps; gps.z = zgps;
    
    err = abs([xgps-x4; ygps-y4; zgps-z4])';   % error over time
    sig = cov(err); %Covariance of GPS data
    
    %% Plot initial results
    plotresults = 1; %Can toggle this, I usually set this to be 0
    if plotresults
        figure(1); cla; hold on;
        N = length(xgps);
        v = round(linspace(1,N,100));
        for i = 1:99
            plot3(xgps(v(i):v(i+1)), ygps(v(i):v(i+1)), zgps(v(i):v(i+1)),'ro');
            plot3(x4(v(i):v(i+1)), y4(v(i):v(i+1)), z4(v(i):v(i+1)),'k.');
        end

        figure(2);
        subplot(3,1,1);
        plot(err(:,1)); title('Error in x');
        subplot(3,1,2); 
        plot(err(:,2)); title('Error in y');
        subplot(3,1,3);
        plot(err(:,3)); title('Error in z');
    end
end

%% Helper function to create structs for accel and omega
function D = makeDataStruct(t1,initialDataMat,indeces,tstart)
    ti = (initialDataMat(:,1)-tstart)/10^6;
    d = zeros(length(t1),3);
    
    for j = 1:length(indeces)
        i = indeces(j);
        di = initialDataMat(:,i);
        intepw = interp1(ti,di,t1);
        d(:,j) = intepw;
    end
    D.x = d(:,1);
    D.y = d(:,2);
    D.z = d(:,3);
    D.t = ti;
end