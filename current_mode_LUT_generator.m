%%% Uses a different motor_fun, which doesn't rely on a controller
%%% converging - super fast

format compact
clear all
%%% Generate a Percent Max Torque lookup table %%%

n = 100;  % Phase resolution
n2 = 100; % Current intervals
n3 = 100; % Velocity intervals
tdot_max = 2500;
mag_max = sqrt(3/2)*200;
v_max = 160/(sqrt(2));
tdot = linspace(0.01, tdot_max, n3)';

phase_min = pi/2;
phase_max = pi;
tol = (phase_max - phase_min)/n;
%phase_max = 2*pi;

pmi = linspace(1, 0, n2);   %Percent Max current.  Easier to generate table this way, then interpolate torques

v_vec = linspace(1, sqrt(3/2)*200, n2);
%phase = linspace(phase_min, phase_max, n);

v_vec = zeros(n3, 1);
v_log_vec = zeros(n2, n3);
phi_max_vec = zeros(n2, n3);
mag_max_vec = zeros(n2, n3);
v_max_vec = zeros(n2, n3);
tdot_vec = zeros(n2, n3);
torque_max_vec = zeros(n2, n3);
tdot_vec = zeros(n2, n3);
i_mag_vec =  zeros(n2, n3);
percent = zeros(n2, n3);

%{
%%% Find max achievable current and pos/neg torques vs speed %%%    parfor k = 1:n3
for k = 1:n3
    tic
    phi = linspace(phase_min, phase_max, 5)';
    delta = phi(2) - phi(1);
    torque = zeros(5, 1);
    v = zeros(5, 1);
    parfor x = 1:5
        [torque(x), v(x)] = motor_fun_current_mode(mag_max, phi(x), tdot(k));
    end
    tmax = max(torque);
    ind_max = find(torque==tmax);
    ind_max = ind_max(1);
    ind = [ind_max-1; ind_max; ind_max+1];
    while(delta > tol)
        delta = delta/2;

        if(ind(3) > length(phi))
            phi = circshift(phi, -1);
            torque = circshift(torque, -1);
            v = circshift(v, -1);
            ind = ind-1;
        elseif (ind(1)==0)
            phi = circshift(phi, 1);
            torque = circshift(torque, 1);
            v = circshift(v, 1);
            ind = ind+1;
        end

        phi = [phi(ind(1)); phi(ind(1))+delta; phi(ind(2)); phi(ind(2))+delta; phi(ind(3))];
        [t1, v1] = motor_fun_current_mode(mag_max, phi(2), tdot(k));
        [t2, v2] = motor_fun_current_mode(mag_max, phi(4), tdot(k));
        torque = [torque(ind(1));t1; torque(ind(2)); t2; torque(ind(3))];
        v = [v(ind(1)); v1; v(ind(2)); v2; v(ind(3))];
        tmax = max(torque);
        ind_max = find(torque==tmax);
        ind_max = ind_max(1);
        ind = [ind_max-1; ind_max; ind_max+1];
        %torque_vec = [torque_vec;torque];
        %phase_vec = [phase_vec;phase];
        torque_max_vec(1, k) = max(torque);
        phi_max_vec(1, k) = phi(ind_max);
        v_max_vec(1, k) = v(ind_max);
        tdot_vec(1, k) = tdot(k);
    end   
    toc
    k
end

max_current = v_max_vec(1,:);
max_current = min(max_current, mag_max);

figure;plotyy(tdot_vec(1,:), torque_max_vec(1,:), tdot_vec(1,:), phi_max_vec(1,:));
%}
parfor j = 1:n2
    %tic
    for k = 1:n3
        %tic
        phi = linspace(phase_min, phase_max, n)';
        delta = phi(2) - phi(1);
        torque = zeros(n, 1);
        v = zeros(n, 1);
        
        for x = 1:n
            [torque(x), v(x)] = motor_fun_current_mode(mag_max*pmi(j), phi(x), tdot(k));
        end
        is_valid = v<v_max;
        tmax = max(torque.*is_valid);
        if(tmax>0)
            ind_max = find(torque==tmax);
        else
            torque = 0;
            ind_max = 1;
        end
        ind_max = ind_max(1);
        %ind = [ind_max-1; ind_max; ind_max+1];
        %{
        while(delta > tol)
            delta = delta/2;

            if(ind(3) > length(phi))
                phi = circshift(phi, -1);
                torque = circshift(torque, -1);
                v = circshift(v, -1);
                ind = ind-1;
            elseif (ind(1)==0)
                phi = circshift(phi, 1);
                torque = circshift(torque, 1);
                v = circshift(v, 1);
                ind = ind+1;
            end

            phi = [phi(ind(1)); phi(ind(1))+delta; phi(ind(2)); phi(ind(2))+delta; phi(ind(3))];
            [t1, v1] = motor_fun_current_mode(mag_max*pmi(j), phi(2), tdot(k));
            [t2, v2] = motor_fun_current_mode(mag_max*pmi(j), phi(4), tdot(k));
            torque = [torque(ind(1));t1; torque(ind(2)); t2; torque(ind(3))];
            v = [v(ind(1)); v1; v(ind(2)); v2; v(ind(3))];
            tmax = max(torque);
            ind_max = find(torque==tmax);
            ind_max = ind_max(1);
            ind = [ind_max-1; ind_max; ind_max+1];
            %torque_vec = [torque_vec;torque];
            %phase_vec = [phase_vec;phase];
            torque_max_vec(j, k) = max(torque);
            phi_max_vec(j, k) = phi(ind_max);
            v_max_vec(j, k) = v(ind_max);
            mag_max_vec(j, k) = mag_max*pmi(j);%mag(ind_max);
            tdot_vec(j, k) = tdot(k);
        end 
           %}
            torque_max_vec(j, k) = tmax;
            phi_max_vec(j, k) = phi(ind_max);
            v_max_vec(j, k) = v(ind_max);
            mag_max_vec(j, k) = mag_max*pmi(j);%mag(ind_max);
            tdot_vec(j, k) = tdot(k);
    %toc    
    end
    
    %j
    
end

is_valid = v_max_vec < v_max;
%toc

%figure;plot(tdot, torque_max_vec(1,:)); title('Max Torque')
%figure;plot(tdot, phi_max_vec(1,:));  title('Max Phase')
%figure;plot(tdot, mag_max_vec(1,:)); title('Max Current')
figure; surf(tdot_vec, mag_max_vec, torque_max_vec.*is_valid);
figure;surf(tdot_vec, mag_max_vec, phi_max_vec);
% figure;plot(tdot, i_mag_max_vec); title('Max Current')%figure;plot(tdot, t_max_vec, tdot, t_min_vec); title('Max/Min Torque')
%figure;plot(tdot, phi_max_vec, tdot, phi_min_vec);  title('Max/Min Phase')
%figure;plot(tdot, i_mag_max_vec, tdot, i_mag_min_vec); title('Max/Min Current')

%figure; scatter(phase_vec, torque_vec);
data.tdot = tdot_vec;
data.v = mag_max_vec;
data.torque = torque_max_vec;
data.phi = phi_max_vec;
save('data5.mat', 'data');
