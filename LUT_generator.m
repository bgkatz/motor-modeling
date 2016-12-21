%%% Generate PMT-based lookup table %%%

speed = data.tdot;
phase = data.phi;
torque = data.torque;
mag = data.mag;
%figure;scatter3(data(:,1), data(:,2), data(:,5), 15, data(:,5))

speedvec = speed(1,:);
tmaxvec = torque(1,:);


%%% Create max torque vector %%%
for x = 1:length(speedvec)
    s = speedvec(x);
    tvec = torque(find(speed==s));
    tmax =  max(tvec);
    phi= phase(find((speed==s).*(torque==tmax)));
    tmax_vec(x) = tmax;
    phimax_vec(x) = phi;
end

%%% Percent Max Torque %%%
pmt  = [0:.01:1]';

[pmt_grid, speed_grid] = meshgrid(pmt, speedvec);
torque_grid = meshgrid(tmax_vec, pmt)';
torque_grid = pmt_grid.*torque_grid;
%figure; surf(speed_grid, pmt_grid, torque_grid); xlabel('Rad/s'); ylabel('PMT'); zlabel('Torque');

%phase_grid = zeros(length(speed_grid(:,1)), length(pmt_grid(1, :)));
%mag_grid = zeros(size(speed_grid(:,1)), size(pmt_grid(1, :)));
for x = 1:length(pmt_grid(1, :))
    for j = 1:length(speed_grid(:,1))
        t = torque_grid(j, x);
        s = speed_grid(j, x);
        
        t_ind = find(speed == s);
        tv = torque(t_ind);
        pv = phase(t_ind);
        mv = mag(t_ind);
        phi = interp1(tv, pv, t, 'pchip');
        m = interp1(tv, mv, t, 'pchip');
        
        phase_grid(j, x) = phi;
        mag_grid(j, x) = m;
        
        
    end
end

figure;surf(speed_grid, pmt_grid, phase_grid); xlabel('Rad/s'); ylabel('PMT'); zlabel('Phase');
figure;surf(speed_grid, pmt_grid, mag_grid); xlabel('Rad/s'); ylabel('PMT'); zlabel('Current');