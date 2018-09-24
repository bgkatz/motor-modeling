tdot = data.tdot;
current = data.v;
torque = data.torque;
phi = data.phi;

is_torque = sum(torque, 1)>0;
max_speed_ind = max(find(is_torque))-1;
torque = torque(:, 1:max_speed_ind);
current = current(:, 1:max_speed_ind);
tdot = tdot(:, 1:max_speed_ind);
phi = phi(:, 1:max_speed_ind);

speed_intervals = 121;
pmt_intervals = 50;

speed_ind = 1:1:length(tdot(1,:));
[max_torque, i] = max(torque, [], 1);
speed = [0, tdot(1,:)];
ind = sub2ind(size(torque), i, speed_ind);
pmt = linspace(0, torque(1), pmt_intervals);

[torque_grid, pmt_grid] = meshgrid(max_torque, pmt);
speed_grid = meshgrid(tdot(1,:), pmt);
mt_grid = min(pmt_grid, torque_grid);
pmt_phase_grid = zeros(size(mt_grid));
pmt_current_grid = zeros(size(mt_grid));


for x = 1:length(max_torque)
    
    torque_vec = torque(i(x):end, x);
    [min_torque, i_min] = min(torque_vec);
    torque_vec = torque_vec(1:i_min-1);
    current_vec = current(i(x):i(x)+i_min-1, x);
    phase_vec = phi(i(x):i(x)+i_min-1, x);

    pmt_torque = mt_grid(:,x);
    pmt_current = interp1(torque_vec, current_vec, pmt_torque, 'pchip');
    pmt_phase = interp1(torque_vec, phase_vec, pmt_torque, 'pchip');
    
    pmt_phase_grid(:,x) = pmt_phase;
    pmt_current_grid(:,x) = pmt_current;
    
end

i_d_grid = pmt_current_grid.*cos(pmt_phase_grid);
i_q_grid = pmt_current_grid.*sin(pmt_phase_grid);

figure;surf(speed_grid, pmt_grid, mt_grid); title('Torque'); xlabel('Speed (rad/s)'); ylabel('Torque Command');
figure;surf(speed_grid, pmt_grid, pmt_phase_grid); title('Phase'); xlabel('Speed (rad/s)'); ylabel('Torque Command');
figure;surf(speed_grid, pmt_grid, pmt_current_grid); title('Current'); xlabel('Speed (rad/s)'); ylabel('Torque Command');
figure;surf(speed_grid, pmt_grid, i_d_grid); title('I_D'); xlabel('Speed (rad/s)'); ylabel('Torque Command');
figure;surf(speed_grid, pmt_grid, i_q_grid); title('I_Q'); xlabel('Speed (rad/s)'); ylabel('Torque Command');

pmt_lut.speed = speed_grid;
pmt_lut.pmt = pmt_grid;
pmt_lut.torque = mt_grid;
pmt_lut.current = pmt_current_grid;
pmt_lut.phase = pmt_phase_grid;
pmt_lut.i_d = i_d_grid;
pmt_lut.i_q = i_q_grid;
pmt_lut.i_q(1,:) = zeros(size(pmt_lut.i_q(1,:)));

save('Data/EX8/FW_lookup_40A.mat', 'pmt_lut');

pqtab = pmt_lut.i_q;
pdtab = pmt_lut.i_d;
zqtab = pmt_lut.i_q(1,:);
zdtab = pmt_lut.i_d(1,:);