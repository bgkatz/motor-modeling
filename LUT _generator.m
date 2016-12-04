%%% Generate PMT-based lookup table %%%

speed = data(:,1);
cmd = data(:,2);
phase = data(:,4);
torque = data(:,5);
figure;scatter3(data(:,1), data(:,2), data(:,5), 15, data(:,5))

speedvec = [speed(find(diff(speed))); speed(end)];