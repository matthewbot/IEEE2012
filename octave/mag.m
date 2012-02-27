load -ascii data.txt;

minx = min(data(:, 1));
maxx = max(data(:, 1));
miny = min(data(:, 2));
maxy = max(data(:, 2));

x_offset = (maxx + minx)/2
y_offset = (maxy + miny)/2
y_scale = (maxx - minx)/(maxy - miny)

data_cal = data;
data_cal(:, 1) -= x_offset;
data_cal(:, 2) -= y_offset;
data_cal(:, 2) *= y_scale;

angles = atan2(data_cal(:, 2), data_cal(:, 1));

figure(1);
plot(data(:, 1), data(:, 2));
title("Raw data");

figure(2);
plot(data_cal(:, 1), data_cal(:, 2));
title("Calibrated data");

figure(3);
plot(angles);


