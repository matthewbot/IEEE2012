load -ascii fdrange_data.txt

y = fdrange_data(:, 1);
x = fdrange_data(:, 3);
invy = power(y, -1);

p = polyfit(x, invy, 1);

x_pred = linspace(800, 4096);
y_pred = power(polyval(p, x_pred), -1);

plot(x_pred, y_pred, 'g', x, y, 'b');

p
