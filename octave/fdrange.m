load -ascii fdrange_data.txt
close all;

for i=2:columns(fdrange_data)
	x = fdrange_data(fdrange_data(:, i) > 0, i);
	y = fdrange_data(1:rows(x), 1);
	invy = power(y, -1);

	p = polyfit(x, invy, 1);

	x_pred = linspace(min(x)-200, max(x)+200);
	y_pred = power(polyval(p, x_pred), -1);

	figure();
	plot(x_pred, y_pred, 'g', x, y, 'b');

	p
end
