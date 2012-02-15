function [fd_p, bk_p, fd, bk] = motorcurve(data)
	count = rows(data);
	data(:, 1) = data(:, 1) / .01 / 1024; # All encoder values were taken over 10 ms timesteps
	
	fd = data(2:count/2-1, :);
	bk = data(count/2+2:count, :);

	fd_good = fd(500:rows(fd)-100, :);
	bk_good = bk(500:rows(bk)-100, :);
	
	fd_p = polyfit(fd_good(:, 1), fd_good(:, 2), 1);
	bk_p = polyfit(bk_good(:, 1), bk_good(:, 2), 1);
end
