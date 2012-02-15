load -ascii data.txt;

[left_fd_p, left_bk_p, left_fd, left_bk] = motorcurve([data(:, 2), data(:, 1)]);
[right_fd_p, right_bk_p, right_fd, right_bk] = motorcurve([data(:, 3), data(:, 1)]);

left_fd_fit = polyval(left_fd_p, left_fd(:, 1));
left_bk_fit = polyval(left_bk_p, left_bk(:, 1));
right_fd_fit = polyval(right_fd_p, right_fd(:, 1));
right_bk_fit = polyval(right_bk_p, right_bk(:, 1));

figure(1);
plot(left_fd(:, 1), left_fd(:, 2), 'r', left_bk(:, 1), left_bk(:, 2), 'b', left_fd(:, 1), left_fd_fit(:), 'g', left_bk(:, 1), left_bk_fit(:), 'g');

figure(2);
plot(right_fd(:, 1), right_fd(:, 2), 'r', right_bk(:, 1), right_bk(:, 2), 'b', right_fd(:, 1), right_fd_fit(:), 'g', right_bk(:, 1), right_bk_fit(:), 'g');

left_fd_p
left_bk_p

right_fd_p
right_bk_p
