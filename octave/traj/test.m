amax = -2;
vmax = -6;
h = -20;
v0 = 0;
v1 = 0;
N = 1000;
[ts, vs, T] = traj(h, v0, v1, vmax, amax, N);
plot(ts, vs);
sum(vs*T/N)
