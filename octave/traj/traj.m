function [ts, vs, T]=traj(h, v0, v1, vmax, amax, N)
  vlim = sqrt(h*amax + (v0^2 + v1^2)/2);
  if vlim >= vmax
	disp('case 1');
    Ta = (vmax - v0)/amax;
    Td = (vmax - v1)/amax;
    T = h/vmax + vmax/(2*amax)*(1 - v0/vmax)^2 + vmax/(2*amax)*(1-v1/vmax)^2;
	Vv = vmax;
  else
    disp('case 2');
    Ta = (vlim - v0)/amax;
    Td = (vlim - v1)/amax;
    T = Ta + Td;
    Vv = vlim;
  end
  
  vs = zeros(N, 1);
  ts = linspace(0, T, N);

  for i = 1:N
    if ts(i) < Ta
	  vs(i) = v0 + ((Vv - v0)/Ta)*ts(i);
	elseif ts(i) < T - Td
	  vs(i) = Vv;
	else
	  vs(i) = v1 + ((Vv - v1)/Td)*(T-ts(i));
	end
end
