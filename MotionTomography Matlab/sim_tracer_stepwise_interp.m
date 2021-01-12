function xx = sim_tracer_stepwise_interp(r0, S, F, N, dt)
    xx = zeros(N,2)*NaN;
    xx(1,:) = r0;
    for n = 2:N
        % Find cell flow for current position
        Fcell = F.get(floor(xx(n-1,1)),floor(xx(n-1,2)));

        xx(n,:) = xx(n-1,:) + S*dt + Fcell*dt;
    end
end