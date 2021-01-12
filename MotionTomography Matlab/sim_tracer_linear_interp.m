function xx = sim_tracer_linear_interp(r0, S, F, N, dt)
    xx = zeros(N,2)*NaN;
    xx(1,:) = r0;
    for n = 2:N
        % Locate point
        x = xx(n-1,1);
        y = xx(n-1,2);
        
        % Locate 4 neighbors
        a = [floor(x-.5), floor(y-.5)];
        b = [floor(x-.5), ceil(y-.5+1.e-10)];
        c = [ceil(x-.5+1.e-10), floor(y-.5)];
        d = [ceil(x-.5+1.e-10), ceil(y-.5+1.e-10)];
        
        % get values at neighbors, if exist
        if ~F.isempty(a(1),a(2))
            a_val = F.get(a(1),a(2));
        elseif ~F.isempty(b(1),b(2))
            a_val = F.get(b(1),b(2));
        elseif ~F.isempty(c(1),c(2))
            a_val = F.get(c(1),c(2));
        else
            a_val = F.get(d(1),d(2));
        end

        if ~F.isempty(b(1),b(2))
            b_val = F.get(b(1),b(2));
        elseif ~F.isempty(a(1),a(2))
            b_val = F.get(a(1),a(2));
        elseif ~F.isempty(d(1),d(2))
            b_val = F.get(d(1),d(2));
        else
            b_val = F.get(c(1),c(2));
        end

        if ~F.isempty(c(1),c(2))
            c_val = F.get(c(1),c(2));
        elseif ~F.isempty(a(1),a(2))
            c_val = F.get(a(1),a(2));
        elseif ~F.isempty(d(1),d(2))
            c_val = F.get(d(1),d(2));
        else
            c_val = F.get(b(1),b(2));
        end

        if ~F.isempty(d(1),d(2))
            d_val = F.get(d(1),d(2));
        elseif ~F.isempty(c(1),c(2))
            d_val = F.get(c(1),c(2));
        elseif ~F.isempty(b(1),b(2))
            d_val = F.get(b(1),b(2));
        else
            d_val = F.get(a(1),a(2));
        end
        
        % intep between pairs
        mid_ab = (b(2)-y+.5)*a_val + (y-.5-a(2))*b_val;
        mid_cd = (d(2)-y+.5)*c_val + (y-.5-c(2))*d_val;
        Fhere = (c(1)-x+.5)*mid_ab + (x-.5-a(1))*mid_cd;
        
        % Advance trajectory
        xx(n,:) = xx(n-1,:) + S*dt + Fhere*dt;
    end
end