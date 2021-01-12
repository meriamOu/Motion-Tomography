%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Georgia Tech Systems Research lab
% Demo Of Motion Tomography
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 
clear

% Choose interpolation mode %
stepwise_interp = true;

% Figure window setup %
if stepwise_interp
    ttext = 'MT Stepwise'
else
    ttext = 'MT Linear'
end
f = figure('name',ttext,'pos',[200 100 800 600]);
colors = get(gca,'colororder');
delete(gca)


% Number of algorithm iterations %
MAX_ITER = 20;
axlist = {1,MAX_ITER};
panlist = {1,MAX_ITER};

% Flow field %
Flow = CellStorage(50);

% Vehicles setup %
V(1).S = [-.1,.75;];        % Constant vehicle speed
V(1).T = 2;                 % Measured total time
V(1).r0 = [0.25,0];         % Measured starting position
V(1).rstar = [0.5,1.9];     % Measured ending position

V(2).S = [.8,.1;];          % Constant vehicle speed
V(2).T = 2;                 % Measured total time
V(2).r0 = [0,1.6];          % Measured starting position
V(2).rstar = [3,1.8];       % Measured ending position

% Trajectory simulation %
N = 10000; % Discretized time steps

% Computed Values -- DO NOT MANUALLY EDIT %
J = length(V); %num vehicles
T = max(V(:).T);
dt = T/N;
for j = 1:J
    V(j).d = [0,0];
    trajectory{j} = zeros(N,2);
    V(j).celltime = CellStorage(50);
end
wb.xmin = V(1).r0(1);
wb.xmax = V(1).r0(1);
wb.ymin = V(1).r0(2);
wb.ymax = V(1).r0(2);
for j = 1:J
    wb.xmin = floor(min([wb.xmin, V(j).r0(1), V(j).rstar(1)]));
    wb.xmax = ceil(max([wb.xmax, V(j).r0(1), V(j).rstar(1)]));
    wb.ymin = floor(min([wb.ymin, V(j).r0(2), V(j).rstar(2)]));
    wb.ymax = ceil(max([wb.ymax, V(j).r0(2), V(j).rstar(2)]));
end
wb.box = [wb.xmin-0.1 wb.xmax+0.1 wb.ymin-0.1 wb.ymax+0.1];
wb.dim = [wb.xmax-wb.xmin, wb.ymax-wb.ymin];


% MT algorithm iterations loop %
for i = 1:MAX_ITER
    % Time per cell %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j = 1:J % loop over Vehicle IDs
        xx = trajectory{j};

        % Time per cell
        V(j).celltime = CellStorage(50);
        V(j).totaltime = 0;
        losttime = 0;
        steps = floor(V(j).T/dt);
        for n = 1:steps
            % Increment cell time for current position
            x = floor(xx(n,1));
            y = floor(xx(n,2));
            V(j).celltime.set(x,y, V(j).celltime.get(x,y) + dt);
        end
    end
    

    % Update Flow %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    w = .5; %weighting parameter
    P = 1/J;
    for j = 1:J
        for x = wb.xmin:wb.xmax
            for y = wb.ymin:wb.ymax
                val = Flow.get(x,y) + w*P*V(j).d*V(j).celltime.get(x,y) / ...
                    sum(V(j).celltime.getall().^2);
                Flow.set(x,y, val);
            end
        end
    end

    
    % Setup axis for plotting %
    panlist{i} = uipanel(f,'Position',[.01 .11 .98 .88],...
                         'Tag','Panel with Sine Plot');
    axlist{i} = axes('Parent',panlist{i},'Position',[.1 .1 .8 .8]);
    axis equal
    grid on
    ax = axlist{i};%gca;
    ax.XAxis.TickValues = wb.xmin:wb.xmax;
    ax.YAxis.TickValues = wb.ymin:wb.ymax;
    axis(wb.box)
    hold on
    
    % Title %
    title(['Iteration ' num2str(i-1)])
        
    % Trace iteration i %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j = 1:J % loop over Vehicle IDs
        
        % Plot flow field %
        gridX = wb.xmin:wb.xmax;
        gridY = wb.ymin:wb.ymax;
        [mgX, mgY] = meshgrid(gridX,gridY);
        F = zeros(numel(mgX),2);
        for ii = 1:numel(mgX)
            F(ii,:) = Flow.get(mgX(ii),mgY(ii));
        end
        quiver(mgX(:)+0.5,mgY(:)+0.5,F(:,1),F(:,2),'Color',colors(3,:),'AutoScale','off')

        % Plot measured points %
        plot(V(j).r0(1),V(j).r0(2),'MarkerEdgeColor',colors(1,:),'Marker','square')
        plot(V(j).rstar(1),V(j).rstar(2),'MarkerEdgeColor',colors(2,:),'Marker','*')

        % Trajectory simulation %
        steps = floor(V(j).T/dt);
        if stepwise_interp
            xx = sim_tracer_stepwise_interp(V(j).r0, V(j).S, Flow, steps, dt);
        else
            xx = sim_tracer_linear_interp(V(j).r0, V(j).S, Flow, steps, dt);
        end

        % Plot trajectory %
        plot(xx(:,1),xx(:,2),'Color',colors(1,:))
        plot(xx(end,1),xx(end,2),'MarkerEdgeColor',colors(1,:),'Marker','o')
        % Plot vehicle velocity %
        quiver(V(j).r0(1),V(j).r0(2),V(j).S(1),V(j).S(2),'Color',colors(4,:),'LineStyle','-.','AutoScale','off')

        % MT error %
        V(j).d = V(j).rstar - xx(end,:);

        % Plot MT error %
        plot([V(j).rstar(1), V(j).rstar(1)-V(j).d(1)],[V(j).rstar(2), V(j).rstar(2)-V(j).d(2)],'LineStyle','--','Color',[.5 .5 .5])
        text((V(j).rstar(1) + V(j).rstar(1)-V(j).d(1))/2-.1,(V(j).rstar(2) + V(j).rstar(2)-V(j).d(2))/2+.1,['d_{' num2str(i-1) '}'])
        
        trajectory{j} = xx;
    end
end

% uistack(panlist{1},'top');
for i = 2:MAX_ITER
    set(panlist{i},'Visible','off')
end
uicontrol('Parent',f,...
        'Style', 'slider',...
        'Min',0, ...
        'Max',MAX_ITER-1, ...
        'Value',[0],...
        'SliderStep',[1/(MAX_ITER-1) 1/(MAX_ITER-1)], ...
        'Position', [100 20 610 20],...
        'Callback', @src,...
        'UserData',struct('plots',[panlist{:}],'total',MAX_ITER));
    
function src(source,event)
    panlist = source.UserData.plots;
    MAX_ITER = source.UserData.total;
    for i = 1:MAX_ITER
        set(panlist(i),'Visible','off')
    end
    val = 1+floor(get(source, 'Value'));
%     uistack(panlist{val},'top');
    set(panlist(val),'Visible','on')
end