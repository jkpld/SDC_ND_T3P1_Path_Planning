function [fig, ax] = plot_1d_trajs(trajs, costs, is_valid, t0, fig)

if nargin < 4
    t0 = 0;
end

if nargin < 5
    fig = figure('Visible','off','Position',[1173          66         459         874]);
else
    fig.Visible = 'off';
end

try


trajGen = TrajectoryGeneration();

ax(4) = subplot(4,1,4);
ax(1) = subplot(4,1,1);
ax(2) = subplot(4,1,2);
ax(3) = subplot(4,1,3);


[~, s_idx] = sort(costs + (~is_valid)*5000,'descend');
costs = costs(s_idx);
trajs = trajs(s_idx);
is_valid = is_valid(s_idx);

costs = (costs - min(costs))/100;

for i = 1:numel(costs)
    t = linspace(0,20,50);%max(trajGen.T),50);%trajs(i).T,50);%

    col = [1 1 1] - [0 1 1]/(costs(i)+1);
    col2 = [1 1 1] - [1 1 0]/(costs(i)+1);
    col3 = [1 1 1] - [1 0 1]/(costs(i)+1);

    if i == numel(costs)
        lw = 2;
        col = [1 0 0];
        col2 = [0 0 1];
        col3 = [0 1 0];
    else
        col = 0.5*col;
        col2 = 0.5*col2;
        col3 = 0.5*col3;
        lw = 1;
    end

    if ~is_valid(i)
        col = 0.05 * [1 1 1];
        col2 = col;
        col3 = col;
    end

    line(t+t0,trajs(i).evaluate(t,0),'color',col, 'Parent',ax(1),'linewidth',lw)
    line(t+t0,trajs(i).evaluate(t,1),'color',col2,'Parent',ax(2),'linewidth',lw)
    line(t+t0,trajs(i).evaluate(t,2),'color',col3,'Parent',ax(3),'linewidth',lw)
    line(t+t0,trajs(i).evaluate(t,3),'color',col2,'Parent',ax(4),'linewidth',lw)
end
% setTheme(fig,'dark')



axis(ax(1),'tight')
xlim(ax(1),ax(1).XLim);
% ylim(ax(1),ax(1).YLim);

ylim(ax(2),trajGen.MAX_SPEED*[-1,1]);
ylim(ax(3),trajGen.MAX_ACCEL*[-1,1]);
ylim(ax(4),trajGen.MAX_JERK*[-1,1]);

xlabel(ax(4),'Time / s');
ylabel(ax(1),'Position / m');
ylabel(ax(2),'Speed / m s^{-1}');
ylabel(ax(3),'Acceleration / m s^{-2}');
ylabel(ax(4),'Jerk / m s^{-3}');

grid(ax(1),'on')
grid(ax(2),'on')
grid(ax(3),'on')
grid(ax(4),'on')

fig.Visible = 'on';
catch ME
    fig.Visible = 'on';
    rethrow(ME)
end

end
