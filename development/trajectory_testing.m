s0 = State(0, [0, 0, 0], [0 0 0]);
slv = State(0, [20, -2, 0.13], [0 0 0]);
sa = State(0, [1, 1, -0.1], [0 0 0]);
sb = State(0, [6, 1, -0.05], [0 0 0]);

v_ref = 20;

profile on
traj = TrajectoryGeneration();

[trajs, costs, is_valid] = traj.stopping(s0,4);

try close(fig), catch, end
[fig, ax] = plot_1d_trajs(trajs, costs, is_valid, 0);

% for i = 1:7
%     [~, best_idx] = min(costs + (~is_valid)*5000);
%     s0.x = trajs(best_idx).state_at(2.5);
%     slv = slv.state_at(2.5*i);
% 
%     [trajs, costs, is_valid] = traj.following(s0,slv);
%     [fig, ax] = plot_1d_trajs(trajs, costs, is_valid, 2.5*i, fig);
% end

profile off
profile viewer

setTheme(fig,'dark')


% loc = slv.location(linspace(0,35,35));
% line(linspace(0,35,35), loc(:,1),'linewidth',2,'color','g','Parent',ax(1))
% line(t, polyval([0.5*sa(3),sa(2),sa(1)],t),'linewidth',2,'color','g','Parent',ax1)
% line(t, polyval([0.5*sb(3),sb(2),sb(1)],t),'linewidth',2,'color','b','Parent',ax1)


%%
[~, best_idx] = min(costs + (~is_valid)*5000);
traj_d = trajs(best_idx);