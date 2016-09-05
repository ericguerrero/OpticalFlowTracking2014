function MTP(Traj, Orien, NumTraj,PS,frame)% MultiTrajectory Ploting

    linespec = {'b', 'r', 'g', 'c', 'k'};
    legendInfo = {'a'};
    ArowSize=0.5;
    s=size(frame);

    %Plot trajectories
    figure('Position',[185 265 1527  342]);
    subplot(1,2,1);

    hold on;
    for i=1:NumTraj
        plot3(Traj(:,1,i)/PS,-Traj(:,2,i)/PS,abs(Orien(:,1,i)),linespec{i});
        axis([0 s(2)/PS -s(1)/PS 0 ]);title('Trajectory'); %
       legendInfo{i} = ['Trajectory' num2str(i)];
       xlabel('[mm]');ylabel('[mm]'); axis equal
       grid on;
    end
    set(gcf, 'color', 'w');
    legend(legendInfo)

    %Plot velocities
    for i=1:NumTraj
        subplot(NumTraj,2,2*i);
        plot(abs(Orien(:,1,i))*30/PS,linespec{i});
        title(['Velocity Trajectory ' num2str(i)]); 
        xlabel('[frame]');ylabel('[mm/s]');
        grid on;
    end
end