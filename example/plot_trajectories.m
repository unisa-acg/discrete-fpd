function [] = plot_trajectories(simulations)

    n_simulations = length(simulations);
    
    f1 = figure;
    f2 = figure;
    f3 = figure;

    for i=1:n_simulations
    
        figure(f1);
        plot(simulations(i).q); hold on;
        xlabel('Time (s)');
        ylabel('Position (rad)');

        figure(f2);
        plot(simulations(i).qd); hold on;
        xlabel('Time (s)');
        ylabel('Velocity (rad/s)');

        figure(f3);
        plot(simulations(i).tau); hold on;
        xlabel('Time (s)');
        ylabel('Torque (Nm)');
    
    end

end
