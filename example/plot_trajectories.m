function [] = plot_trajectories(simulations)

    n_simulations = length(simulations);

    f1 = figure(1);
    f2 = figure(2);
    f3 = figure(3);
    f4 = figure(4);
    f5 = figure(5);
    f6 = figure(6);

    font_size = 20;
    font_name = "Times New Roman";

    time = simulations(1).q.Time;
    q = zeros(n_simulations, length(simulations(1).q.Time));
    qd = zeros(n_simulations, length(simulations(1).qd.Time));
    tau = zeros(n_simulations, length(simulations(1).tau.Time));

    for i=1:n_simulations
    
        % Plot with transparency
        figure(f1);
        h = plot(simulations(i).q); hold on;
        h.Color = [0 0.4470 0.7410 0.2];
        xlabel('Time (s)', FontSize=font_size, FontName=font_name);
        ylabel('Position (rad)', FontSize=font_size, FontName=font_name);
        title("");

        % Plot in bold colors
        figure(f4);
        plot(simulations(i).q); hold on;
        xlabel('Time (s)', FontSize=font_size, FontName=font_name);
        ylabel('Position (rad)', FontSize=font_size, FontName=font_name);
        title("");

        % Plot with transparency
        figure(f2);
        h = plot(simulations(i).qd); hold on;
        h.Color = [0 0.4470 0.7410 0.2];
        xlabel('Time (s)', FontSize=font_size, FontName=font_name);
        ylabel('Velocity (rad/s)', FontSize=font_size, FontName=font_name);
        title("");

        % Plot in bold colors
        figure(f5);
        plot(simulations(i).qd); hold on;
        xlabel('Time (s)', FontSize=font_size, FontName=font_name);
        ylabel('Velocity (rad/s)', FontSize=font_size, FontName=font_name);
        title("");

        % Plot with transparency
        figure(f3);
        h = plot(simulations(i).tau); hold on;
        h.Color = [0 0.4470 0.7410 0.2];
        xlabel('Time (s)', FontSize=font_size, FontName=font_name);
        ylabel('Torque (Nm)', FontSize=font_size, FontName=font_name);
        title("");

        % Plot in bold colors
        figure(f6);
        plot(simulations(i).tau); hold on;
        xlabel('Time (s)', FontSize=font_size, FontName=font_name);
        ylabel('Torque (Nm)', FontSize=font_size, FontName=font_name);
        title("");

        % Verify that all time vectors are the same to compute mean and
        % standard deviation
        time_equal = isequal(simulations(i).q.Time, time) & isequal(simulations(i).qd.Time, time) & isequal(simulations(i).tau.Time, time);

        if ~time_equal
            error("Time vectors of simulations differ");
        end

        % Collect positions, velocities and torques from all simulations
        q(i,:) = simulations(i).q.Data;
        qd(i,:) = simulations(i).qd.Data;
        tau(i,:) = simulations(i).tau.Data;
    
    end

    % Compute mean and standard deviation
    [q_std, q_mean] = std(q);
    [qd_std, qd_mean] = std(qd);
    [tau_std, tau_mean] = std(tau);

    % Plot means with confidence values
    t_conf = [time' time(end:-1:1)'];
    q_conf = [q_mean-q_std q_mean(end:-1:1)+q_std(end:-1:1)];
    qd_conf = [qd_mean-qd_std qd_mean(end:-1:1)+qd_std(end:-1:1)];
    tau_conf = [tau_mean-tau_std tau_mean(end:-1:1)+tau_std(end:-1:1)];

    figure(7);
    h = fill(t_conf, q_conf, 'red'); hold on;
    h.FaceColor = [0 0.4470 0.7410];
    h.FaceAlpha = 0.2;
    h.EdgeColor = 'none';
    plot(time, q_mean);
    xlabel('Time (s)', FontSize=font_size, FontName=font_name);
    ylabel('Position (rad)', FontSize=font_size, FontName=font_name);

    figure(8);
    h = fill(t_conf, qd_conf, 'red'); hold on;
    h.FaceColor = [0 0.4470 0.7410];
    h.FaceAlpha = 0.2;
    h.EdgeColor = 'none';
    plot(time, qd_mean);
    xlabel('Time (s)', FontSize=font_size, FontName=font_name);
    ylabel('Velocity (rad/s)', FontSize=font_size, FontName=font_name);

    figure(9);
    h = fill(t_conf, tau_conf, 'red'); hold on;
    h.FaceColor = [0 0.4470 0.7410];
    h.FaceAlpha = 0.2;
    h.EdgeColor = 'none';
    plot(time, tau_mean);
    xlabel('Time (s)', FontSize=font_size, FontName=font_name);
    ylabel('Torque (Nm)', FontSize=font_size, FontName=font_name);

end
