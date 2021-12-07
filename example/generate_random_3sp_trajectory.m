function [q_t, qd_t] = generate_random_3sp_trajectory(lambdad_min, lambdad_range, delta_lambda, variance)

    number_of_samples = 1/delta_lambda + 1;
    
    second_sp = -1;
    third_sp = -1;
    
    first_sp = lambdad_min + rand*lambdad_range;
    
    while second_sp < 0
        second_sp = first_sp + variance*randn;
    end
    
    while third_sp < 0
        third_sp = first_sp + variance*randn;
    end

    ctrl_points(:,1) = [0;0];
    ctrl_points(:,2) = [0.25; first_sp];
    ctrl_points(:,3) = [0.50; second_sp];
    ctrl_points(:,4) = [0.75; third_sp];
    ctrl_points(:,5) = [1;0];

    x = ctrl_points(1,:);
    v = ctrl_points(2,:);
    lambda = linspace(0,1,number_of_samples);

    lambdad = interp1(x, v, lambda, 'linear');

    time = zeros(1, number_of_samples);

    for i=2:number_of_samples

        time(i) = time(i-1) + 2*(lambda(i)-lambda(i-1))/(lambdad(i)+lambdad(i-1));

    end

    q = linspace(-pi/2, pi/2, number_of_samples);
    qd = diff(q)./diff(lambda).*lambdad(1:end-1);
    qd = [qd 0];

    q_t = timeseries(q, time);
    qd_t = timeseries(qd, time);

end

