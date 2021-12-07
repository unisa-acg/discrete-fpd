function [q_t, qd_t] = generate_random_1sp_trajectory(lambdad_min, lambdad_range, delta_lambda)

    number_of_samples = 1/delta_lambda + 1;

    ctrl_points(:,1) = [0;0];
    ctrl_points(:,2) = [0.5; lambdad_min + rand*lambdad_range];
    ctrl_points(:,3) = [1;0];

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

