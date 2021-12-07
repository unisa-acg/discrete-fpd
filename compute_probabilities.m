
function [probabilities] = compute_probabilities(domain, m, sigma)

    deltas = diff(domain);
    delta = deltas(1);

    density = pdf('Normal', domain, m, sigma);
    
    for i=length(domain):-1:1
   
        probabilities(i) = density(i)*delta;
    
    end
    
    % normalization
    total = sum(probabilities);    
    probabilities = probabilities/total;
    
end

