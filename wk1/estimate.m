function [mu, covariance ] = estimate(samples)
    samples = im2double(samples);
    mu = mean(samples, 1);
    covariance = cov(samples)*(length(samples)-1)/length(samples);
end