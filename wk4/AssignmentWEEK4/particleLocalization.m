% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)
    % Number of poses to calculate
    N = size(ranges, 2);
    % Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
    myPose = zeros(3, N);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    % Map Parameters 
    % 
    % % the number of grids for 1 meter.
    myResolution = param.resol;
    % the origin of the map in pixels
    myOrigin = param.origin; 

    % The initial pose is given
    myPose(:,1) = param.init_pose;
    % You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
    % The pose(:,1) should be the pose when ranges(:,j) were measured.

    % Decide the number of particles, M.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    M = 100;                        % Please decide a reasonable number of M, 
                                   % based on your experiment using the practice data.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create M number of particles
    function [xvalue, yvalue] = toCoord(x, y, sx, sy)
        if x > 0,
            xvalue = ceil(x*myResolution) + sx;
        else
            xvalue = floor(x*myResolution) + sx;
        end
        if y > 0,
            yvalue = ceil(y*myResolution) + sy;
        else
            yvalue = floor(y*myResolution) + sy;
        end
    end

    function res = invalid(px, py)
        res = false;
        if px < 1 || py < 1 || px > size(map, 2) || py > size(map, 1),
            res = true;
        end
    end

    function Xout = addRand(Xin, dr, t, dt)
        count = 0;
        while true,
            dr = 0.025;
            dist = dr + randn(1,1)*0.1;
            heading = mod(Xin(3) + dt + randn(1,1)*0.6, 2*pi);
            Xout = Xin + [dist*cos(heading); -dist*sin(heading); 0];
            Xout(3) = heading;
            count = count + 1;
            [px, py] = toCoord(Xout(1), Xout(2), myOrigin(1), myOrigin(2));
            if ~invalid(px, py) || count > 200,
                if count > 200,
                    disp('count exceed 200');
                end
                break;
            end
        end
    end

    P = repmat(myPose(:, 1), [1, M]);
    W = ones(1, M) / M;
    for j = 2:N
        WW = zeros(1, M);
        for i = 1:M
            %dist = 0.025 + randn(1,1) * 0.1;
            X = P(:,i);
            count = 0;
            px = 0;
            py = 0;
            while true,
                r = 0.025;
                %if j >= 3,
                    %r = ((myPose(1, j-1) - myPose(1, j-2))^2 + (myPose(2,j-1) - myPose(2, j-2))^2)^0.5;
                %end
                dist = r + randn(1,1)*0.1;
                heading = X(3) + randn(1, 1) * 0.15; %heading is the sum of the previous one plus a random value
                P(1, i) = X(1) + dist * cos(heading);
                P(2, i) = X(2) - dist * sin(heading);
                P(3, i) = heading;
                [px, py] = toCoord(P(1, i), P(2, i), myOrigin(1), myOrigin(2));
                count = count + 1;
                if ~invalid(px, py) || count > 200,
                    break;
                end
            end

            if map(py, px) > 0.2,%occupited
                W(i) = 0;
                continue;
            end
            
            items = size(ranges, 1);
            %items = 100;
            ids = randsample(size(ranges, 1), items); %sample the lidar
            phi = P(3, i) + scanAngles(ids);
        
            x_occ = ranges(ids, j) .* cos(phi) + P(1, i);
            y_occ = -ranges(ids, j) .* sin(phi) + P(2, i);
        
            [i_x_occ, i_y_occ] = toCoord(x_occ, y_occ, myOrigin(1), myOrigin(2));
        
        
        N_occ = size(i_x_occ, 1);
        corr = 0;
        count = 0;
        for k = 1:N_occ
            if (i_x_occ(k) > size(map, 2)) || (i_x_occ(k) < 1)
                continue;
            end
            if (i_y_occ(k) > size(map, 1)) || (i_y_occ(k) < 1)
                continue;
            end
            
            %if map(i_y_occ(k), i_x_occ(k)) < 0
            if map(i_y_occ(k), i_x_occ(k)) < 0.2,
                %corr = corr - 5 * map(i_y_occ(k), i_x_occ(k));
                corr = corr + map(i_y_occ(k), i_x_occ(k));% + 0.5;
            end
            %if map(i_y_occ(k), i_x_occ(k)) >= 1
            if map(i_y_occ(k), i_x_occ(k)) >= 0.8,
                %corr = corr + 10 * map(i_y_occ(k), i_x_occ(k));
                corr = corr + map(i_y_occ(k), i_x_occ(k));% + 0.5;
            end
            %if map(i_y_occ(k), i_x_occ(k)) > 0 && map(i_y_occ(k), i_x_occ(k)) < 1,
                %corr = corr - map(i_y_occ(k), i_x_occ(k));
            %end
        end
        WW(i) = WW(i) + corr;
        end
    WW = WW - min(WW) + 1e-4;
    WW= WW/max(WW);
    W = W.*WW;
    W_norm = W / sum(W);
    
        [W_sorted, sort_ids] = sort(W_norm, 'descend');
        neffective = round((sum(W_sorted)^2)/sum(W_sorted.^2));
        %neffective = min(neffective, 10);
        W_sample = W_sorted(1:neffective);

        P_sorted = P(:, sort_ids);
        P_sample = P_sorted(:, 1:neffective);
    
        myPose(:, j) = P_sorted(:, 1);
        %myPose(:, j) = mean(P_sorted(:, 1:10), 2);  %The two methods show similar in results

        p_ids = randsample(neffective, M, 'true', W_sample);
        W = W_sample(p_ids');
        W = ones(1, M)/M;
        P = P_sample(:, p_ids');
        %Pin = P;
        %W = W_norm;
        %neffective = (sum(W_norm)^2)/sum(W_norm.^2);
        %[~, I] = max(W_norm);
        %if neffective < 0.5*M,
        %    [W, P] = resample(Pin, W_norm);
        %end
        %myPose(:, j) = Pin(:, I);
    end
    figure;
    imagesc(map);hold on;
    plot(myPose(1,:)*param.resol+param.origin(1), ...
    myPose(2,:)*param.resol+param.origin(2), 'r.-');
end

