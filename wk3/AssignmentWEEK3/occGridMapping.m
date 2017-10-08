% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)
    starting = param.origin;
    myMap = zeros(param.size);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    % Parameters 
    % 
    % % the number of grids for 1 meter.
    myResol = param.resol;
    % % the initial map size in pixels
    myMap = zeros(param.size);
    % % the origin of the map in pixels
    %myorigin = param.origin; 
    % 
    % % 4. Log-odd parameters 
    lo_occ = param.lo_occ;
    lo_free = param.lo_free; 
    lo_max = param.lo_max;
    lo_min = param.lo_min;

    function [xvalue, yvalue] = toCoord(x, y, sx, sy)
        %xvalue = ceil(x*myResol) +sx;
        %yvalue = ceil(y*myResol) + sy;
        if x > 0,
            xvalue = ceil(x*myResol) + sx;
        else
            xvalue = floor(x*myResol) + sx;
        end
        if y > 0,
            yvalue = ceil(y*myResol) + sy;
        else
            yvalue = floor(y*myResol) + sy;
        end
    end

    N = size(pose,2);
    for j = 1:N % for each time,
        disp(j);
        %Find grids hit by the rays (in the gird map coordinate)
        [startx, starty] = toCoord(pose(1, j), pose(2, j), starting(1), starting(2));
        theta = pose(3, j);
        local = [ranges(:,j).*cos(scanAngles+theta) -ranges(:,j).*sin(scanAngles+theta)];

        %Find occupied-measurement cells and free-measurement cells
        for i=1:size(ranges, 1),
            %d = ranges(i, j);
            %[xocc, yocc] = toCoord(d*cos(theta + scanAngles(i)), ...
                                %-d*sin(theta + scanAngles(i)), startx, starty);
            [xocc, yocc] = toCoord(local(i, 1), local(i, 2), startx, starty);
            myMap(yocc, xocc) = min(myMap(yocc, xocc) + lo_occ, lo_max);
            [freex, freey] = bresenham(startx,starty,xocc,yocc);
            free = sub2ind(size(myMap),freey,freex);
            myMap(free) = max(myMap(free) - lo_free, lo_min);
        end
    end
end


