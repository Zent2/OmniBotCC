function map = genmap(obs)
%GENMAP Genera el mapa actual sobre el Robotat mediante el array de
%obstáculos
    width = 4 * 100;  % en cm 
    height = 5 * 100; % en cm
    map = zeros(height, width);
    map(:, 1:10) = ones(size(map(:, 1:10)));
    map(:, end-9:end) = ones(size(map(:, 1:10)));
    map(1:10, :) = ones(size(map(1:10, :)));
    map(end-9:end, :) = ones(size(map(1:10, :)));

    obsradius = 20; % en cm
    
    for i = 1:length(obs)
        obsx = round((-obs(i, 1) + width/200) * 100);
        obsy = round((obs(i, 2) + height/200) * 100);
        
        for j = max(0, obsx - obsradius):min(obsx + obsradius, width)
            for k = max(0, obsy - obsradius):min(obsy + obsradius, height)
                if(map(k, j) < 1)
                    map(k, j) = double(((j - obsx) ^ 2 + (k - obsy)^2) <= obsradius^2);
                end
            end
        end
    end
end