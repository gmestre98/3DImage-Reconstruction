function [inliers1, inliers2, inaux1, inaux2] = ransac(match3d_1, match3d_2, aux1, aux2)
    niterations = 500;
    errorthresh = 0.10;
    goodinds = [];
    %% RANSAC
    for i=1:niterations
        xyz2_points = zeros(4,3);
        %% Getting the 4 random points
        while det([xyz2_points(:,:) ones(4,1)]') < 10^-5
            random = randperm(length(match3d_1), 4);
            xyz1_points = match3d_1(random(:), :);
            xyz2_points = match3d_2(random(:), :);
        end
        [~, ~, transform] = procrustes(xyz1_points, xyz2_points, 'Scaling', 0, 'Reflection', 0);
        R = transform.T;
        T = transform.c(1, :);
        %model = xyz1_points'/([xyz2_points ones(4,1)]');
        inds = [];
        for k=1:length(match3d_1)
            error = norm(match3d_1(k, :)' - R'*match3d_2(k, :)' - T');
            %error = norm(match3d_1(k, :)' - model*([match3d_2(k, :) 1]'));
            if error < errorthresh
                inds = [inds k];
            end
        end
        if(length(inds) > length(goodinds))
            goodinds = inds;
        end
    end
    inliers1 = match3d_1(goodinds, :);
    inliers2 = match3d_2(goodinds, :);
    inaux1 = aux1(goodinds, :);
    inaux2 = aux2(goodinds, :);
end