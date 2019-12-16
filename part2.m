%[transforms, objects] = part2( imglistdepth, imglistrgb,   cam_params)
close all;
%RANSAC parameters
niterations = 100;
errorthreshold = 0.1;
limit_inliers = 12;

%Images load - Remove for teacher's code
imglistrgb = {'room1/rgb_0000.jpg','room1/rgb_0001.jpg','room1/rgb_0002.jpg','room1/rgb_0003.jpg','room1/rgb_0004.jpg','room1/rgb_0005.jpg','room1/rgb_0006.jpg'};
imglistdepth = {'room1/depth_0000.mat','room1/depth_0001.mat', 'room1/depth_0002.mat','room1/depth_0003.mat','room1/depth_0004.mat','room1/depth_0005.mat','room1/depth_0006.mat'};
%imglistrgb = {'board1/rgb_0000.jpg', 'board1/rgb_0001.jpg', 'board1/rgb_0002.jpg', 'board1/rgb_0003.jpg', 'board1/rgb_0004.jpg', 'board1/rgb_0005.jpg', 'board1/rgb_0006.jpg', 'board1/rgb_0007.jpg', 'board1/rgb_0008.jpg', 'board1/rgb_0009.jpg', 'board1/rgb_0010.jpg', 'board1/rgb_0011.jpg', 'board1/rgb_0012.jpg', 'board1/rgb_0013.jpg', 'board1/rgb_0014.jpg'};
%imglistdepth = {'board1/depth_0000.mat', 'board1/depth_0001.mat', 'board1/depth_0002.mat', 'board1/depth_0003.mat', 'board1/depth_0004.mat', 'board1/depth_0005.mat', 'board1/depth_0006.mat', 'board1/depth_0007.mat', 'board1/depth_0008.mat', 'board1/depth_0009.mat', 'board1/depth_0010.mat', 'board1/depth_0011.mat', 'board1/depth_0012.mat', 'board1/depth_0013.mat', 'board1/depth_0014.mat'};
%imglistrgb = {'board2/rgb_0000.jpg', 'board2/rgb_0001.jpg', 'board2/rgb_0002.jpg', 'board2/rgb_0003.jpg', 'board2/rgb_0004.jpg', 'board2/rgb_0005.jpg', 'board2/rgb_0006.jpg'};
%imglistdepth = {'board2/depth_0000.mat', 'board2/depth_0001.mat', 'board2/depth_0002.mat', 'board2/depth_0003.mat', 'board2/depth_0004.mat', 'board2/depth_0005.mat', 'board2/depth_0006.mat'};
%imglistrgb = {'labpiv/rgb_image_1.png', 'labpiv/rgb_image_2.png', 'labpiv/rgb_image_3.png', 'labpiv/rgb_image_4.png', 'labpiv/rgb_image_5.png', 'labpiv/rgb_image_6.png', 'labpiv/rgb_image_7.png', 'labpiv/rgb_image_8.png', 'labpiv/rgb_image_9.png', 'labpiv/rgb_image_10.png', 'labpiv/rgb_image_11.png', 'labpiv/rgb_image_12.png', 'labpiv/rgb_image_13.png', 'labpiv/rgb_image_14.png', 'labpiv/rgb_image_15.png', 'labpiv/rgb_image_16.png', 'labpiv/rgb_image_17.png', 'labpiv/rgb_image_18.png', 'labpiv/rgb_image_19.png', 'labpiv/rgb_image_20.png', 'labpiv/rgb_image_21.png', 'labpiv/rgb_image_22.png', 'labpiv/rgb_image_23.png'};
%imglistdepth = {'labpiv/depth_1.mat', 'labpiv/depth_2.mat', 'labpiv/depth_3.mat', 'labpiv/depth_4.mat', 'labpiv/depth_5.mat', 'labpiv/depth_6.mat', 'labpiv/depth_7.mat', 'labpiv/depth_8.mat', 'labpiv/depth_9.mat', 'labpiv/depth_10.mat', 'labpiv/depth_11.mat', 'labpiv/depth_12.mat', 'labpiv/depth_13.mat', 'labpiv/depth_14.mat', 'labpiv/depth_15.mat', 'labpiv/depth_16.mat', 'labpiv/depth_17.mat', 'labpiv/depth_18.mat', 'labpiv/depth_19.mat', 'labpiv/depth_20.mat', 'labpiv/depth_21.mat', 'labpiv/depth_22.mat', 'labpiv/depth_23.mat'};
%imglistrgb = {'table/rgb_0000.jpg', 'table/rgb_0001.jpg', 'table/rgb_0002.jpg', 'table/rgb_0003.jpg', 'table/rgb_0004.jpg', 'table/rgb_0005.jpg', 'table/rgb_0006.jpg', 'table/rgb_0007.jpg'};
%imglistdepth = {'table/depth_0000.mat', 'table/depth_0001.mat', 'table/depth_0002.mat', 'table/depth_0003.mat', 'table/depth_0004.mat', 'table/depth_0005.mat', 'table/depth_0006.mat', 'table/depth_0007.mat'};

%Return arrays
transforms = cell(length(imglistrgb));
objects = [];
transforms{1}.R = eye(3);
transforms{1}.T = zeros(3, 1);

%% Cell array declarations for images
imrgb = cell(length(imglistrgb));
depth = cell(length(imglistdepth));
imrgbd = cell(length(imglistrgb));
xyz_array = cell(length(imglistrgb));
xyz = cell(length(imglistrgb));

%% Images load from the given lists
for i=1:length(imglistrgb)
    imrgb{i} = imread(imglistrgb{i});
    depth{i} = load(imglistdepth{i}); 
end
cam = load("calib_asus.mat");

%% Get Images in rgbd
for i=1:length(imrgb)
     xyz_array{i} = get_xyzasus(depth{i}.depth_array(:), [480 640], 1:480*640, cam.Depth_cam.K,1,0);
     xyz_array{i}(isnan(xyz_array{i})) = 0;
     imrgbd{i} = get_rgbd(xyz_array{i}, imrgb{i}, cam.R_d_to_rgb, cam.T_d_to_rgb, cam.RGB_cam.K);
     xyz{i} = reshape(xyz_array{i}, [480, 640, 3]);
 end
 
%% Declaration of cell arrays used for feature matching
pts = cell(length(imglistrgb));
features = cell(length(imglistrgb));
valid_points = cell(length(imglistrgb));
matchpoints = cell(length(imglistrgb) - 1, 2);

%% Detecting features on image 1 and main loop
pts{1} = detectSURFFeatures(rgb2gray(imrgbd{1}));
[features{1}, valid_points{1}] = extractFeatures(rgb2gray(imrgbd{1}), pts{1});
for main=2:length(imglistrgb)
    %% Feature matching done
    pts{main} = detectSURFFeatures(rgb2gray(imrgbd{main}));
    [features{main}, valid_points{main}] = extractFeatures(rgb2gray(imrgbd{main}), pts{main});
    for checkmatches=1:(main - 1)
        pairs = matchFeatures(features{checkmatches}, features{main});
        matchpoints1 = valid_points{checkmatches}(pairs(:,1));
        matchpoints2 = valid_points{main}(pairs(:,2));

        figure; 
        showMatchedFeatures(imrgbd{checkmatches}, imrgbd{main}, matchpoints1, matchpoints2, 'montage');
        if (length(matchpoints1) < limit_inliers)
            continue
        end
        
        
        %% Filter Points to remove depth nulls and get matchpoints in 3d
        k=0;
        for i=1:length(matchpoints1)
            if (xyz{checkmatches}(round(matchpoints1.Location(i, 2)), round(matchpoints1.Location(i, 1)), 3) ~= 0)
                if(xyz{main}(round(matchpoints2.Location(i, 2)), round(matchpoints2.Location(i, 1)), 3) ~= 0)
                    k = k + 1;
                end
            end
        end
        match3d_1 = zeros(k, 3);
        match3d_2 = zeros(k, 3);
        a=0;
        for i=1:length(matchpoints1)
            if (xyz{checkmatches}(round(matchpoints1.Location(i, 2)), round(matchpoints1.Location(i, 1)), 3) ~= 0)
                if (xyz{main}(round(matchpoints2.Location(i, 2)), round(matchpoints2.Location(i, 1)), 3) ~= 0)
                    a = a + 1;
                    %match3d_1(a, 1:2) = matchpoints1.Location(i, :);
                    %match3d_2(a, 1:2) = matchpoints2.Location(i, :);
                    match3d_1(a, :) = xyz{checkmatches}(round(matchpoints1.Location(i, 2)), round(matchpoints1.Location(i, 1)), :);
                    match3d_2(a, :) = xyz{main}(round(matchpoints2.Location(i, 2)), round(matchpoints2.Location(i, 1)), :);
                end
            end
        end

        if length(match3d_1) < 4
            continue
        end
        %% Necessary declarations to set up RANSAC
        random = zeros(4, 1);
        xyz1_points = zeros(4,3);
        xyz2_points = zeros(4,3);
        n_inliers = zeros(1, niterations);
        im1_inliers = zeros(length(match3d_1), 3, niterations);
        im2_inliers = zeros(length(match3d_2), 3, niterations);
        model = zeros(3, 4, niterations);
        max_inliers = 0;
        index_max_inliers = 0;
        %% RANSAC
        for i=1:niterations
            %% Getting the 4 random points
            %% VERIFIY IF THESE POINTS ARE COPLANAR
            random = randperm(length(match3d_1), 4);
            xyz1_points = match3d_1(random(:), :);
            xyz2_points = match3d_2(random(:), :);
            %% Getting the model
            A = xyz1_points(:,:)';
            B = [xyz2_points(:,:) ones(4,1)]';
            model(:, :, i) = A/B;
            %% Finding Inliers
            for k=1:length(match3d_1)
                C = [match3d_2(k, 1:3) 1]';
                error = norm(match3d_1(k, 1:3)' - model(:,:,i)*C);
                if(error < errorthreshold)
                    n_inliers(i) = n_inliers(i) + 1;
                    im1_inliers(n_inliers(i), :, i) = match3d_1(k, :);
                    im2_inliers(n_inliers(i), :, i) = match3d_2(k, :);
                end
            end
            if(max_inliers < n_inliers(i))
                max_inliers = n_inliers(i);
                index_max_inliers = i;
            end
        end
        if (max_inliers >= limit_inliers)
            break;
        end
    end
    
    %% Estimating the model
    inliers_ransac_im1 = im1_inliers(1:max_inliers, :, index_max_inliers);
    inliers_ransac_im2 = im2_inliers(1:max_inliers, :, index_max_inliers);

    centroid1 = sum(inliers_ransac_im1(:, :))/max_inliers;
    centroid2 = sum(inliers_ransac_im2(:, :))/max_inliers;

    A = inliers_ransac_im1';
    B = inliers_ransac_im2';

    [u, c, v] = svd(A*B');

    R = u*v';
    T = centroid1' - R*centroid2';
    if(checkmatches ~= 1)
        transforms{main}.R = transforms{checkmatches}.R*R;
        transforms{main}.T = transforms{checkmatches}.R*T + transforms{checkmatches}.T;
    else
        transforms{main}.R = R;
        transforms{main}.T = T;
    end

    xyz21=R*xyz_array{main}' + repmat(T, 1,480*640);

    pc1 = pointCloud(xyz_array{checkmatches}, 'Color', reshape(imrgbd{checkmatches}, [480*640 3]));
    pc2 = pointCloud(xyz21', 'Color', reshape(imrgbd{main}, [480*640 3]));
    %figure;
    %showPointCloud(pcmerge(pc2,pc1,0.00001));
end

%% Code to check pointcloud
pc = cell(length(imglistrgb));
pc{1} = pointCloud(xyz_array{1}, 'Color', reshape(imrgbd{1}, [480*640 3]));
pcmmm = pc{1};
for alfa=2:7%length(imglistrgb)
    aux = transforms{alfa}.R*xyz_array{alfa}' +  repmat(transforms{alfa}.T, 1, 480*640);
    pc{alfa} = pointCloud(aux', 'Color', reshape(imrgbd{alfa}, [480*640 3]));
    pcmmm = pcmerge(pc{alfa}, pcmmm, 0.00001);
end
figure;
showPointCloud(pcmmm);