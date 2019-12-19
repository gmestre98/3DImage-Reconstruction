%function [transforms, objects] = part2( imglistdepth, imglistrgb, cam_params)

close all;

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

%% Return arrays
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
limit_inliers = 9;

%% Detecting features on image1 and main loop
pts{1} = detectSURFFeatures(rgb2gray(imrgbd{1}), 'MetricThreshold', 400);
[features{1}, valid_points{1}] = extractFeatures(rgb2gray(imrgbd{1}), pts{1});
for imget=2:length(imglistrgb)
    pts{imget} = detectSURFFeatures(rgb2gray(imrgbd{imget}), 'MetricThreshold', 400);
    [features{imget}, valid_points{imget}] = extractFeatures(rgb2gray(imrgbd{imget}), pts{imget});
    for imgot=1:(imget-1)
        pairs = matchFeatures(features{imgot}, features{imget}, 'Unique', true);
        matchpoints1 = valid_points{imgot}(pairs(:,1));
        matchpoints2 = valid_points{imget}(pairs(:,2));
        [match3d_1, match3d_2, aux1, aux2] = removezeros(xyz, matchpoints1, matchpoints2, imgot, imget);
        if length(match3d_1) < 8
            continue;
        end
        [in1, in2, inaux1, inaux2] = ransac(match3d_1, match3d_2, aux1, aux2);
        if(length(in1) >= limit_inliers || imgot == imget-1)
            figure;
            showMatchedFeatures(imrgbd{imgot}, imrgbd{imget}, inaux1, inaux2, 'montage');
            break;
        end
    end
    centroid1 = sum(in1(:, :))/length(in1);
    centroid2 = sum(in2(:, :))/length(in2);
    
    A = in1' - repmat(centroid1', 1, length(in1));
    B = in2' - repmat(centroid2', 1, length(in2));
    
    [u, c, v] = svd(A*B');
    
    R = u*v';
    det(R);
    if(det(R) < 0)
       a = eye(3);
       a(3, 3) = -1;
       R = u*a*v';
    end
    det(R);
    T = centroid1' - R*centroid2';
    
    transforms{imget}.R = transforms{imgot}.R*R;
    transforms{imget}.T = transforms{imgot}.R*T + transforms{imgot}.T;
end

%% Code to check pointcloud
pc = cell(length(imglistrgb));
pc{1} = pointCloud(xyz_array{1}, 'Color', reshape(imrgbd{1}, [480*640 3]));
pcmmm = pc{1};
figure;
showPointCloud(pc{1});
for alfa=2:length(imglistrgb)
    aux = transforms{alfa}.R*xyz_array{alfa}' +  repmat(transforms{alfa}.T, 1, 480*640);
    pc{alfa} = pointCloud(aux', 'Color', reshape(imrgbd{alfa}, [480*640 3]));
    pcmmm = pcmerge(pc{alfa}, pcmmm, 0.00001);
    showPointCloud(pcmmm);
    drawnow;pause;
end