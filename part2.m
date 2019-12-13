imglistrgb = {'room1/rgb_0000.jpg','room1/rgb_0001.jpg','room1/rgb_0002.jpg','room1/rgb_0003.jpg','room1/rgb_0004.jpg','room1/rgb_0005.jpg','room1/rgb_0006.jpg'};
imglistdepth = {'room1/depth_0000.mat','room1/depth_0001.mat', 'room1/depth_0002.mat','room1/depth_0003.mat','room1/depth_0004.mat','room1/depth_0005.mat','room1/depth_0006.mat'};

imrgb = cell(length(imglistrgb));
depth = cell(length(imglistdepth));
imrgbd = cell(length(imglistrgb));

for i=1:length(imglistrgb)
    imrgb{i} = imread(imglistrgb{i});
    depth{i} = load(imglistdepth{i});
end
cam = load("calib_asus.mat");
for i=1:length(imrgb)
    xyz = get_xyzasus(depth{i}.depth_array(:), [480 640], 1:480*640, cam.Depth_cam.K,1,0)*1000;
    xyz(isnan(xyz)) = 0;
    imrgbd{i} = get_rgbd(xyz, imrgb{i}, cam.R_d_to_rgb, cam.T_d_to_rgb, cam.RGB_cam.K);
    figure;
    imshow(imrgbd{i});
    hold on; 
end

pts = cell(length(imglistrgb));
features = cell(length(imglistrgb));
valid_points = cell(length(imglistrgb));
matchpoints = cell(length(imglistrgb) - 1, 2);

pts{1} = detectSURFFeatures(rgb2gray(imrgbd{1}));
[features{1}, valid_points{1}] = extractFeatures(rgb2gray(imrgbd{1}), pts{1});
pts{2} = detectSURFFeatures(rgb2gray(imrgbd{2}));
[features{2}, valid_points{2}] = extractFeatures(rgb2gray(imrgbd{2}), pts{2});
pairs = matchFeatures(features{1}, features{2});
matchpoints1 = valid_points{1}(pairs(:,1));
matchpoints2 = valid_points{2}(pairs(:,2));

figure;
showMatchedFeatures(imrgbd{1}, imrgbd{2}, matchpoints1, matchpoints2, 'montage');



% transforms = cell(1, 7);
% matchpoints = cell(7 - 1, 2);
% pts_imrgb{1} = detectSURFFeatures(rgb2gray(imrgb{1}));
% for i = 2:length(7)
%     pts_imrgb{i} = detectSURFFeatures(imrgb{i});
%     [features_imrgb{i}, valid_pointsrgb{i}] = extractFeatures(imrgb{i}, pts_imrgb{i});
%     pairs = matchFeatures(features_imrgb{1}, features_imrgb{i});
%     matchpoints{i-1, 1} = valid_pointsrgb{1}(pairs(:, 1));
%     matchpoints{i-1, 2} = valid_pointsrgb{i}(pairs(:, 2));
% end

