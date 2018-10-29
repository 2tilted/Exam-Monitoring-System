% This function detects faces or objects, calculates and returns their 
% 3D position. In case depth on the Z axis could not be evaluated, it
% returns [NaN NaN NaN].
%function centroids3D = getPos(f, f_depth, stereoParams, target)

v = VideoReader('corrected_video_left.avi'); % main channel
v_depth = VideoReader('corrected_video_right.avi'); % depth channel
dummy_video = VideoReader('corrected_video_right.avi'); 

%% get number of frames
numFrames = 0;
while hasFrame(dummy_video)
    readFrame(dummy_video);
    numFrames = numFrames + 1;
end

for i = 1:numFrames
    
    % get both frames
    f = readFrame(v);
    f_depth = readFrame(v_depth);

    if i < 852
        continue;
    end 
%% load stereo parameters
load('steoreoParams_classroom.mat');
 
%% read and rectify video frames
%f = imread('image0000_right.png');
%f_depth = imread('image0000_left.png');
[f_rect, f_depth_rect] = rectifyStereoImages(f, f_depth, stereoParams);
 
%figure;
%imshow(stereoAnaglyph(f_rect, f_depth_rect));
%title('Rectified Video Frames');
 
%% compute disparity
% Since the baseline for the used camera setup is 140mm between both
% lenses and objects are observed in rather near distance, 
 
% Method 'SemiGlobal' provides smooth results, 'BlockMatching' is rather 
% noisy but mostly more accurate. There is no observable effect on patches.
DISPMETHOD = 'SemiGlobal'; % ['SemiGlobal', 'BlockMatching']
 
% Using method 'SemiGlobal', max disparity should be no more than 128
% since it takes away information from the sides of the disparityMap. In
% case of 'BlockMatching', either value provides good results.
DISPRANGE = 128; % [128, 256, 512]
 
% Convert frames to gray color and create disparityMap
f_gray  = rgb2gray(f_rect);
f_depth_gray = rgb2gray(f_depth_rect);
disparityMap = disparity( ...
    f_gray, f_depth_gray, ...
    'Method', DISPMETHOD, ...
    'DisparityRange', [0 DISPRANGE]);
 
% display disparityMap
%figure;
%imshow(disparityMap, [0,64]);
%title('Map');
%colormap jet 
%colorbar
 
%% Reconstruct the 3-D Scene
points3D = reconstructScene(disparityMap, stereoParams);
points3D = points3D ./ 1000; % Convert to meters and create a pointCloud object

% show ptCloud
%ptCloud = pointCloud(points3D, 'Color', f_rect);
%player3D = pcplayer([-1, 1], [-0.4, 0.225], [0.75, 1.75], 'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
%view(player3D, ptCloud);
 
 
%% detect objects
%faceDetector = vision.CascadeObjectDetector('FrontalFaceLBP'); % 'ProfileFace'
noseDetector = vision.CascadeObjectDetector('Nose');
righteyeDetector = vision.CascadeObjectDetector('RightEye');
lefteyeDetector = vision.CascadeObjectDetector('LeftEye');

load('bottleDetector.mat'); % detects FitnessOnline.fi bottle 
load('calcDetector.mat'); % detects Calculator

bboxes_nose = noseDetector(f_gray);
bboxes_lefteye = lefteyeDetector(f_gray);
bboxes_righteye = righteyeDetector(f_gray);

% remove the bboxes that are invalid or NaN
[bboxes_nose, dists_nose, centroids3D_nose] = filterBbox(bboxes_nose,disparityMap,points3D,1);
[bboxes_lefteye, dists_lefteye, centroids3D_lefteye] = filterBbox(bboxes_lefteye,disparityMap,points3D,1);
[bboxes_righteye, dists_righteye, centroids3D_righteye] = filterBbox(bboxes_righteye,disparityMap,points3D,1);

[bboxes_bottle, scores_bottle] = getBboxes(bottleDetector, f_gray);
[bboxes_calc, scores_calc] = getBboxes(calcDetector, f_gray);

[bboxes_bottle, dists_bottle, centroids3D_bottle] = filterBbox(bboxes_bottle, disparityMap, points3D,2);
[bboxes_calc, dists_calc, centroids3D_calc] = filterBbox(bboxes_calc, disparityMap, points3D,2);
bboxes = bboxes_nose;
bboxes = vertcat(bboxes, bboxes_lefteye);
bboxes = vertcat(bboxes, bboxes_righteye);

% get the best bbox for the face
if size(bboxes,1) > 0 
    bboxes = bboxes(1,:);
end    

bboxes = vertcat(bboxes,bboxes_bottle);
bboxes = vertcat(bboxes, bboxes_calc);

dists = dists_nose;
dists = horzcat(dists, dists_lefteye); 
dists = horzcat(dists, dists_righteye);

% get the corresponding dist for the best bbox
if size(dists) > 0 
    dists = dists(:,1);
end 
dists = horzcat(dists, dists_bottle);
dists = horzcat(dists, dists_calc);


centroids3D = centroids3D_nose;
centroids3D = horzcat(centroids3D, centroids3D_lefteye);
centroids3D = horzcat(centroids3D, centroids3D_righteye);

if size(centroids3D) > 0 
    centroids3D = centroids3D(:,1);
end

centroids3D = horzcat(centroids3D, centroids3D_bottle);
centroids3D = horzcat(centroids3D, centroids3D_calc);

%% determine distance of face to camera
  
% display detected face and its distance
labels = cell(1, numel(dists));
for n = 1:numel(dists)
    labels{n} = sprintf('%0.2f meters', dists(n));
end

image = insertObjectAnnotation(f_rect, 'rectangle', bboxes, labels);

if i < 10
    baseFileName = sprintf('image000%d.png', i);     
elseif i < 100 && i >= 10
    baseFileName = sprintf('image00%d.png', i);
elseif i < 1000 && i >= 100
    baseFileName = sprintf('image0%d.png', i);
elseif i < 10000 && i >= 1000
    baseFileName = sprintf('image%d.png', i);
end
   
mkdir images_annotated
fname=strcat('images_annotated/',baseFileName);
imwrite(image, fullfile(fname), 'png');

end


function [bboxes, scores] = getBboxes(detector, img)
    [bboxes, scores] = detect(detector, img);
    maximum_score = 0;
    index = 0;
    for i = 1:length(scores)
        old_maximum = maximum_score;
        if old_maximum < scores(i)
            maximum_score = scores(i);
            index = i;
        end
    end
    if index == 0
       return;
    else
        bboxes = bboxes(index, :);
        scores = scores(index, :);
    end 
end

function [bboxes, dists, centroids3D] = filterBbox(bboxes,disparityMap,points3D, mode)

    if mode == 1
        % this is for face
        range = [1.2 1.4];
    else
        % this is for objects
        range = [0.5 1.2];
    end 
    
    centroids = [round(bboxes(:, 1) + bboxes(:, 3) / 2), round(bboxes(:, 2) + bboxes(:, 4) / 2)];
 
    % return linear index equivalents to row and column subscripts for specific matrix size
    centroidsIdx = sub2ind(size(disparityMap), centroids(:, 2), centroids(:, 1));

    X = points3D(:,:,1);
    Y = points3D(:,:,2);
    Z = points3D(:,:,3);
    % find the 3D world coordinates of centroids
    centroids3D = [X(centroidsIdx)'; Y(centroidsIdx)'; Z(centroidsIdx)'];
    
    % find distances from the camera in meters
    dists = sqrt(sum(centroids3D .^ 2));
  
    for n = 1 : size(dists,2)
        % remove nans
        if isnan(dists(n)) 
            bboxes(n,:) = -99;
            dists(:,n) = -99;
        % remove invalid ranges
        elseif or(range(1) > dists(n), range(2) < dists(n))
            bboxes(n,:) = -99;
            dists(:,n) = -99;
        end 
    end

    % remove the invalid rows
    bboxes(find(bboxes(:,1) == -99),:) = [];
    dists(dists == -99) = [];
    if(size(dists) >= 1)
        
        bboxes = bboxes(1,:);  
        dists = dists(1,1);
    end
    
end 
 