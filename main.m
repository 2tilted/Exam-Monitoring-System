function main()
 
%% load stereo parameters
load('steoreoParams_classroom.mat');
 
%% load video files
v = VideoReader('corrected_video_left.avi'); % main channel
v_depth = VideoReader('corrected_video_right.avi'); % depth channel
dummy = VideoReader('corrected_video_left.avi');
 
%% get number of frames
numFrames = 0;
while hasFrame(dummy)
    readFrame(dummy);
    numFrames = numFrames + 1;
end
 
%% run through videos frame by frame
 
% create and open output file
v_processed = VideoWriter(['output', '.avi']);
open(v_processed);
 
% geometry
GDISTMAX = 0.15; % max distance from recognized object to vector [cm]
 
% time
TDELTAMAX = v.FrameRate * 5; % max time elapsed before creating next incident [s]
timeDelta = Inf;
 
TLENGTHMIN = v.FrameRate * 10; % min time elapsed before creating new incident
timeLength = 0;
 
incidentID = 1;
tempFrames = cell([],1);

headpose_csv = csvread('output.csv');

objs_3D_GLOBAL = [-1000 -1000; -1000 -1000; -1000 -1000];
objs_2D_GLOBAL = [-1000 -1000; -1000 -1000];
GLOBAL_C = [0;0;0];
 
% analyze video frame by frame
for i = 1:numFrames
    

    % getboth frames
    f = readFrame(v);
    f_depth = readFrame(v_depth);
     
    % get object positions and headpose vector
    [face_3D, face_2D] = getPos(f, f_depth, stereoParams, 'face');
    [objs_3D, objs_2D] = getPos(f, f_depth, stereoParams, 'objs');

    for it = 1:numel(objs_3D)
        val_3D = objs_3D(it);
        
        if ~isnan(val_3D)
            objs_3D_GLOBAL(it) = val_3D;
        end
    end
    
    for it = 1:numel(objs_2D)
        val_2D = objs_2D(it);
        
        if ~isnan(val_3D)
            objs_2D_GLOBAL(it) = val_2D;
        end
    end
    
    vec = headpose_csv(i,:);
    vec = vec';
    vec(2,1) = -1 * vec(2,1);
    
   if eq(face_3D, [NaN; NaN; NaN])
       writeVideo(v_processed, f);
       continue;
   end
     
    % get distance to closest object
    
    [dist, idx] = get3dDistance(face_3D, vec, objs_3D_GLOBAL);
    
    % does this qualify as incident with regard to ...
    isIncidentDistance = dist < GDISTMAX; % maximum geometric distance
    isIncidentDelta = timeDelta < TDELTAMAX; % maximum time elapsed since last incident
    isIncidentLength = timeLength >= TLENGTHMIN; % minimum duration per incident
    
    f = markFrame(f, face_2D, objs_2D_GLOBAL(:,2), vec);
    f = markFrame(f, face_2D, objs_2D_GLOBAL(:,1), vec);
     
    % if vector intersects object, mark frame
    if eq(isIncidentDistance, true)
         
        % set iterators
        timeDelta = 0;
        timeLength = timeLength + 1;  
         
        % mark frame and store it in tempFrames
        f = markFrame(f, face_2D, objs_2D_GLOBAL(:,idx), vec);
        tempFrames{end+1} = f;
    else
         
        % if vector does not intersect object and timeDelta is small 
        % enough, keep running
        if eq(isIncidentDelta, true)
             
            % set iterators
            timeDelta = timeDelta + 1;
            timeLength = timeLength + 1;
             
        else
             
            % if vector does not intersect object, timeDelta is too large 
            % and timeLength is large enough, create new incident
            if eq(isIncidentLength, true)
                 
                % create and save incident to disk
                createIncident(i-timeLength, tempFrames, incidentID, v)
                disp('we are creating an incident');
             
                % reset iterators
                incidentID = incidentID + 1;
                tempFrames = cell([],1);
                timeDelta = 0;
                timeLength = 0;
   
            % else, reset iterators
            else
                 
                % reset iterators
                timeDelta = Inf;
                timeLength = 0;
  
            end
   
        end
         
    end
     
    % catch last incident
    if eq(i, numFrames) && eq(isIncidentLength, true)
         
        % create and save incident to disk
        createIncident(i-timeLength, tempFrames)
      
    end
    
    baseFileName = sprintf('image%d.png', i);
   
    if ~exist('images_annotated', 'dir')
       mkdir images_annotated
    end
    
    fname=strcat('images_annotated/',baseFileName);
    imwrite(f, fullfile(fname), 'png');
 
    writeVideo(v_processed,f);
    
    % print progress
    disp(['progress: ', num2str(i/numFrames*100), '%']);
     
end
 
% close output file
close(v_processed);
 
%% clear all

end
 
 
 
 
%% functions
 
% get distance and position of object that is closest to headpose vector
% https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d
function [bestDist, bestIdx] = get3dDistance(face, vec, objs) 
    % temporary variables
    bestDist = Inf;
    bestIdx = 1;
 
    % specify two points as part of 3D line
    B = face;
    C = B + 1000 * vec;
    
    % run through all objects
    for i = 1:size(objs,2)
        
        P = objs(:,i);
        v_0 = face;
        v_1 = face + vec;
        
        tmp = norm(cross(v_1,(v_0 - P)));
        dist = tmp / norm(v_1);
        
        % get position of object with smallest distance
        if dist <= bestDist
            bestDist = dist;
            bestIdx = i;
        end
         
    end
     
end
 
% mark face and object in frame
function frame = markFrame(frame, face, obj, vec)
 
% get 2D positions of face and object
fX = face(1,1);
fY = face(2,1);

vec_x = vec(1,1);
vec_y = vec(2,1);

dirX = face(1,1) + vec_x;
dirY = face(2,1) + vec_y;

o_1X = obj(1,1);
o_1Y = obj(2,1);

pos = [fX fY; o_1X o_1Y; dirX dirY]; 

% insert Markers
frame = insertMarker( ...
    frame, pos, 'circle', ...
    'color', {'magenta', 'blue', 'green'}, ...
    'size', 25); 
end
 
% save frames and timestamp to disk
function createIncident(startsAt, tempFrames, incidentID, v)
 
% create new directory
mkdir(['temp/', num2str(incidentID)]); 
 
% save every 10th frame
for j=1:length(tempFrames)
    if eq(mod(j, 10), 0)        
        imwrite( ...
            tempFrames{j},...
            ['temp/', num2str(incidentID), '/snapshot', num2str(j), '.jpg']);
        
    %end
end
 
% calculate timeString
seconds = round(startsAt / v.Framerate);
H = floor(seconds/3600); secondsAfterH = mod(seconds, 3600);
M = floor(secondsAfterH/60); secondsAfterM = mod(seconds, 60);
S = secondsAfterM;
timeString = [num2str(H), ':', num2str(M), ':', num2str(S)];
 
% save timeString to new text file
fileID = fopen('startsAt.txt', 'w');
fprintf(fileID, timeString);
fclose(fileID);
     
end

% This function detects faces or objects, calculates and returns their 
% 3D position. In case depth on the Z axis could not be evaluated, it
% returns [NaN NaN NaN].
function [centroids3D, centroids] = getPos(f, f_depth, stereoParams, target)

%% read and rectify video frames
[f_rect, f_depth_rect] = rectifyStereoImages(f, f_depth, stereoParams);
 
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
 
%% Reconstruct the 3-D Scene
points3D = reconstructScene(disparityMap, stereoParams);
points3D = points3D ./ 1000; % Convert to meters and create a pointCloud object
 
%% check which kind of detection is requested
if strcmp(target, 'face')
    noseDetector = vision.CascadeObjectDetector('Nose');
    righteyeDetector = vision.CascadeObjectDetector('RightEye');
    lefteyeDetector = vision.CascadeObjectDetector('LeftEye');

    bboxes_nose = noseDetector(f_gray);
    bboxes_lefteye = lefteyeDetector(f_gray);
    bboxes_righteye = righteyeDetector(f_gray);

    % remove the bboxes that are invalid or NaN
    [bboxes_nose, dists_nose, centroids3D_nose, centroids_nose] = filterBbox(bboxes_nose,disparityMap,points3D,1);
    [bboxes_lefteye, dists_lefteye, centroids3D_lefteye, centroids_lefteye] = filterBbox(bboxes_lefteye,disparityMap,points3D,1);
    [bboxes_righteye, dists_righteye, centroids3D_righteye, centroids_righteye] = filterBbox(bboxes_righteye,disparityMap,points3D,1);
    
    bboxes = concMats(bboxes_nose,bboxes_lefteye, bboxes_righteye, 0);
    dists = concMats(dists_nose, dists_lefteye, dists_righteye,1);
    centroids3D = concMats(centroids3D_nose, centroids3D_lefteye, centroids3D_righteye, 1); 
    centroids = concMats(centroids_nose.', centroids_lefteye.', centroids_righteye.', 1);
    
elseif strcmp(target,'objs')
    
    load('bottleDetector.mat'); % detects FitnessOnline.fi bottle 
    load('calcDetector.mat'); % detects Calculator
    
    [bboxes_bottle, scores_bottle] = getBboxes(bottleDetector, f_gray);
    [bboxes_calc, scores_calc] = getBboxes(calcDetector, f_gray);
    
    [bboxes_bottle, dists_bottle, centroids3D_bottle, centroids_bottle] = filterBbox(bboxes_bottle, disparityMap, points3D,2);
    [bboxes_calc, dists_calc, centroids3D_calc, centroids_calc] = filterBbox(bboxes_calc, disparityMap, points3D,2);

    bboxes = bboxes_bottle;
    bboxes = vertcat(bboxes, bboxes_calc);
    dists = dists_bottle;
    dists = horzcat(dists, dists_calc);
    centroids3D = centroids3D_bottle;
    centroids3D = horzcat(centroids3D, centroids3D_calc);
    centroids = centroids_bottle.';
    centroids = horzcat(centroids, centroids_calc.');

end 

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

function [bboxes, dists, centroids3D, centroids] = filterBbox(bboxes,disparityMap,points3D, mode)

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
    
    centroids = [round(bboxes(:, 1) + bboxes(:, 3) / 2), round(bboxes(:, 2) + bboxes(:, 4) / 2)];

end 

% orientation : 0 = vertical; 1 = horizontal
function mat = concMats(mat1, mat2, mat3, orientation)
   
    mat = mat1;
   
    if orientation == 0
        mat = vertcat(mat, mat2);
        mat = vertcat(mat, mat3);
        if size(mat,1) > 0 
            mat = mat(1,:);
    	end   
    elseif orientation == 1
        mat = horzcat(mat, mat2);
        mat = horzcat(mat, mat3);
        
        if size(mat) > 0 
            mat = mat(:,1);
    	end   
    end
    
end 
 


