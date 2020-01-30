function H_best = RANSAC(im0, im1)

image_0 = single(rgb2gray(im0));
image_1 = single(rgb2gray(im1));
[keypoints1,features0] = sift(image_0,'Levels',4,'PeakThresh',5);
[keypoints2,features1] = sift(image_1,'Levels',4,'PeakThresh',5);

point_loc1 = keypoints1(1:2,:)';
point_loc2 = keypoints2(1:2,:)';

IndexPairs = matchFeatures(transpose(features0),transpose(features1), 'MatchThreshold', 10);
[row,~] = size(IndexPairs);
% Loop for the best result
num_best = -1;
for i=1:1000
    randIdx = randperm(row,4);
    chosePoints = IndexPairs(randIdx(:),:);
    
    % Homography
    H = solveHomo(point_loc1,point_loc2,chosePoints);
    % Consensus
    [num, inliers] = consensusSetCount(H, row, randIdx, point_loc1, point_loc2, IndexPairs,1);
    
    if num > num_best
        num_best = num;
        inliers_best = inliers;
    end
end

H_best = solveHomo_step4(inliers_best);

end


function H = solveHomo(points_loc1, points_loc2, IndexPairs)

% Homography
A = [];
for i=1:4
    x_0 = points_loc1(IndexPairs(i,1),1);
    y_0 = points_loc1(IndexPairs(i,1),2);
    x_1 = points_loc2(IndexPairs(i,2),1);
    y_1 = points_loc2(IndexPairs(i,2),2);
    
    A = [A; x_0 y_0 1 0 0 0 -x_1*x_0 -x_1*y_0 -x_1;
            0 0 0 x_0 y_0 1 -y_1*x_0 -y_1*y_0 -y_1];
end

[U,S,V] = svd(A);
H=V(:,end);
H=reshape(H,3,3);
end


function [num, inliers] = consensusSetCount(H, row, vector_rand, vp0, vp1, pairs, threshold)

num = 0;
inliers = [];
for i=1:row
   % skip the random points
   if ismember(i,vector_rand) == 1
       continue;
   end
   % calculate the distance
   x = vp0(pairs(i,1),1);
   y = vp0(pairs(i,1),2);
   xp = vp1(pairs(i,2),1);
   yp = vp1(pairs(i,2),2);
   
   projected_point = [x y 1]*H;
   xep = projected_point(1,1)/projected_point(1,3);
   yep = projected_point(1,2)/projected_point(1,3);
   if sqrt((xp-xep)^2+(yp-yep)^2) < threshold
       num = num + 1;
       inliers = [inliers; x y xp yp xep yep];
   end
end

end




function H = solveHomo_step4(IndexPairs)
[row,~]=size(IndexPairs);
% Homography
A = [];
for i=1:row
    x_0 = IndexPairs(i,1);
    y_0 = IndexPairs(i,2);
    x_1 = IndexPairs(i,3);
    y_1 = IndexPairs(i,4);
    
    A = [A; x_0 y_0 1 0 0 0 -x_1*x_0 -x_1*y_0 -x_1;
            0 0 0 x_0 y_0 1 -y_1*x_0 -y_1*y_0 -y_1];
end

[U,S,V] = svd(A);
H=V(:,end);
H=reshape(H,3,3);

end