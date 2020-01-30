function [Best_H, finalInlier] = RANSAC(name1, name2, threshold, newThreshold)
%name1 = 'trot3.png';
%name2 = 'trot4.png';
N = 400;             %numebr of iteration of ransac
%threshold = 5000;
[indexPairs, keypoints1, keypoints2] = compare(name1, name2);     %all the matching we get
A = zeros(8, 9);                %matrix
H = zeros(3, 3);                %transform matrix H
H_1 = zeros(3, 3);
inlier = 0;                     %number of inliers
consensusSet = zeros(0, 0);          %consensus set initialization
maxInlier = 0;
maxConsensusSet = zeros(0, 0);
for ite = 1:N
    %now we randomly pick 4 matching in each iteration
    inlier = 0;
    consensusSet = zeros(0, 0);
    idx = randperm(size(indexPairs, 1), 4);
    rows = 1;
    %construct A
    pts1 = keypoints1(indexPairs(idx, 1), 1:2);
    pts1 = pts1';
    pts2 = keypoints2(indexPairs(idx, 2), 1:2);
    pts2 = pts2';
    n = size(pts1,2);
    A = zeros(2*n,9);
    A(1:2:2*n,1:2) = pts1';
    A(1:2:2*n,3) = 1;
    A(2:2:2*n,4:5) = pts1';
    A(2:2:2*n,6) = 1;
    x1 = pts1(1,:)';
    y1 = pts1(2,:)';
    x2 = pts2(1,:)';
    y2 = pts2(2,:)';
    A(1:2:2*n,7) = -x2.*x1;
    A(2:2:2*n,7) = -y2.*x1;
    A(1:2:2*n,8) = -x2.*y1;
    A(2:2:2*n,8) = -y2.*y1;
    A(1:2:2*n,9) = -x2;
    A(2:2:2*n,9) = -y2;
    
    
    [U,S,V] = svd(A);
    H=V(:,end);
    H=reshape(H,3,3);
    H = H';
    for i = 1:size(indexPairs, 1)
        p1 = [keypoints1(indexPairs(i, 1), 1:2), 1];        %row vector, homogeneous coordinate
        p2 = [keypoints2(indexPairs(i, 2), 1:2), 1];        %row vector, homogeneous coordinate
        p2new = H*p1';
        p2new = p2new/p2new(3);                             %col vector
        p2new = p2new';                                     %being row vector now
        distance = sqrt(sum((p1 - p2new).^2, 'all'));                         %calculate the distance
        if distance < threshold
            inlier = inlier + 1;
            consensusSet = [consensusSet, i];
        end
        
        
    end
    if inlier >= maxInlier
       maxInlier = inlier;
       maxConsensusSet = consensusSet;
       H_1 = H;
    end
    
end
%%
pts1 = keypoints1(indexPairs(maxConsensusSet, 1), 1:2);
pts1 = pts1';
pts2 = keypoints2(indexPairs(maxConsensusSet, 2), 1:2);
pts2 = pts2';
n = size(pts1,2);
A = zeros(2*n,9);
A(1:2:2*n,1:2) = pts1';
A(1:2:2*n,3) = 1;
A(2:2:2*n,4:5) = pts1';
A(2:2:2*n,6) = 1;
x1 = pts1(1,:)';
y1 = pts1(2,:)';
x2 = pts2(1,:)';
y2 = pts2(2,:)';
A(1:2:2*n,7) = -x2.*x1;
A(2:2:2*n,7) = -y2.*x1;
A(1:2:2*n,8) = -x2.*y1;
A(2:2:2*n,8) = -y2.*y1;
A(1:2:2*n,9) = -x2;
A(2:2:2*n,9) = -y2;

[U,S,V] = svd(A);
H=V(:,end);
H=reshape(H,3,3);
H = H';
Best_H = H;

if size(Best_H, 1) == 9
    if size(Best_H, 2) == 1
        Best_H = reshape(Best_H, 3, 3);
        Best_H = Best_H';
    end
end
finalConsensusSet = zeros(0, 0);
finalInlier = 0;
%newThreshold = 150000;
for i = 1:size(indexPairs, 1)
    p1 = [keypoints1(indexPairs(i, 1), 1:2), 1];        %row vector, homogeneous coordinate
    p2 = [keypoints2(indexPairs(i, 2), 1:2), 1];        %row vector, homogeneous coordinate
    p2new = Best_H*p1';
    p2new = p2new/p2new(3);                             %col vector
    p2new = p2new';                                     %being row vector now
    distance = sqrt(sum((p1 - p2new).^2, 'all'));                         %calculate the distance
    if distance < newThreshold
       finalInlier = finalInlier + 1;
       finalConsensusSet = [finalConsensusSet, i];
    end
end


image1 = imread(name1);
image2 = imread(name2);


figure();
showMatchedFeatures(image1,image2,keypoints1(indexPairs(finalConsensusSet, 1), 1:2),keypoints2(indexPairs(finalConsensusSet, 2), 1:2), 'montage','Parent',axes);


%figure();
%append = ones(length(finalConsensusSet), 1);
%keypoints3 = [keypoints1(indexPairs(finalConsensusSet, 1), 1:2), append];
%keypoints3 = keypoints3';
%keypoints3 = single(Best_H*keypoints3);
%keypoints3(1:2, :) = keypoints3(1:2, :)./keypoints3(3, :);
%showMatchedFeatures(image1,image2,keypoints1(indexPairs(finalConsensusSet, 1), 1:2), keypoints3(1:2, :)', 'montage','Parent',axes);

