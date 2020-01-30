rgb1 = imread('3.png');
image1 = single(rgb2gray(rgb1));
rgb2 = imread('4.png');
image2 = single(rgb2gray(rgb2));
[keypoints1,features1] = sift(image1,'Levels',4,'PeakThresh',5);
subplot(1,2,1)
imshow(rgb1);hold on;
viscircles(keypoints1(1:2,:)',keypoints1(3,:)');

theta = 1;
[keypoints2,features2] = sift(image2,'Levels',4,'PeakThresh',5);
subplot(1,2,2)
imshow(rgb2);hold on;
viscircles(keypoints2(1:2,:)',keypoints2(3,:)');
%%

[indexPairs, m] = matchFeatures(features1',features2');
matchedPoints1 = single(keypoints1(1:2, indexPairs(:,1)));
matchedPoints2 = single(keypoints2(1:2, indexPairs(:,2)));
%%
figure; ax = axes;
showMatchedFeatures(image1,image2,matchedPoints1',matchedPoints2', 'montage','Parent',ax);
title(ax, 'Candidate point matches');
legend(ax, 'Matched points 1','Matched points 2');

