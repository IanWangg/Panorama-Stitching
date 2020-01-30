function [indexPairs, keypoints1, keypoints2] = compare(name1, name2)
rgb1 = imread(name1);
image1 = single(rgb2gray(rgb1));
rgb2 = imread(name2);
image2 = single(rgb2gray(rgb2));
[keypoints1,features1] = sift(image1,'Levels',4,'PeakThresh',5);

[keypoints2,features2] = sift(image2,'Levels',4,'PeakThresh',5);


[indexPairs, m] = matchFeatures(features1',features2', 'MatchThreshold', 10);
matchedPoints1 = single(keypoints1(1:2, indexPairs(:,1)));
matchedPoints2 = single(keypoints2(1:2, indexPairs(:,2)));

figure; ax = axes;

showMatchedFeatures(image1,image2,matchedPoints1',matchedPoints2', 'montage','Parent',ax);

keypoints1 = keypoints1';
keypoints2 = keypoints2';

end