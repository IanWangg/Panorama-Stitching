function [] = showStitching(folderName, centerIdx)
ims = imageDatastore(folderName);

Images_count = numel(ims.Files);

tforms(Images_count) = projective2d(eye(3));
imSize = zeros(Images_count,2);
imgBase = readimage(ims,1);


for i = 1:Images_count-1
    I1 = readimage(ims, i);
    I2 = readimage(ims, i+1);    
    I_gray = rgb2gray(I2);    
    imSize(i,:) = size(I_gray);    
    H = RANSAC(I1, I2);
    H = inv(H);
    tforms(i + 1) = projective2d(H).T * tforms(i).T;
end
imSize(Images_count,:) = size(rgb2gray(readimage(ims, Images_count)));

%%
for i = 1:Images_count          
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imSize(i,2)], [1 imSize(i,1)]);
end
%%
avgXLim = mean(xlim, 2);
[~, idx] = sort(avgXLim);
centerImageIdx = idx(centerIdx);
Tinv = invert(tforms(centerImageIdx));
for i = 1:Images_count     
    tforms(i).T = tforms(i).T * Tinv.T;
end
for i = 1:Images_count         
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imSize(i,2)], [1 imSize(i,1)]);
end

%%
max_x = max(imSize(2));
max_y = max(imSize(1));
% Find the size of panorama
xMin = min([1; xlim(:)]);
xMax = max([5; xlim(:)]);
yMin = min([1; ylim(:)]);
yMax = max([max_y; ylim(:)]);

% Define the size of panorama
width  = round(xMax - xMin);
height = round(yMax - yMin);
panorama = zeros([height width 3], 'like', imgBase);
blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  
panoramaView = imref2d([height width], [xMin xMax], [yMin yMax]);
%%
% Create the panorama.
for i = 1:Images_count
    img_rgb = readimage(ims, i);
    
    % Transform I into the panorama.
    warpedImage = imwarp(img_rgb, tforms(i), 'OutputView', panoramaView);      
    % Generate a binary mask.    
    mask = imwarp(true(size(img_rgb,1),size(img_rgb,2)), tforms(i), 'OutputView', panoramaView);
    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
end

figure, imshow(panorama)

