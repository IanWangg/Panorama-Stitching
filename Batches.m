function [orientation, magnitude, weighted, siftNeighbourhood] = Batches(keypoints, scalelevel)
m = keypoints(1);
n = keypoints(2);
m = m+8;
n = n+8;
scalelevel = padarray(scalelevel,[8 8],0 ,'both');
siftNeighbourhood = scalelevel(m-8:m+8, n-8:n+8);

orientation = zeros(17, 17);
magnitude = zeros(17, 17);
weighted = zeros(17, 17);
for i = -8:8
    for j = -8:8
        mag = (abs(double(scalelevel(m+i+1, n+j) - scalelevel(m+i-1, n+j).^2 + (double(scalelevel(m+i, n+j+1) - scalelevel(m+i, n+j-1)).^2)))).^(1/2);
        theta = atand(double(scalelevel(m+i, n+j+1) - scalelevel(m+i, n+j-1)) ./ double(scalelevel(m+i+1, n+j) - scalelevel(m+i-1, n+j)));
        magnitude(i+9, j+9) = mag;
        orientation(i+9, j+9) = theta;
    end
end
[magnitude, orientation] = imgradient(siftNeighbourhood, 'central');
filter = fspecial('gaussian', [17 17], 2);
weighted = magnitude .* filter;
    