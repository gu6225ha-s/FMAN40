%% Close all figures and clear workspace
close all;
clear;

%% Load image
im = imread("../../datasets/4/images/IMG_4546.jpeg");

%% Define polygon, plane normal and camera parameters
poly_im = pextend([2760.67385826772 1540.42866242038; ...
    3243.24393700787 307.715286624203; ...
    1691.40015748032 1066.60445859873; ...
    1023.42157480315 1921.79936305732]');
n = [-0.0508115 -0.0681014 -0.0468912]';
R = [0.993566 0.0318944  0.108671; ...
    -0.036544 0.998488 0.0410662; ...
    -0.107197 -0.0447733 0.993229];
t = [-5.87574 -0.112385 -1.0327]';
K = [2921.17 0 2016; ...
    0 2885.2 1512;
    0 0 1];

%% Show image and polygon
figure(1);
imagesc(im);
hold on;
drawpoly(poly_im);
axis off;
set(gca,'LooseInset',get(gca,'TightInset'));
print('im-orig.eps','-depsc');

%% Define a coordinate system in the plane
c = projplane(mean(poly_im,2),K,R,t,n); % center point
z = -n; % TODO: check direction of normal
x = [z(2); -z(1); 0];
x = x/norm(x);
y = cross(z,x);
y = y/norm(y);

%% Compute homography from plane to original image
H = K*[R t]*[x y c; 0 0 1]; % plane -> image
Hinv = inv(H); % image -> plane

%% Adjust scale so that area remains constant
poly_plane = pflat(Hinv*poly_im);
s = sqrt(abs(polyarea(poly_im))/abs(polyarea(poly_plane)));
x = x/s;
y = y/s;
H = K*[R t]*[x y c; 0 0 1];
Hinv = inv(H);
poly_plane = pflat(Hinv*poly_im);

%% Warp using MATLAB's imwarp
lower = floor(min(poly_plane,[],2));
upper = ceil(max(poly_plane,[],2));
im_size = [upper(2)-lower(2) upper(1)-lower(1) 3];
Rout = imref2d(im_size,[lower(1) upper(1)],[lower(2) upper(2)]);
im_warp = imwarp(im,projective2d(Hinv'),'OutputView',Rout);

%% Draw warped image and polygon
figure(2);
imagesc(im_warp);
hold on;
drawpoly(poly_plane-lower);
axis off;
set(gca,'LooseInset',get(gca,'TightInset'));
print('im-warped.eps','-depsc');

%% Warp manually with for loops
im_warp2 = zeros(size(im_warp),'uint8');
for y = 1:size(im_warp2,1)
    for x = 1:size(im_warp2,2)
        p = round(pflat(H*[x+lower(1);y+lower(2);1]));
        if p(1) >= 1 && p(1) <= size(im,2) && p(2) >= 1 && p(2) <= size(im,1)
            im_warp2(y,x,:) = im(p(2),p(1),:);
        end
    end
end

%% Draw warped image and polygon
figure(3);
imagesc(im_warp2);
hold on;
drawpoly(poly_plane-lower);

%% Helper functions 
function drawpoly(P)
    for i = 1:size(P,2)
        j = 1+mod(i,size(P,2));
        plot([P(1,i) P(1,j)],[P(2,i) P(2,j)],'LineWidth',2);
    end
end