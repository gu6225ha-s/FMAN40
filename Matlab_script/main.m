clear;close;clc;

%% Read images
% Set this path to your image folder 
path_images = '../datasets/1/images';
imds = imageDatastore(fullfile(path_images),...
"FileExtensions",[".jpg",".jpeg"],"LabelSource","foldernames");
images = readall(imds);

%% Mark the planes and find points within it
polygon = planeMarker(images,imds.Files);

%% Write the result as a text file
% Set this path to where you what export the result
total_path = '../datasets/1/polygons/poly1.txt';
exportPolygon(total_path,polygon)