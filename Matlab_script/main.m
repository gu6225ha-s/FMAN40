clear;close;clc;

%% Read images
% Set this path to your image folder 
path_images = 'F:\COLMAP_project\testSet\Test_1_28_3\images';
imds = imageDatastore(fullfile(path_images),...
"FileExtensions",".jpg","LabelSource","foldernames");
images = readall(imds);
%% Read the COLMAP txt files
% Set this path to your COLMAP resulting .txt files folder
path_COLMAP = 'F:\COLMAP_project\testSet\Test_1_28_3\txts';
[cameras_txt, images_txt, points3D_txt] = read_model(path_COLMAP);

%% Mark the planes and find points within it
polygon = planeMarker(images,images_txt);

%% Write the result as a text file
% Set this path to where you what export the result
folder_path = 'E:\MSc_Lund_Machine_Learning\Semester_2\Project\Matlab_script';
file_name = 'Polygons.txt';
total_path = append(folder_path,'\',file_name);
expertPolygon(total_path,polygon)
