function expertToText(total_path,plane_points)
% By JLL

comment1 = '# Image list with two lines of data per image:';
comment2 = '# IMAGE_ID, IMAGE_NAME, CAMERA_ID';
comment3 = '# POINTS2D[] as (X, Y, POINT3D_ID)';
 
fid = fopen(total_path,'w');
fprintf(fid,'%s\n',comment1);
fprintf(fid,'%s\n',comment2);
fprintf(fid,'%s\n',comment3);
fclose(fid);

for i = 1:1:size(plane_points,1)
    image = plane_points{i};
    A = reshape([image.point2D,image.point3D_ids]',1,[]);
    fid = fopen(total_path,'a');
    fprintf(fid,'%.0f %s %.0f\n',image.image_id,image.image_name,image.camera_id);
    fclose(fid);
    writematrix(A,total_path,'WriteMode','append')
end