function expertPolygon(total_path,polygon)
% By JLL
comment1 = '# Image list with multiple lines of data per image:';
comment2 = '# IMAGE_ID, IMAGE_NAME, CAMERA_ID, The number of polygons contained';
comment3 = '# Coordinates of the 1:th polygon as (X1, Y1, X2, Y2, ...)';
comment4 = '# Coordinates of the 2:th polygon as (X1, Y1, X2, Y2, ...)';
comment5 = '# ........................................................';
comment6 = '# Coordinates of the n:th polygon as (X1, Y1, X2, Y2, ...)';

fid = fopen(total_path,'w');
fprintf(fid,'%s\n',comment1);
fprintf(fid,'%s\n',comment2);
fprintf(fid,'%s\n',comment3);
fprintf(fid,'%s\n',comment4);
fprintf(fid,'%s\n',comment5);
fprintf(fid,'%s\n',comment6);
fclose(fid);

for i = 1:1:size(polygon,1)
    image = polygon{i};
    n = image.nOfPolygon;
    if n ~= 0
        fid = fopen(total_path,'a');
        fprintf(fid,'%.0f %s %.0f %.0f\n',image.image_id,image.image_name,image.camera_id,image.nOfPolygon);
        fclose(fid);
        for j = 1:1:n
            cpc = image.PolygonCoordinate{j};
            A = reshape(cpc',1,[]);
            writematrix(A,total_path,'WriteMode','append')
        end
    end
end