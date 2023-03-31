function plane_point = planeMarker(images,images_txt)
% By JLL

plane_corners = cell(size(images));
plane_point = cell(size(images));
for i = 1:1:size(images,1)
    plane = struct;
    image_infor = images_txt(i);
    image_2D = image_infor.xys;
    % Manual plane marking
    f = images{i};
    imagesc(f)
    title('Finish a polygon to continue,close the figure window manually when the image no longer changes')
    xlabel(['Single click to creat a point, trace back the starting point to close a polygon'])
    h = drawpolygon;
    waitfor(f)
    % Find points within the polygon
    plane_corners{i} = h.Position;
    poly = plane_corners{i};
    [in,edge] = inpolygon(image_2D(:,1),image_2D(:,2),poly(:,1),poly(:,2));
    in = in + edge;

    plane.image_id = image_infor.image_id;
    plane.image_name = image_infor.name;
    plane.camera_id = image_infor.camera_id;
    plane.point2D = image_2D(in == 1,:);
    tem = image_infor.point3D_ids;
    plane.point3D_ids = tem(in == 1,:);
    plane_point{i} = plane;
end