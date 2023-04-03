function result = planeMarker(images,images_txt)
% By JLL
result = cell(size(images,1),1);
polygon = cell(size(images,1),[]);
for i = 1:1:size(images,1)
    polygon_infor = struct;
    image_infor = images_txt(i);
    % Manually plane mark
    count_p = 1;
    f = images{i};
    fig = imagesc(f);
    hold on
    while isvalid(fig)
        title('Close the figure window manually to proceed to the next image')
        xlabel('Single click to creat a point, trace back the starting point to close a polygon') 
        h = drawpolygon;
        if ~isvalid(fig)
            polygon{i,count_p} = {};
            break
        else
            polygon{i,count_p} = h.Position;
            count_p = count_p + 1;
        end
    end

    polygon_infor.image_id = image_infor.image_id;
    polygon_infor.image_name = image_infor.name;
    polygon_infor.camera_id = image_infor.camera_id;

    pOfCI = polygon(i, :);
    nOfp = sum(~cellfun('isempty', pOfCI));
    polygon_infor.nOfPolygon = nOfp;
    polygon_infor.PolygonCoordinate = pOfCI(~cellfun('isempty',pOfCI));
    result{i} = polygon_infor;
end