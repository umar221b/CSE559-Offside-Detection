function [team, teamSize] = FindConnectedComponents(maskImage)
    mask = imfill(maskImage, 'holes'); 
    mask_open = bwareaopen(mask, 20);
    mask_open = imfill(mask_open, 'holes'); 
    mask_open = bwareafilt(mask_open, 20);
    
    S_E_D = strel('disk', 15);
    mask_open = imdilate(mask_open, S_E_D);
    
    conn_components = bwconncomp(mask_open, 8);
    team = regionprops(conn_components, 'BoundingBox', 'Area');
    teamSize = size(team, 1);
end