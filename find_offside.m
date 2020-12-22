clear all
close all
clc
%% Reading the Video
file_name = 'fifa';
obj = VideoReader("input/" + file_name + ".mp4");
v = VideoWriter("outputs/" + file_name + "_out2","MPEG-4");
open(v);
skip_until = 40;

%% Detecting the Offside Line
for frame_index = 1 : 140 %fill in the appropriate number
    % exit if no more frames in video
    if (~hasFrame(obj))
        break
    end
    
    % if not first frame, then store previous frame in prev_img
    if (frame_index > skip_until)
        prev_img = img;
    end
    
    img = readFrame(obj);
    
    % skip a few frames
    if (frame_index < skip_until)
        continue
    end
    
    % calculate vanishing point.
    % user input required here, please mark the lines in the GUI.
    if (frame_index == skip_until)
%         imshow(img)
%         [x, y] = getpts;
%         points = [x, y];
%         close all
        
        % for testing
        % file: fifa
        points = [222,130;4.00000000000001,342;330,134;4.00000000000001,548;444,130;136,718;560,132;438,720;682,130;740,720;796,130;1008,718;914,130;1280,672;1036,130;1280,390;1158,130;1278,228];
        % file: fifa3
%         points = [110.000000000000,23.9999999999999;1.99999999999983,212;188.000000000000,27.9999999999999;23.9999999999998,448.000000000000;264.000000000000,27.9999999999999;154.000000000000,450.000000000000;348,23.9999999999999;314.000000000000,448.000000000000;424.000000000000,23.9999999999999;456,448.000000000000;508,23.9999999999999;616.000000000000,448.000000000000;590.000000000000,21.9999999999999;768.000000000000,448.000000000000;672.000000000000,19.9999999999999;798.000000000000,260];

        num_of_points = size(points, 1);
        num_of_lines = num_of_points / 2;
        
        m = zeros(num_of_lines, 1);
        c = zeros(num_of_lines, 1);
        k = 1;
        vp = zeros(2, 1);
        thetas = zeros(num_of_lines, 1);

        for j = 1:2:num_of_points
            m(k) = (points(j + 1, 2) - points(j, 2)) / (points(j + 1, 1) - points(j, 1));
            c(k) = -points(j, 1) * m(k) + points(j, 2);
            
            
            thetas(k) = rad2deg(atan((points(j + 1, 1) - points(j, 1)) / (points(j + 1, 2) - points(j, 2))));
            k = k + 1;
%             plot([points(j, 1) points(j + 1, 1)],[points(j, 2) points(j + 1, 2)],'Color','g','LineWidth', 2)
        end
        
%         point = [116,128;4.00000000000001,214];
%         theta = rad2deg(atan((point(2, 1) - point(1, 1)) / (point(2, 2) - point(1, 2))));

        for p = 1:num_of_lines
           for q = (p + 1):num_of_lines
               A = [-m(p), 1; -m(q), 1];
               b = [c(p); c(q)];
               vp = vp + A \ b;
           end
        end
        vp = int16(vp / (num_of_lines * (num_of_lines - 1) / 2));
        disp(vp)

        continue
    end
    
    % track for 19/20 frames, only 1 in every 20 frames we do the
    % actual detection. the rest 19 frames are tracked using KLT algorithm.

    % Note here that for the first time, detection runs and creates variable
    % S which contains all the bounding boxes and Team_Ids which contains
    % corresponding teams.
    if (frame_index > skip_until + 1 && mod(frame_index - skip_until, 10) ~= 0)
        f = figure('visible', 'off');
        imshow(img)
        left_most = 9999;
        for i = 1:size(S,1)
            BB = S(i).BoundingBox;
            if (BB(1) < 1)
                BB(1) = 1;
            end
            if (BB(2) < 1)
                BB(2) = 1;
            end
            if (BB(1) + BB(3) > size(img, 2))
                BB(3) = size(img, 2) - BB(1);
            end
            if (BB(2) + BB(4) > size(img, 1))
                BB(4) = size(img, 1) - BB(2);
            end
            S(i).BoundingBox = BB;
            points = detectMinEigenFeatures(rgb2gray(prev_img), 'ROI', BB);
            if (size(points, 1) == 0)
                disp('ERROR in points here')
                continue
            end
            pointImage = insertMarker(prev_img,points.Location, '+', 'Color', 'white');
            tracker = vision.PointTracker('MaxBidirectionalError', 1);
            initialize(tracker, points.Location, prev_img);         
            frame = img;
            [points, validity] = step(tracker, frame);
            mean_x = mean(points(:, 1));
            mean_y = mean(points(:, 2));
            BB(1) = floor(mean_x - BB(3) / 2);
            BB(2) = floor(mean_y - BB(4) / 2);
            S(i).BoundingBox = BB;
            img1 = insertMarker(frame, points(validity, :), '+');
            hold on;
            rectangle('Position',[BB(1), BB(2), BB(3), BB(4)],...
                    'LineWidth', 2, 'EdgeColor', 'red')
            if (Team_Ids(i) == 1)
            text(BB(1) - 2, BB(2) - 2, 'D_T');
            end
            if (Team_Ids(i) == 2)
                text(BB(1) - 2, BB(2) - 2, 'A_T');
            end
            %Calculating the last defender on the left side using
            %vanishing point. Same can be done symmetrically to the
            %right hand side as well.

            x1 = floor(BB(1) + BB(3) / 2);
            y1 = floor(BB(2) + BB(4));
            slope = (double(vp(2)) - y1) / (double(vp(1)) - x1);
            lx = (ly - vp(2)) / slope + vp(1);
            if (lx < left_most && Team_Ids(i) == 1)
                left_most = lx;
            end
            rectangle('Position',[BB(1), BB(2), BB(3), BB(4)],...
            'LineWidth', 2, 'EdgeColor', 'red')
        end
        plot([left_most,vp(1)], [ly ,vp(2)], 'y', 'LineWidth', 1)
        fig = getframe(gcf);
        writeVideo(v, fig);
        close(gcf);
        continue;
    end
    
    
%% Actual Detection starts (one every 20 frames).
    % preprocessing the image to grayscale
    img_valid = img;
    BW_img = rgb2gray(img_valid);
    Edge_img_orig = edge(BW_img, 'sobel');

%% Removing the TOP Boundary
    Edge_img = Edge_img_orig;
    
    start_angle = 89;
    end_angle = 89.99;
    theta_resolution = 0.01;
    
    % get lines using hough transform
    [hou, theta, rho] = hough(Edge_img_orig(1:floor(size(Edge_img_orig, 1) / 2),:), 'Theta', start_angle:theta_resolution:end_angle);
    peaks = houghpeaks(hou, 2, 'threshold', ceil(0.3 * max(hou(:))));
    lines = houghlines(Edge_img_orig(1:floor(size(Edge_img_orig, 1) / 2),:), theta, rho,peaks, 'FillGap', 5, 'MinLength', 7);    


    % find top-most row by comparing y values
    min_row = lines(1).point1(2);
    xy_long = [lines(1).point1; lines(1).point2];
    for k = 1:length(lines) 
        xy = [lines(k).point1; lines(k).point2];
        row_index = lines(k).point1(2);
        if (row_index < min_row)
            min_row = row_index; 
            xy_long = xy;
        end
    end

    % set all pixels above the line to black
    img_valid(1:xy_long(:, 2), :, :) = 0;
    BW_img(1:xy_long(:,2), :, :) = 0;
    Edge_img(1:xy_long(:,2), :, :) = 0;

%% Determining the actual play area
    % remove hud elements
    hud = [647, 64, 37, 204 ; 647, 1014, 37, 204]; % hardcoded - replace as needed
    img_valid(hud(1, 1):(hud(1, 1) + hud(1, 3)), hud(1, 2):(hud(1, 2) + hud(1, 4)), :) = 0;
    img_valid(hud(2, 1):(hud(2, 1) + hud(2, 3)), hud(2, 2):(hud(2, 2) + hud(2, 4)), :) = 0;

    [field, fieldSize, maskedRGBImage] = colorDetectionHSV(img_valid, 'green', 200);
    img_valid = maskedRGBImage;

%% Determining the players and Team_Ids
    % get players by color
    [teamAttack, teamAttackSize, ~] = colorDetectionHSV(img_valid, 'blue', 5);
    [teamDefense, teamDefenseSize, ~] = colorDetectionHSV(img_valid, 'red', 5);

    % put all players/teamids in one list
    S = [teamDefense; teamAttack];
    Team_Ids = [ones(teamDefenseSize,1); 2 * ones(teamAttackSize, 1)];

%% Mark the bounding boxes
    f = figure('visible','off');    
    figure();
    imshow(img)
    hold on;
    left_most = 9999;
    ly = size(img, 1);
    for i = 1:size(S, 1)
        BB = S(i).BoundingBox;
        %Accounting for static UI elements with team logos which disrupt
        %the detection. We know the locations of these static UI elements.
        %In real life, this need not be done as we can work with the raw
        %camera feed directly. Remove/change this portion to suit your needs.

        %         if(( BB(1)+(BB(3)/2)<115 || BB(1)+(BB(3)/2)>130) || (BB(2)+(BB(4)/2)<990 || BB(2)+(BB(4)/2)>1010)) 
% no need to account for them anymore because they are removed previously
% (called hud elements)
        if (Team_Ids(i) == 1)
            text(BB(1) - 2, BB(2) - 2, 'D');
            BB(4) = 1.5 * BB(4);
            S(i).BoundingBox(4) = BB(4);
        end
        if (Team_Ids(i) == 2)
            text(BB(1) - 2, BB(2) - 2, 'A');
        end

        x1 = floor(BB(1) + BB(3) / 2);
        y1 = floor(BB(2) + BB(4));
        slope = (double(vp(2)) - y1) / (double(vp(1)) - x1);
        lx = (ly - vp(2)) / slope + vp(1);
        if (lx < left_most && Team_Ids(i) == 1)
            left_most = lx;
        end
        rectangle('Position',[BB(1), BB(2), BB(3), BB(4)],...
        'LineWidth', 2, 'EdgeColor', 'red')
%         plot([x1, vp(1)], [y1, vp(2)], 'r', 'LineWidth', 1)
%         plot([lx, vp(1)], [ly, vp(2)], 'c', 'LineWidth', 1)
    end

    % plot the offside line, this is currently done for the left most
    % player, same can be repeated for the right most player as well.
    plot([left_most, vp(1)], [ly, vp(2)], 'c', 'LineWidth', 1)
    fig = getframe(gcf);
    writeVideo(v,fig);
    close(gcf);
end
close(v);
