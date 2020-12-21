function [team, teamSize] = colorDetectionHSV(img, color, minBlobSize)
    rgbImage = im2double(img);
   
    % Convert RGB image to HSV
    if (strcmp(color, 'red')) % invert and apply cyan hue threshold
        hsvImage = rgb2hsv(1 - rgbImage);
    else
        hsvImage = rgb2hsv(rgbImage);
    end

    % Extract out the H, S, and V images individually
    hImage = hsvImage(:, :, 1);
	sImage = hsvImage(:, :, 2);
	vImage = hsvImage(:, :, 3);
    
    % Assign the low and high thresholds for each color band.
    [hueThresholdLow, hueThresholdHigh, saturationThresholdLow, saturationThresholdHigh, valueThresholdLow, valueThresholdHigh] = SetThresholds(color);

    % Now apply each color band's particular thresholds to the color band

    hueMask = (hImage >= hueThresholdLow) & (hImage <= hueThresholdHigh);
	saturationMask = (sImage >= saturationThresholdLow) & (sImage <= saturationThresholdHigh);
	valueMask = (vImage >= valueThresholdLow) & (vImage <= valueThresholdHigh);

    % Combine the masks to find where all 3 are "true."
	% Then we will have the mask of only the red parts of the image.
    coloredObjectsMask = ceil(hueMask & saturationMask & valueMask);

    % filter out small objects.
    smallestAcceptableArea = minBlobSize;

    % Get rid of small objects.  Note: bwareaopen returns a logical.
    coloredObjectsMask = uint8(bwareaopen(coloredObjectsMask, smallestAcceptableArea));

    % Smooth the border using a morphological closing operation, imclose().
    structuringElement = strel('disk', 4);
	coloredObjectsMask = imclose(coloredObjectsMask, structuringElement);

    % Fill in any holes in the regions, since they are most likely red also.
    coloredObjectsMask = imfill(logical(coloredObjectsMask), 'holes');

    % You can only multiply integers if they are of the same type.
	% (coloredObjectsMask is a logical array.)
	% We need to convert the type of coloredObjectsMask to the same data type as hImage.
    coloredObjectsMask = cast(coloredObjectsMask, 'like', rgbImage); 
    
    % Use the colored object mask to mask out the colored-only portions of the rgb image.
    maskedImageR = coloredObjectsMask .* rgbImage(:,:,1);
	maskedImageG = coloredObjectsMask .* rgbImage(:,:,2);
	maskedImageB = coloredObjectsMask .* rgbImage(:,:,3);
    
    % Concatenate the masked color bands to form the rgb image.
	maskedRGBImage = cat(3, maskedImageR, maskedImageG, maskedImageB);
    
    [team, teamSize] = FindConnectedComponents(coloredObjectsMask);
end

