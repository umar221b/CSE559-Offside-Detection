function [hueThresholdLow, hueThresholdHigh, saturationThresholdLow, saturationThresholdHigh, valueThresholdLow, valueThresholdHigh] = SetThresholds(color)
    switch color
		case 'yellow'
			% Yellow
			hueThresholdLow = 0.10;
			hueThresholdHigh = 0.14;
			saturationThresholdLow = 0.4;
			saturationThresholdHigh = 1;
			valueThresholdLow = 0.8;
			valueThresholdHigh = 1.0;
		case 'green'
			% Green
			hueThresholdLow = 0.192;
			hueThresholdHigh = 0.291;
			saturationThresholdLow = 0.587;
			saturationThresholdHigh = 1;
			valueThresholdLow = 0.151;
			valueThresholdHigh = 0.657;
		case 'red'
			% Red.
			% actually cyan, the image will be inverted and checked for
			% cyan threshold. This helps us avoid the circularity of red's Hue range.
			hueThresholdLow = 0.467;
			hueThresholdHigh = 0.525;
			saturationThresholdLow = 0.123;
			saturationThresholdHigh = 0.547;
			valueThresholdLow = 0.805;
			valueThresholdHigh = 1.0;
		case 'white'
			% White
			hueThresholdLow = 0.0;
			hueThresholdHigh = 1;
			saturationThresholdLow = 0;
			saturationThresholdHigh = 0.36;
			valueThresholdLow = 0.7;
			valueThresholdHigh = 1.0;
        case 'blue'
			% Blue
			hueThresholdLow = 0.551;
			hueThresholdHigh = 0.749;
			saturationThresholdLow = 0.555;
			saturationThresholdHigh = 1;
			valueThresholdLow = 0.133;
			valueThresholdHigh = 0.664;
        case 'purple'
			% Purple
			hueThresholdLow = 0.76;
			hueThresholdHigh = 0.94;
			saturationThresholdLow = 0.33;
			saturationThresholdHigh = 0.67;
			valueThresholdLow = 0.1;
			valueThresholdHigh = 0.7;
		otherwise
			% do nothing
			hueThresholdLow = 0;
			hueThresholdHigh = 1;
			saturationThresholdLow = 0;
			saturationThresholdHigh = 1;
			valueThresholdLow = 0;
			valueThresholdHigh = 1;
    end
end