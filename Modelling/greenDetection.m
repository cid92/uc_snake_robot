function [BW] = greenDetection(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder App. The colorspace and
%  minimum/maximum values for each channel of the colorspace were set in the
%  App and result in a binary mask BW and a composite image maskedRGBImage,
%  which shows the original RGB image values under the mask BW.

% Auto-generated by colorThresholder app on 07-Nov-2016
%------------------------------------------------------


% Convert RGB image to chosen color space
RGB = im2double(RGB);
cform = makecform('srgb2lab', 'AdaptedWhitePoint', whitepoint('D65'));
I = applycform(RGB,cform);

% % Define thresholds for channel 1 based on histogram settings
% channel1Min = 17.624;
% channel1Max = 73.641;
% 
% % Define thresholds for channel 2 based on histogram settings
% channel2Min = -15.081;
% channel2Max = -7.366;
% 
% % Define thresholds for channel 3 based on histogram settings
% channel3Min = -33.806;
% channel3Max = -3.087;

%Rolling_06
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.020;
channel1Max = 100.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = -15.561;
channel2Max = -1.641;

% Define thresholds for channel 3 based on histogram settings
channel3Min = -47.152;
channel3Max = -15.398;


% Create mask based on chosen histogram thresholds
BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
