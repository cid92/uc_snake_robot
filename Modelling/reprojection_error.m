close all;
clear all;
%Load saved data: camera parameter and frame range
load('cameraParams4.mat');
load('imagePoints.mat');
load('stat.mat');
imSize = [310 130 1010 500];

%% Green colour threshold settings
green1Min = 17.172;
green1Max = 71.815;

% Define thresholds for green 2 based on histogram settings
green2Min = -57.621;
green2Max = 1.110;

% Define thresholds for green 3 based on histogram settings
green3Min = 26.328;
green3Max = 51.616;
%% Red colour threshold settings
% Define thresholds for channel 1 based on histogram settings
% Define thresholds for red 1 based on histogram settings
red1Min = 40.925;
red1Max = 78.253;

% Define thresholds for red 2 based on histogram settings
red2Min = 9.160;
red2Max = 73.370;

% Define thresholds for red 3 based on histogram settings
red3Min = -8.946;
red3Max = 51.616;
%% Blue colour threshold settings
blue1Min = 51.418;
blue1Max = 71.986;

% Define thresholds for channel 2 based on histogram settings
blue2Min = -40.600;
blue2Max = 8.122;

% Define thresholds for channel 3 based on histogram settings
blue3Min = -44.905;
blue3Max = -25.782;
%% Reprojection Errors in pixel
% figure; showReprojectionErrors(cameraParams4);
% title('Reprojection Errors');
%% Compute Extrinsics
boardSize = [7,10];
worldPoints = cameraParams4.WorldPoints;
% Detect the checkerboard
imVirtual = imread(fullfile('121216/calibration','02.png'));
%figure, imshow(imVirtual);
[imVirtual, newOrigin] = undistortImage(imVirtual, cameraParams4, 'OutputView', 'full');
%figure, imshow(imVirtual);
[imagePoints, boardSize] = detectCheckerboardPoints(imVirtual);
% Commpute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams4);
%% Image Processing
fileDirectory = '121216/trajectory_error';
for k = 1:1:7
    jpgFilename = sprintf('00%d.png', k);
    fullFileName = fullfile(fileDirectory, jpgFilename);
    if exist(fullFileName, 'file')
        imageData = imread(fullFileName);
    else
        warningMessage = sprintf('Warning: image file does not exist:\n%s', fullFileName);
        uiwait(warndlg(warningMessage));
    end
    [imageData2, newOrigin1] = undistortImage(imageData, cameraParams4, 'OutputView', 'full'); % undistored image
    %figure, imshow(imageData2);
    imageData3 = imcrop(imageData2,imSize); % crop image
    %figure, imshow(imageData3);
    %% RGB to L*a*b
    imageData3 = im2double(imageData3);
    cform = makecform('srgb2lab', 'AdaptedWhitePoint', whitepoint('D65'));
    imageLab = applycform(imageData3,cform);
     %% Green detection
    imgreen = (imageLab(:,:,1) >= green1Min ) & (imageLab(:,:,1) <= green1Max) & ...
    (imageLab(:,:,2) >= green2Min ) & (imageLab(:,:,2) <= green2Max) & ...
    (imageLab(:,:,3) >= green3Min ) & (imageLab(:,:,3) <= green3Max); 
    %figure, imshow(imgreen);
    imgreen = bwareaopen(imgreen, 3, 4);
    se = strel('cube', 1);
     imgreen = imdilate(imgreen,se);
%    figure, imshow(imgreen)
    bw = bwlabel(imgreen, 8);
    statgreen = regionprops(bw, 'BoundingBox', 'Centroid');
    %% Red detection
    imred = (imageLab(:,:,1) >= red1Min ) & (imageLab(:,:,1) <= red1Max) & ...
    (imageLab(:,:,2) >= red2Min ) & (imageLab(:,:,2) <= red2Max) & ...
    (imageLab(:,:,3) >= red3Min ) & (imageLab(:,:,3) <= red3Max); 
    %figure, imshow(imred);
    %imred = bwareaopen(imred, 3, 4);
    se = strel('cube', 10);
    imred = imdilate(imred,se);
    %figure, imshow(imred)
    bw = bwlabel(imred, 8);
    statred = regionprops(bw, 'BoundingBox', 'Centroid');
    %% Blue detection
    imblue = (imageLab(:,:,1) >= blue1Min ) & (imageLab(:,:,1) <= blue1Max) & ...
    (imageLab(:,:,2) >= blue2Min ) & (imageLab(:,:,2) <= blue2Max) & ...
    (imageLab(:,:,3) >= blue3Min ) & (imageLab(:,:,3) <= blue3Max); 
    %figure, imshow(imblue);
 %   imblue = bwareaopen(imblue, 3, 4);
%     se = strel('cube', 6);
%     imblue = imdilate(imblue,se);
    %figure, imshow(imblue)
    bw = bwlabel(imblue, 8);
    statblue = regionprops(bw, 'BoundingBox', 'Centroid');
    
    
    
%     figure,
%     imshow(imageData2);%figure 3
%     hold on
    for object = 1:length(statgreen)
        gg = statgreen(object).BoundingBox + [imSize(1:2) 0 0];
        gc = statgreen(object).Centroid + imSize(1:2);
        posGreens(k,:,object) = gc;
        pixGreens(k,:,object) = gc;
%         rectangle('Position',gg,'EdgeColor','g','LineWidth',2)
%         plot(gc(1),gc(2), '-m+')
%         a=text(gc(1)+15,gc(2), strcat('X: ', num2str(round(gc(1))), ' Y: ', num2str(round(gc(2)))));
%         set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'green');
    end
    for object = 1:length(statred)
        gg = statred(object).BoundingBox + [imSize(1:2) 0 0];
        gc = statred(object).Centroid + imSize(1:2);
        posReds(k,:,object) = gc;
        pixReds(k,:,object) = gc;
    end    
    for object = 1:length(statblue)
        gg = statblue(object).BoundingBox + [imSize(1:2) 0 0];
        gc = statblue(object).Centroid + imSize(1:2);
        posBlues(k,:,object) = gc;
        pixBlues(k,:,object) = gc;
    end  

end
for M = 1:1:7
    if (posGreens(M,2,2) < posGreens(M,2,1))
        prev = posGreens(M,:,1);
        posGreens(M,:,1) = posGreens(M,:,2);
        posGreens(M,:,2) = prev;
    end
end

posGreens(:,:,1) = pointsToWorld(cameraParams4, R, t, posGreens(:,:,1));
posGreens(:,:,2) = pointsToWorld(cameraParams4, R, t, posGreens(:,:,2));
posReds(:,:,1) = pointsToWorld(cameraParams4, R, t, posReds(:,:,1));
posReds(:,:,2) = pointsToWorld(cameraParams4, R, t, posReds(:,:,2));
posBlues(:,:,1) = pointsToWorld(cameraParams4, R, t, posBlues(:,:,1));
posBlues(:,:,2) = pointsToWorld(cameraParams4, R, t, posBlues(:,:,2));
jpgFilename = 'background.png';
fullFileName = fullfile(fileDirectory, jpgFilename);
imageData4 = imread(fullFileName);
[imageData4, newOrigin1] = undistortImage(imageData4, cameraParams4, 'OutputView', 'full'); 
background = bgDetection(imageData4);
% figure
% imshow(background);
bw = bwlabel(background, 8);
statbg = regionprops(bw, 'BoundingBox', 'Centroid');
bgg = statbg(28).BoundingBox;
a1 = pointsToWorld(cameraParams4, R, t, bgg(1:2));
a2 = pointsToWorld(cameraParams4, R, t, [bgg(1)+bgg(3),bgg(2)+bgg(4)]);
figure('position', [500, 250, 750, 400]) 
view(90,-90)
axis equal
hold on
r = rectangle('Position',[a2(1)/10,a1(2)/10,(a1(1)-a2(1))/10,(a2(2)-a1(2))/10],'EdgeColor',[1 1 1],'LineWidth',2);
r.FaceColor = [1 1 0.5];
r.LineWidth = 3;
plot(posReds(:,1,1)/10,posReds(:,2,1)/10,'*r')
plot(posReds(:,1,2)/10,posReds(:,2,2)/10,'*r')
plot(posGreens(:,1,1)/10,posGreens(:,2,1)/10,'*g')
plot(posGreens(:,1,2)/10,posGreens(:,2,2)/10,'*g')
plot(posBlues(:,1,1)/10,posReds(:,2,1)/10,'*b')
plot(posBlues(:,1,2)/10,posBlues(:,2,2)/10,'*b')
xlim([-80 80])
ylim([-30 250])
xlabel('x (cm)')
ylabel('y (cm)')
hold off

% for I = 1:1:6
%     error(I,:) = [posGreens(I+1,1,1)-posGreens(I,1,1),posGreens(I+1,2,1)-posGreens(I,2,1)];
%     error2(I,:) = [posGreens(I+1,1,2)-posGreens(I,1,2),posGreens(I+1,2,2)-posGreens(I,2,2)];
% end
% errorY1 = (posGreens(:,2,2)+posGreens(:,2,1))/20;
% errorY = (sum(errorY1(2:length(errorY1))-errorY1(1:length(errorY1)-1))/6)-29.3
% errorX =  sum((abs(posGreens(:,1,1)-posGreens(:,1,2))/10-6.8))/7
posGreen1(:,:) = posGreens(:,:,1);
posGreen2(:,:) = posGreens(:,:,2);
posRed1(:,:) = posReds(:,:,1);
posRed2(:,:) = posReds(:,:,2);
posBlue1(:,:) = posBlues(:,:,1);
posBlue2(:,:) = posBlues(:,:,2);
% pixGreen1(:,:) = pixGreens(:,:,1);
% pixGreen2(:,:) = pixGreens(:,:,2);
% % 
% imageData3 = imcrop(imread(fullFileName),imSize);

% BW = edge(rgb2gray(imageData3),'Canny');
% figure,imshow(BW);
% [imageData3, newOrigin1] = undistortImage(imageData3, cameraParams4, 'OutputView', 'full'); 
% BW = edge(rgb2gray(imageData3),'Canny');
% figure,imshow(BW);