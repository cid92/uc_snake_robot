% Extract the trajectory from video frames with camera calibration using
% L*a*b
% In the command window, insert posReds(:,1,:) for redx1,redx2
%                               posReds(:,2,:) for redy1,redy2
clc;
close all;
clear all;
%Load saved data: camera parameter and frame range
load('cameraParams3.mat');
load('021016/frame.mat');
load('imagePoints.mat');
load('stat.mat');
imSize = [420 185 1475 730];

%% Options
gait = 1; % Gait pattern
trial = 1; % Trial number
plotFigures = 0;
segMode = 1;
%% Range of Frame
I = frame(trial,1);
F = frame(trial,2);
%% Initialisation
eRed = 0;
eGreen = 0;
eBlue = 0; 
i = 0;
Step = 1;
falseNegative = 0;
falsePositive = 0;
%% Red colour threshold settings 
red1Min = 0.000;
red1Max = 100.000;
% Define thresholds for channel 2 based on histogram settings
red2Min = 8.819;
red2Max = 32.942;
% Define thresholds for channel 3 based on histogram settings
red3Min = -11.528;
red3Max = 14.328;

%% Green colour threshold settings
green1Min = 0.020;
green1Max = 100.000;

% Define thresholds for channel 2 based on histogram settings
green2Min = -15.561;
green2Max = -5.641;

% Define thresholds for channel 3 based on histogram settings
green3Min = -47.152;
green3Max = -15.398;

%% Blue colour threshold settings 
% Define thresholds for channel 1 based on histogram settings, L
blue1Min = 0.000;
blue1Max = 100.000;
% Define thresholds for channel 2 based on histogram settings, b 
blue2Min = 5.395;
blue2Max = 32.942;
% Define thresholds for channel 3 based on histogram settings, a
blue3Min = -56.733;
blue3Max = -24.028;

%% Gait detection
if gait == 1
    gatename = 'rolling';
elseif gait == 2
    gatename = 'rotation';
elseif gait == 3
    gatename = 'side_winding';
elseif gait == 4
    gatename = 'turning';
elseif gait == 5
    gatename = 'linear_progression';
end

%% Compute Extrinsics
boardSize = [7,10];
worldPoints = cameraParams3.WorldPoints;
% Detect the checkerboard
imVirtual = imread(fullfile('021016/calibration3','01.png'));
%figure, imshow(imVirtual);
[imVirtual, newOrigin] = undistortImage(imVirtual, cameraParams3, 'OutputView', 'full');
%figure, imshow(imVirtual);
[imagePoints, boardSize] = detectCheckerboardPoints(imVirtual);
% Commpute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams3);
%% Image Processing
fileDirectory = sprintf('021016/%s_0%d', gatename, trial);
for k = I:Step:F
    i = i+1;
    if k < 10
        jpgFilename = sprintf('00%d.png', k);
        fullFileName = fullfile(fileDirectory, jpgFilename);
    elseif k >= 10 && k < 100
        jpgFilename = sprintf('0%d.png', k);
        fullFileName = fullfile(fileDirectory, jpgFilename);
    elseif k >= 100 && k < 1000
         jpgFilename = sprintf('%d.png', k);
         fullFileName = fullfile(fileDirectory, jpgFilename);
    end
    if exist(fullFileName, 'file')
        imageData = imread(fullFileName);
    else
        warningMessage = sprintf('Warning: image file does not exist:\n%s', fullFileName);
        uiwait(warndlg(warningMessage));
    end
    %% Preprocessing image
    statred = stat;
    statblue = stat;
    statgreen = stat;
    k;
    i;
    %figure, imshow(imageData);
    [imageData2, newOrigin1] = undistortImage(imageData, cameraParams3, 'OutputView', 'full'); % undistored image
    %figure, imshow(imageData2);
    imageData3 = imcrop(imageData2,imSize); % crop image
%     imageData3 = imgaussfilt(imageData3, 2);
%     h = ones(2,2) / 1;
%     imageData3 = imfilter(imageData3,h);
    %figure,imshow(imageData3);
    
    %% Snake detection
     % Threshold with Otsu method.
    grayImage = rgb2gray(imageData3); % Convert to gray level
    %figure, imshow(grayImage);
    thresholdLevel = graythresh(grayImage); % Get threshold.
    %binaryImage = im2bw( grayImage, 0.38);
    binaryImage = im2bw( grayImage, thresholdLevel); % Do the binarization
    %figure, imshow(binaryImage);
    binaryImage = imcomplement(binaryImage);
    %figure, imshow(binaryImage); 
    se = strel('cube',8);
    binaryImage= imerode(binaryImage,se);
    binaryImage = bwareaopen(binaryImage, 10, 18);
    se = strel('sphere',10);
    binaryImage = imdilate(binaryImage,se); 
    bw = bwlabel(binaryImage, 8);
    statsnake = regionprops(bw, 'BoundingBox', 'Centroid');
    %figure, imshow(grayImage);   
    topRegion = logical(zeros(size(binaryImage)));
    midRegion = logical(zeros(size(binaryImage)));
    botRegion = logical(zeros(size(binaryImage)));
    x = round(statsnake.BoundingBox(1));
    y = round(statsnake.BoundingBox(2));
    dy = round(statsnake.BoundingBox(4)/3);
    dx = round(statsnake.BoundingBox(3));
    topRegion(y:y+dy,x:x+dx)= binaryImage(y:y+dy,x:x+dx);
    midRegion(y+dy:y+(2*dy),x:x+dx)= binaryImage(y+dy:y+(2*dy),x:x+dx);
    botRegion(y+(2*dy):y+(3*dy),x:x+dx)= binaryImage(y+(2*dy):y+(3*dy),x:x+dx);
%     figure, imshow(topRegion);
%     figure, imshow(midRegion);
%     figure, imshow(botRegion);
    %% RGB to L*a*b
    if segMode == 1
        imageLAB = rgb2lab(imageData3);
        imRed = redLAB(imageLAB)&botRegion;
        imGreen = greenLAB(imageLAB)&midRegion;
        imBlue = blueLAB(imageLAB)&topRegion;     
    else
        imageHSV = rgb2hsv(imageData3);
        imRed = redHSV(imageHSV)&botRegion;
        imGreen = greenHSV(imageHSV)&midRegion;
        imBlue = blueHSV(imageHSV)&topRegion;
    end
    %% Red detection 
    imRed = bwareaopen(imRed, 15, 4);
    %figure,imshow(imRed);
    bw = bwlabel(imRed, 8);
    statred = regionprops(bw, 'BoundingBox', 'Centroid');
    
   
    %% Green detection
    %figure, imshow(imGreen);
    imGreen = bwareaopen(imGreen, 30, 4);
    se = strel('cube', 4);
    imGreen = imdilate(imGreen,se);
    %figure, imshow(imGreen);
    bw = bwlabel(imGreen, 8);
    statgreen = regionprops(bw, 'BoundingBox', 'Centroid');
    
    %% Blue detection
    imBlue = bwareaopen(imBlue, 30, 4);
    se = strel('cube', 4);
    imBlue = imdilate(imBlue,se);
    %figure, imshow(imBlue);
    bw = bwlabel(imBlue, 8);
    statblue = regionprops(bw, 'BoundingBox', 'Centroid');
    %statblue2 = regionprops(bw, 'BoundingBox', 'Centroid');
    %L = length(statblue2);
    % Detect false blue
    snakeBounding = statsnake(1).BoundingBox;
    j = 1;
%     %Check the oreintation of the snake relative to the frame
%     if (snakeBounding(3) < snakeBounding(4)) % snake head in vertical axis
%         order(i) = 1;
%         bottomBoundry = statsnake(1).BoundingBox(2)+statsnake(1).BoundingBox(4)-100;
%         topBoundry = statsnake(1).BoundingBox(2)+100;
%         % Check whether the red markers at the bottom
%         if (statred(1).Centroid(2) > bottomBoundry && statred(2).Centroid(2) > bottomBoundry) % if the red at the bottom, the blue will be at the top        
%             for J = 1:1:L
%                 if (statblue2(J).Centroid(2) >  topBoundry)
%                     statblue2(J).Centroid(2) = -1;
%                 end
%             end
%             %disp('Red at bottom');
%         else % if the red at the top, the blue will be at the bottom
%             for J = 1:1:L
%                 if (statblue2(J).Centroid(2) <  bottomBoundry) %if blue marker is below the top blue boundry, make it false
%                     statblue2(J).Centroid(2) = -1;
%                 end
%             end  
%             %disp('Red at top');
%         end
%         %disp('Head parallel to the vertical axis');   
%     else % snake head in horizontal axis
%         order(i) = 2;
%         leftBoundry = statsnake(1).BoundingBox(1)+100;
%         rightBoundry = statsnake(1).BoundingBox(1)+statsnake(1).BoundingBox(3)-100;
%         % Check whether the red markers on the left side
%         if (statred(1).Centroid(1) < leftBoundry && statred(2).Centroid(1) < leftBoundry) % if the red on the left, the blue will be on the right                  
%             for J = 1:1:L
%                 if (statblue2(J).Centroid(1) <  rightBoundry)
%                     statblue2(J).Centroid(2) = -1;
%                 end
%             end
%             %disp('Red on left');
%         else % if the red on the right, the blue will be on the left
%             for J = 1:1:L
%                 if (statblue2(J).Centroid(1) >  leftBoundry) %if blue marker is more to , make it false
%                     statblue2(J).Centroid(2) = -1;
%                 end
%             end  
%             %disp('Red on right');
%         end
%         %disp('Head parallel to the horizontal axis'); 
%     end
%     %Check the distance between the markers to detect false markers.
%     for J = 1:1:L-1
%         for K = 1:1:L-J
%             if (statblue2(J).Centroid(2) ~= -1 || statblue2(K+J).Centroid(2) ~= -1)% any marker out of boundry is ignored
%                 d = distance(statblue2(J).Centroid,statblue2(K+J).Centroid);
%                 if d <55 && d >30 % the distance should be within the range
%                     statblue(1) = statblue2(J);
%                     statblue(2) = statblue2(K+J);
%                 end
%             end
%         end
%     end
    %% Check Bad Detection 
    noRed = length(statred);
    noGreen = length(statgreen);
    noBlue = length(statblue);
    if noRed ~= 2
        disp('Bad Red !!!');
        eRed = 1+eRed;
        redFrame(eRed) = k
        figure, imshow(imRed);
        noRed
        if noRed < 2
           falseNegative = falseNegative+1; 
        else
           falsePositive = falsePositive+1;
        end
    end
    if noGreen ~= 2
        disp('Bad Green !!!');
        eGreen = 1+eGreen;
        greenFrame(eGreen) = k
        figure, imshow(imGreen);
        noGreen
        if noGreen < 2
           falseNegative = falseNegative+1; 
        else
           falsePositive = falsePositive+1;
        end
    end
    if noBlue ~= 2
        disp('Bad Blue !!!');
        eBlue = 1+eBlue;
        blueFrame(eBlue) = k
        figure, imshow(imBlue);
        noBlue
        if noBlue < 2
           falseNegative = falseNegative+1; 
        else
           falsePositive = falsePositive+1;
        end
    end
    %% Display the image
    if plotFigures == true
        figure
        imshow(imageData2);%figure 3
        hold on
    end
    snakeDetection = size(statsnake);
    if snakeDetection == 1
        sr = statsnake(1).BoundingBox + [imSize(1:2) 0 0];
        sc = statsnake(1).Centroid + imSize(1:2);
        if plotFigures == true
            rectangle('Position',sr,'EdgeColor','y','LineWidth',2)
        end
    else
        disp('Bad Snake !!!');
        k
    end
    for object = 1:length(statred)
        rr = statred(object).BoundingBox + [imSize(1:2) 0 0];
        rc = statred(object).Centroid + imSize(1:2);
        posReds(i,:,object) = rc;
        if plotFigures == true
            posRed((k-I)/Step+1,:,object) = rc;
            rectangle('Position',rr,'EdgeColor','r','LineWidth',2)
            plot(rc(1),rc(2), '-m+')
            a=text(rc(1)+15,rc(2), strcat('X: ', num2str(round(rc(1))), ' Y: ', num2str(round(rc(2)))));
            set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'red');
        end
    end
    for object = 1:length(statgreen)
        gg = statgreen(object).BoundingBox + [imSize(1:2) 0 0];
        gc = statgreen(object).Centroid + imSize(1:2);
        posGreens(i,:,object) = gc;
        if plotFigures == true
            posGreen((k-I)/Step+1,:,object) = gc;
            rectangle('Position',gg,'EdgeColor','g','LineWidth',2)
            plot(gc(1),gc(2), '-m+')
            a=text(gc(1)+15,gc(2), strcat('X: ', num2str(round(gc(1))), ' Y: ', num2str(round(gc(2)))));
            set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'green');
        end


    end
    for object = 1:length(statblue)
        bb = statblue(object).BoundingBox + [imSize(1:2) 0 0];
        bc = statblue(object).Centroid + imSize(1:2);
        posBlues(i,:,object) = bc;
        if plotFigures == true
            posBlue((k-I)/Step+1,:,object) = bc;
            rectangle('Position',bb,'EdgeColor','b','LineWidth',2)
            plot(bc(1),bc(2), '-m+')
            a=text(bc(1)+15,bc(2), strcat('X: ', num2str(round(bc(1))), ' Y: ', num2str(round(bc(2)))));
            set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'blue');
        end
    end
    if plotFigures == true
        hold off
    end
end
for M = 1:1:F-I+1
%     if (order(M) == 1)
        if (posReds(M,2,2) < posReds(M,2,1))
            prev = posReds(M,:,1);
            posReds(M,:,1) = posReds(M,:,2);
            posReds(M,:,2) = prev;
        end
        if (posGreens(M,2,2) < posGreens(M,2,1))
            prev = posGreens(M,:,1);
            posGreens(M,:,1) = posGreens(M,:,2);
            posGreens(M,:,2) = prev;
        end
        if (posBlues(M,2,2) < posBlues(M,2,1))
            prev = posBlues(M,:,1);
            posBlues(M,:,1) = posBlues(M,:,2);
            posBlues(M,:,2) = prev;
        end
%     end
end

%Convert to the real world coordinate 
posReds(:,:,1) = pointsToWorld(cameraParams3, R, t, posReds(:,:,1))/10;
posGreens(:,:,1) = pointsToWorld(cameraParams3, R, t, posGreens(:,:,1))/10;
posBlues(:,:,1) = pointsToWorld(cameraParams3, R, t, posBlues(:,:,1))/10;
posReds(:,:,2) = pointsToWorld(cameraParams3, R, t, posReds(:,:,2))/10;
posGreens(:,:,2) = pointsToWorld(cameraParams3, R, t, posGreens(:,:,2))/10;
posBlues(:,:,2) = pointsToWorld(cameraParams3, R, t, posBlues(:,:,2))/10;

%Negates the x-axis
posReds(:,1,1) = -posReds(:,1,1);
posGreens(:,1,1) = -posGreens(:,1,1);
posBlues(:,1,1) = -posBlues(:,1,1);
posReds(:,1,2) = -posReds(:,1,2);
posGreens(:,1,2) = -posGreens(:,1,2);
posBlues(:,1,2) = -posBlues(:,1,2);

%Trajectory Plot
figure
axis equal,
hold on,
plot(posReds(:,1,1),posReds(:,2,1),'r')
plot(posReds(:,1,2),posReds(:,2,2),'r')
plot(posGreens(:,1,2),posGreens(:,2,2),'g')
plot(posGreens(:,1,1),posGreens(:,2,1),'g')
plot(posBlues(:,1,2),posBlues(:,2,2),'b')
plot(posBlues(:,1,1),posBlues(:,2,1),'b')
xlabel('X (cm)')
ylabel('Y (cm)')
h = findobj('Color','r');
g = findobj('Color','g');
i = findobj('Color','b');
v = [h(1) g(1) i(1)];
legend(v, 'Front', 'Middle', 'Rear');
hold off,





