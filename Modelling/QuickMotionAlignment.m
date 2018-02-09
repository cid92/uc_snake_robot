%Chapter 6
% Realign the motion
clc;
clear all;
close all;
gatename = 'RL';
trial = 2;
posFilename = sprintf('pos%s00%d.mat', gatename, trial);
load(posFilename);

% %For raw data
% posRedA = [posRed(1:7,:), ones(7,1)];
% posGreenA = [posGreen(1:7,:), ones(7,1)];
% posBlueA = [posBlue(1:7,:), ones(7,1)];
% L = length(posRedA);
% posRedB = [posRed(7:13,:)-[posGreen(7,1)*ones(7,1), posGreen(7,2)*ones(7,1)],ones(7,1)];
% posGreenB = [posGreen(7:13,:)-[posGreen(7,1)*ones(7,1), posGreen(7,2)*ones(7,1)],ones(7,1)];
% posBlueB = [posBlue(7:13,:)-[posGreen(7,1)*ones(7,1), posGreen(7,2)*ones(7,1)],ones(7,1)];
% posRedA(:,2) =-posRedA(:,2);
% posRedB(:,2) =-posRedB(:,2);
% posGreenA(:,2) =-posGreenA(:,2);
% posGreenB(:,2) =-posGreenB(:,2);
% posBlueA(:,2) =-posBlueA(:,2);
% posBlueB(:,2) =-posBlueB(:,2);

% For simulation
posRedA = [posRed(1:7,:)-[posGreen(1,1)*ones(7,1), posGreen(1,2)*ones(7,1)], ones(7,1)];
posGreenA = [posGreen(1:7,:)-[posGreen(1,1)*ones(7,1), posGreen(1,2)*ones(7,1)], ones(7,1)];
posBlueA = [posBlue(1:7,:)-[posGreen(1,1)*ones(7,1), posGreen(1,2)*ones(7,1)], ones(7,1)];
L = length(posRedA);
posRedB = [posRed(8:14,:)-[posGreen(8,1)*ones(7,1), posGreen(8,2)*ones(7,1)],ones(7,1)];
posGreenB = [posGreen(8:14,:)-[posGreen(8,1)*ones(7,1), posGreen(8,2)*ones(7,1)],ones(7,1)];
posBlueB = [posBlue(8:14,:)-[posGreen(8,1)*ones(7,1), posGreen(8,2)*ones(7,1)],ones(7,1)];
posRedA(:,1) =-posRedA(:,1);
posRedB(:,1) =-posRedB(:,1);
posGreenA(:,1) =-posGreenA(:,1);
posGreenB(:,1) =-posGreenB(:,1);
posBlueA(:,1) =-posBlueA(:,1);
posBlueB(:,1) =-posBlueB(:,1);

%% Overall motion
%Forward motion
[dRed, rRed] = cart2pol(posRedA(:,1), posRedA(:,2));
[dBlue, rBlue] = cart2pol(posBlueA(:,1), posBlueA(:,2));
[dGreen, rGreen] = cart2pol(posGreenA(1,1), posGreenA(1,2));
if (strcmp(gatename, 'LP') == 1)
    fixAngle = (abs(dRed(1))-(abs(dRed(1))+pi()-abs(dBlue(1)))/2);
elseif (strcmp(gatename, 'TN') == 1)
    fixAngle = -(dRed(1)-(abs(dRed(1))+abs(dBlue(1)))/2);
    %fixAngle = pi()/2+((abs(dRed(1))+pi()-abs(dBlue(1)))/2);
elseif (strcmp(gatename, 'RL') == 1)
    fixAngle = (abs(dRed(1))-(abs(dRed(1))+pi()-abs(dBlue(1)))/2);
elseif (strcmp(gatename, 'RT') == 1)
    fixAngle = -dRed(1);
elseif (strcmp(gatename, 'SW') == 1)
    %fixAngle = pi()/2+(-dRed(1)+(abs(dRed(1))+abs(dBlue(1)))/2);
    fixAngle = -((abs(dRed(1))+pi()-abs(dBlue(1)))/2);
end
%fixAngle = 0;
posRedA = (rot(fixAngle)*posRedA')';
posGreenA = (rot(fixAngle)*posGreenA')';
posBlueA = (rot(fixAngle)*posBlueA')';
[dRedAf, rRedAf] = cart2pol(posRedA(L,1), posRedA(L,2));
[dGreenAf, rGreenAf] = cart2pol(posGreenA(L,1), posGreenA(L,2));
posDirRedA = posRedA-posGreenA; 
[dRedA, rRedA] =  cart2pol(posDirRedA(:,1),posDirRedA(:,2));

% Reverse Motion
[dRed, rRed] = cart2pol(posRedB(:,1), posRedB(:,2));
[dBlue, rBlue] = cart2pol(posBlueB(:,1), posBlueB(:,2));
if (strcmp(gatename, 'LP') == 1)
     fixAngle = (abs(dRed(1))-(abs(dRed(1))+pi()-abs(dBlue(1)))/2); %when dBlue > 0
elseif (strcmp(gatename, 'TN') == 1)
    fixAngle = -pi()/2-(abs(dRed(1))+(abs(dRed(1))+pi()-abs(dBlue(1)))/2);
elseif (strcmp(gatename, 'RL') == 1)
    fixAngle = (abs(dRed(1))-(abs(dRed(1))+pi()-abs(dBlue(1)))/2);
    %fixAngle = ((abs(dRed(1))+pi()-abs(dBlue(1)))/2);
elseif (strcmp(gatename, 'RT') == 1)
    fixAngle = -dRed(1);
elseif (strcmp(gatename, 'SW') == 1)
    fixAngle = -((abs(dRed(1))+pi()-abs(dBlue(1)))/2);
    %fixAngle = (dRed(1)+(abs(dRed(1))+abs(dBlue(1)))/2);
end
%fixAngle = 0;
posRedB = (rot(fixAngle)*posRedB')';
posGreenB = (rot(fixAngle)*posGreenB')';
posBlueB = (rot(fixAngle)*posBlueB')';
[dRedBf, rRedBf] = cart2pol(posRedB(L,1), posRedB(L,2));
[dGreenBf, rGreenBf] = cart2pol(posGreenB(L,1), posGreenB(L,2));
posDirRedB = posRedB-posGreenB; 
[dRedB, rRedB] =  cart2pol(posDirRedB(:,1),posDirRedB(:,2));

% Store the full motion positions
posOverallA = [rGreenAf, dGreenAf*180/pi(), (dRedA(L)-dRedA(1))*180/pi()];
posOverallB = [rGreenBf, dGreenBf*180/pi(), (dRedB(L)-dRedB(1))*180/pi()];

%% Average Motions
for I = 1:1:6
    % Forward 
    distGreenA(I,:) = posGreenA(I+1,:)-posGreenA(I,:);
    rotRedA(I) = dRedA(I+1,:)-dRedA(I,:);
    % Backward
    distGreenB(I,:) = posGreenB(I+1,:)-posGreenB(I,:);
    rotRedB(I) = dRedB(I+1,:)-dRedB(I,:);    
end
% Forward 
xAvgDistA = sum(distGreenA(:,1))/(L-1);
xVarDistA = variance(distGreenA(:,1),xAvgDistA);
xDevDistA1 = xAvgDistA+sqrt(xVarDistA);
xDevDistA2 = xAvgDistA-sqrt(xVarDistA);

yAvgDistA = sum(distGreenA(:,2))/(L-1);
yVarDistA = variance(distGreenA(:,2),yAvgDistA);
yDevDistA1 = yAvgDistA+sqrt(yVarDistA);
yDevDistA2 = yAvgDistA-sqrt(yVarDistA);

[dDevA1, rDevA1] = cart2pol(xDevDistA1, yDevDistA1);
[dDevA2, rDevA2] = cart2pol(xDevDistA2, yDevDistA2);
[dAvgA, rAvgA] = cart2pol(xAvgDistA, yAvgDistA);

dDevA = (abs(dDevA1-dAvgA) + abs(dAvgA-dDevA2))/2;
rDevA = (abs(rDevA1-rAvgA) + abs(rAvgA-rDevA2))/2;
rotAvgA = sum(rotRedA)/(L-1);
rotDevA = variance(rotRedA,rotAvgA);

% Backward
xAvgDistB = sum(distGreenB(:,1))/(L-1);
xVarDistB = variance(distGreenB(:,1),xAvgDistB);
xDevDistB1 = xAvgDistB+sqrt(xVarDistB);
xDevDistB2 = xAvgDistB-sqrt(xVarDistB);

yAvgDistB = sum(distGreenB(:,2))/(L-1);
yVarDistB = variance(distGreenB(:,2),yAvgDistB);
yDevDistB1 = yAvgDistB+sqrt(yVarDistB);
yDevDistB2 = yAvgDistB-sqrt(yVarDistB);

[dDevB1, rDevB1] = cart2pol(xDevDistB1, yDevDistB1);
[dDevB2, rDevB2] = cart2pol(xDevDistB2, yDevDistB2);
[dAvgB, rAvgB] = cart2pol(xAvgDistB, yAvgDistB);

dDevB = (abs(dDevB1-dAvgB) + abs(dAvgB-dDevB2))/2;
rDevB = (abs(rDevB1-rAvgB) + abs(rAvgB-rDevB2))/2;
rotAvgB = sum(rotRedB)/(L-1);
rotDevB = variance(rotRedB,rotAvgB);

posCycleA = [rAvgA rDevA dAvgA*180/pi() dDevA*180/pi() rotAvgA*180/pi() rotDevA*180/pi()];
posCycleB = [rAvgB rDevB dAvgB*180/pi() dDevB*180/pi() rotAvgB*180/pi() rotDevB*180/pi()];

%% Plots
figure('position', [500, 250, 750, 400]) 
hold on,
axis equal
plot(posRedA(:,1), posRedA(:,2), 'r');
plot(posGreenA(:,1), posGreenA(:,2), 'Color', [0 0.95 0]);
plot(posBlueA(:,1), posBlueA(:,2), 'b');
hold off,

figure('position', [500, 250, 750, 400]) 
hold on,
axis equal
plot(posRedB(:,1), posRedB(:,2), 'r');
plot(posGreenB(:,1), posGreenB(:,2), 'Color', [0 0.95 0]);
plot(posBlueB(:,1), posBlueB(:,2), 'b');
hold off,