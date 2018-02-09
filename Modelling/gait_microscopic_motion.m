clc;
clear all;
close all;

M = 10; % total number of modules
R = 2.81;% radius(cm)
Av = 25; % vertical axis(yaw) magnitude
Ah = 5; % horizontal axis(pitch) magnitude
wv = pi()/2; %vertical speed 
wh = pi()/2; %horizontal speed 
dv = 144*pi()/180;
dh = 144*pi()/180;
dvh = 0*pi()/180;%phase difference between module 
ov = 0;% turning angle 
oh = 0;
len = [6.81 6.81 6.81 6.81 6.81 6.81 6.81 6.81 6.81 6.81]; % length of each module(cm)
centreMass = [[-3.405;0;0],[-3.405;0;0],[-3.405;0;0],[-3.405;0;0],[-3.405;0;0], ...
    [-3.405;0;0],[-3.405;0;0],[-3.405;0;0],[-3.405;0;0],[-3.405;0;0]]; % centre mass of each module
mass = [1; 1; 1; 1; 1; 1; 1; 1; 1; 1];
% mass = [0.75 1 1 1 1 1 1 1 1 1.19];
% centreMass = [[-2.403;0;0],[-2.404;0;0],[-2.404;0;0],[-2.404;0;0],[-2.404;0;0], ...
%      [-2.404;0;0],[-2.404;0;0],[-2.404;0;0],[-2.404;0;0],[-3.058;0;0]]; % centre mass of each module

tStep = 1/10;  
t = 0:tStep:4;
tLen = length(t);
dotBM(:,:,:) = zeros(tLen,4,M+1);
dotCM(:,:,:) = zeros(tLen,3,M+1);
dotPM(:,:,:) = zeros(tLen,3,M+1);
for K=1:1:tLen
    %%
    %---Local Initialisation---% (To reset the variables for each time step)
    pos = zeros(3,M+1);
    posCm = zeros(3,M+1); % Position of the centre of each module
    posBottom = zeros(3,M+1); % Position of the bottom centre of each module (use for visualisation)
    posVc = zeros(4,M+1); % Position of each module in virtual chassis frame
    posBM = zeros(4,M+1); % Position of each bottom of a module in virtual chassis frame (use for visualisation)
    i = 1;
    j = 1;
    trans = eye(4); 
    transM = eye(4);
    transCm = eye(4);
    %%
    %--- Angle Calculation ---%
    horAngle = zeros(1,(M/2));
    verAngle = zeros(1,(M/2));
    for I = 1:1:M/2
        horAngle(I) = Ah*sin(wh*t(K)+((I-1)*dh));
        verAngle(I) = Av*sin(wv*t(K)+((I-1)*dv)+dvh)+ov;
    end
    horAngle = round(horAngle,2);
    verAngle = round(verAngle,2);
    verAngle = [0 verAngle(1:(M/2-1))];%shift the angle as the head module is passive
    %%
    %--- Forward simulation ---%
    posBottom(:,1) = [0;0;-R];%Intialise the bottom marker
    for I = 2:1:M+1
        %rotation in vertical axis
        if mod(I,2) == 0
            transCm = trans*trotz(verAngle(i)*pi()/180)...
                *transl(centreMass(1,I-1), centreMass(2,I-1), centreMass(3,I-1));% calculating the centre of mass
            posCm(:,I) = transCm(1:3,4);
            trans = trans*trotz(verAngle(i)*pi()/180)*transl(-len(I-1),0,0);% calculating the module end
            pos(:,I) = trans(1:3,4); 
            transM = trans*transl(0,0,-R);% calculating the bottom marker
            posBottom(:,I) = transM(1:3,4); 
            i = i+1;
        %rotation in horizontal axis    
        else
            transCm = trans*troty(horAngle(j)*pi()/180)...
                *transl(centreMass(1,I-1), centreMass(2,I-1), centreMass(3,I-1));% calculating the centre of mass
            posCm(:,I) = transCm(1:3,4);
            trans = trans*troty(horAngle(j)*pi()/180)*transl(-len(I-1),0,0);
            pos(:,I) = trans(1:3,4);
            transM = trans*transl(0,0,-R);
            posBottom(:,I) = transM(1:3,4);
            j = j+1;
        end
    end
    
    %%
    %--- Centre Mass Calculation ---%
    %Position of centre mass of each module in the normal frame(without virtual chassis) 
    cm(:,:) = posCm(:,2:M+1);
    %Position of centre mass of the snake robot in the normal frame
    CM = calculateCentreMass(cm, mass , M);
    %Centralise the global position to the centre mass of the snake robot
    cm(1,:) = cm(1,:)-CM(1,1);
    cm(2,:) = cm(2,:)-CM(2,1);
    cm(3,:) = cm(3,:)-CM(3,1);
    %%
    %--- SVD calculation ---%
    P = cm';
    [U,S,V] = svd(P);
    %%
    %--- Resolving Sign Ambiguity ---%
    Vc = V; % Current matrix V
    if K == 1 % Alligning the virtual chassis frame to the world frame
        if Av == 0 && Ah > 0 && ov == 0
            s1 = sign(dot(V(:,1),[1 0 0]));
            s2 = sign(dot(V(:,2),[0 0 1]));
            Vc(:,1) = s1*V(:,1);
            Vc(:,2) = s2*V(:,2);
            Vc(:,3) = cross(Vc(:,1),Vc(:,2));
        else
            s1 = sign(dot(V(:,1),[1 0 0]));
            s2 = sign(dot(V(:,2),[0 1 0]));
            Vc(:,1) = s1*V(:,1);
            Vc(:,2) = s2*V(:,2);
            Vc(:,3) = cross(Vc(:,1),Vc(:,2));
        end
    end 
    if K > 1 % Alligning the current virtual chassis frame to the world frame
        s1 = sign(dot(Vc(:,1),Vp(:,1))); % Compare the direction of the previous and the curernt first signular vector V.
        Vc(:,1) = s1 * Vc(:,1);
        s2 = sign(dot(Vc(:,2),Vp(:,2)));
        Vc(:,2) = s2 * Vc(:,2);
        s3 = sign(dot(Vc(:,3),Vp(:,3)));
        Vc(:,3) = s3 * Vc(:,3);
    end
    Vp = Vc; % Previous matrix V 
    %%
    %Forming transformation matrix
    if Av == 0 && Ah > 0 && ov == 0
        T(:,:) = [Vc CM; zeros(1,3) 1]*trotx(-pi()/2);
    else
        T(:,:) = [Vc CM; zeros(1,3) 1];
    end
    %
    %Aligning the the snake robot position in virtual chassis
    for I=1:1:M+1
        posBM(:,I) = T\[posBottom(:,I);1];
        posVc(:,I) = T\[posCm(:,I);1];%Snake robot in virtual chassis coordinates
    end 
    dotBM(K,:,:)= posBM;
    dotVc(K,:,:)= posVc;
end
dotPM(2:tLen,:,:) = (dotBM(1:tLen-1,1:3,:));
dotPM(1,:,:) = dotBM(1,1:3,:);
for K=2:1:tLen
   for I = 1:1:M+1
       dotX = dotBM(K,1:3,I) - dotPM(K,:,I);
       dotCM(K,:,I) = dotX/rssq(dotX);
   end
end
dotPc(2:tLen,:,:) = (dotVc(1:tLen-1,1:3,:));
dotPc(1,:,:) = dotVc(1,1:3,:);
for K=2:1:tLen
   for I = 1:1:M+1
       dotVX = dotVc(K,1:3,I) - dotPc(K,:,I);
       dotVM(K,:,I) = dotVX/rssq(dotVX);
   end
end

figure
axis equal
hold on
for I = 1:1:M+1
    quiver3(dotBM(1:2:tLen,1,I),dotBM(1:2:tLen,2,I),dotBM(1:2:tLen,3,I),dotCM(1:2:tLen,1,I),dotCM(1:2:tLen,2,I),dotCM(1:2:tLen,3,I),1)
end
hold off

figure
axis equal
hold on
for I = 1:1:M+1
    quiver3(dotVc(1:2:tLen,1,I),dotVc(1:2:tLen,2,I),dotVc(1:2:tLen,3,I),dotVM(1:2:tLen,1,I),dotVM(1:2:tLen,2,I),dotVM(1:2:tLen,3,I),1)
end
hold off
i = 0;
for J = 1:1:tLen-1
    dX = 0;
    dY = 0;
    for I = 1:2:M+1
        dx(J,I) = abs(dotBM(J+1,1,I)-dotBM(J,1,I))+dX;
        dy(J,I) = abs(dotBM(J+1,2,I)-dotBM(J,2,I))+dY;
        dX = dx(J,I);
        dY = dy(J,I);
        i = i+1;
    end
    deltaX = sum(dx(J,:));
    deltaY = sum(dy(J,:));
end
if ov == 0
   Tt = [cos(0),-sin(0),0, deltaX;... %look at K = 2 and K = 9, they are the same.
            sin(0),cos(0),0,0;... %so it doesn't not look like the error comes from here.
            0,0,1,0;...
            0,0,0,1];
    
end
[d, r] = cart2pol(-sign(wv)*deltaX,-sign(wv)*deltaY);
posFinal = [r, d*180/pi()];