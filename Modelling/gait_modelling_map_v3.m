% Kinematic Motion Model for linear progression and turning gaits
% Author: Cid Gilani
clc;
clear all;
close all;
M = 10; % total number of modules
Av = 0; % vertical axis(yaw) magnitude
Ah = 30; % horizontal axis(pitch) magnitude
wv = 0;%pi();%pi();%pi(); %vertical speed 
wh = pi(); %horizontal speed 
dv = 0*pi()/180;%10*pi()/180;%(2*pi()*k1)/(M/2);
dh = 144*pi()/180;%(2*pi()*k2)/(M/2);
dvh = 0*pi()/180;%phase difference between module 
ov = 10;% turning angle 
oh = 0;
rActual = 3.392364203;
dActual = 89.10412736;
rotActual = -2.528681934;
%Mechanical Properties
radius = 2.8;
len = [6.81 6.81 6.81 6.81 6.81 6.81 6.81 6.81 6.81 6.81]; % length of each module(cm)
mass = [1 1 1 1 1 1 1 1 1 1];
%mass = [0.75; 1; 1; 1; 1; 1; 1; 1; 1; 1.19];
%cModule = [[-2.403;0;0],[-2.404;0;0],[-2.404;0;0],[-2.404;0;0],[-2.404;0;0], ...
 %   [-2.404;0;0],[-2.404;0;0],[-2.404;0;0],[-2.404;0;0],[-3.058;0;0]]; % centre mass of each module
hModule = [[-3.405;0;0],[-3.405;0;0],[-3.405;0;0.2],[-3.405;0;0],[-3.405;0;0], ...
    [-3.405;0;0],[-3.405;0;0],[-3.405;0;0],[-3.405;0;0],[-3.405;0;0]];
cModule = hModule;
%hModule = cModule;

%Time setting
tStep = 1/10; % for some reason decreasing the time step makes the accumulative error increases faster.
tMax = 12;% max time
t = 0:tStep:tMax;%time range
tLen = length(t);
tau = 5;%side-winding 3.075;
delta = -4.5;%side-winding 1;%threshold setting
%%
%--- Global Initialisation ---% (Just to speed things up)
transStep = zeros(4,4,M+1,tLen);
VStep = zeros(3,3,tLen);
posStep = zeros(4,M+1,tLen);
posFrontX = zeros(1,tLen);
posFrontY = zeros(1,tLen);
posMidX = zeros(1,tLen);
posMidY = zeros(1,tLen);
posRearX = zeros(1,tLen);
posRearY = zeros(1,tLen);

for K=1:1:tLen
    %%
    %---Local Initialisation---% (To reset the variables for each time step)
    pos = zeros(3,M+1);
    posCm = zeros(3,M+1); % Position of the centre of each module
    posCn = zeros(3,M+1); % The position of the centroid of each module
    posBottom = zeros(3,M+1); % Position of the bottom centre of each module (use for visualisation)
    posVc = zeros(4,M+1); % Position of each module in virtual chassis frame
    posBM = zeros(4,M+1); % Position of each bottom of a module in virtual chassis frame (use for visualisation)
    %error = zeros(1,M+1);
    alpha = zeros(1,M);
    wi = zeros(1,M);   
    Ri = zeros(3,3,M);
    ri = zeros (3,M);
    omega = zeros(3,3,M);
    deltaB = zeros (3,M);
    di = zeros(3,M);
    normDi = zeros(3,M);
    normWi = zeros(1,M);
    zMin1 = 0;
    zMin2 = 0;
    i = 1;
    j = 1;
    trans = eye(4); 
    Tt = zeros(4); 
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
    angle(K,:) = horAngle; 
    posBottom(:,1) = [0;0;-radius];%Intialise the bottom marker
    %%
    %--- Forward simulation ---%
    for I = 2:1:M+1
        %rotation in vertical axis
        if mod(I,2) == 0
            transCm = trans*trotz(verAngle(i)*pi()/180)...
                *transl(cModule(1,I-1), cModule(2,I-1), cModule(3,I-1));% calculating the centre of mass
            transCn = trans*trotz(verAngle(i)*pi()/180)...
                *transl(hModule(1,I-1), hModule(2,I-1), hModule(3,I-1));% calculating the centroid
            posCm(:,I) = transCm(1:3,4);
            posCn(:,I) = transCn(1:3,4);
            trans = trans*trotz(verAngle(i)*pi()/180)*transl(-len(I-1),0,0);% calculating the module end
            pos(:,I) = trans(1:3,4); 
            transStep(:,:,I,K) = trans;
            transM = trans*transl(0,0,-radius);% calculating the bottom marker
            posBottom(:,I) = transM(1:3,4); 
            i = i+1;
        %rotation in horizontal axis    
        else
            transCm = trans*troty(horAngle(j)*pi()/180)...
                *transl(cModule(1,I-1), cModule(2,I-1), cModule(3,I-1));% calculating the centre of mass
            transCn = trans*troty(horAngle(j)*pi()/180)...
                *transl(hModule(1,I-1), hModule(2,I-1), hModule(3,I-1));% calculating the centre of mass
            posCm(:,I) = transCm(1:3,4);
            posCn(:,I) = transCn(1:3,4);
            trans = trans*troty(horAngle(j)*pi()/180)*transl(-len(I-1),0,0);
            pos(:,I) = trans(1:3,4);
            transStep(:,:,I,K) = trans;
            transM = trans*transl(0,0,-radius);
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
    VStep(:,:,K) = Vp; % Store the V matrix into array for each time step
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
        posVc(:,I) = T\[pos(:,I);1];%Snake robot in virtual chassis coordinates
        %posStep(:,I,K) = T\[posCn(:,I);1];
        posBM(:,I) = T\[posBottom(:,I);1];
        posStep(:,I,K) =  posBM(:,I);
    end 
    [X,Y,Z] = posCoordinate(M, radius, posVc, posBM); 
    %%
    %--- Modelling ---% 
    if K == 1 % At time zero, the snake position is set at the origin of the world frame
        Tp = eye(4); % initial transform matrix that describe the snake robot in the world frame
        posT = posVc;
        posBT = posBM;
        %Recording the positions of front, middle and rear modules.   
        posFrontX(K) = posT(1,2);
        posFrontY(K) = posT(2,2);
        posMidX(K) = posT(1,6);
        posMidY(K) = posT(2,6);
        posRearX(K) = posT(1,M);
        posRearY(K) = posT(2,M);
    %%
    %This part onwards is based on the "Simplified Modelling Motion
    %Modelling for Snake Robots" paper.
    else % After zero, start calculating the position
        %Translational displacement calculation
        posCurrent = posStep(1:3,2:M+1,K);
        posPrevious = posStep(1:3,2:M+1,K-1);
        deltaA = posCurrent-posPrevious;
        deltaA(1,:) = abs(deltaA(1,:));
        deltaA(2,:) = abs(deltaA(2,:));
        %deltaAA(K,:) = deltaA(1,:);
        %Rotational displacement calculation
        rotCurrent = transStep(1:3,1:3,2:M+1,K);%Current rotation matrix which decribes the relative angle between modules
        rotPrevious = transStep(1:3,1:3,2:M+1,K-1);%Previous rotation matrix 
%         for L = 1:1:M
%             Ri(:,:,L) = VStep(:,:,K)\rotCurrent(:,:,L);%include virtual chassis as suggested in the paper
%             ri(:,L) = Ri(:,:,L)\[0;0;-radius];
%             omega(:,:,L) = ((VStep(:,:,K-1))\rotPrevious(:,:,L))\Ri(:,:,L);
%             deltaB(:,L) = Ri(:,:,L)*((omega(:,:,L)*ri(:,L))-(omega(:,:,L)\ri(:,L)))./2;
%         end
        %Total displacement
        deltaP = deltaA+deltaB;
        %%
        %--- Searching for the lowest ground contact ---%
        % The minimum Z in the virtual chasis have two coloumn, so I checked both and take the lowest.
        zMin1 = min(Z(2,:));
        zMin2 = min(Z(1,:));
        if zMin1 < zMin2 
            zMin = zMin1;
        else 
            zMin = zMin2;
        end
        %%
        %--- Ground contact threshold ---%
        for L = 1:1:M
            if (posCurrent(3,L)-zMin) < tau
                alpha(L) = 1-((posCurrent(3,L)-zMin)/tau);
            else
                alpha(L) = 0;
            end
            wi(L) = (1-exp(-delta*alpha(L)))/(1-exp(-delta));
        end
        normWi = wi./sum(wi);
        %%
        %--- Body frame translation ---%
        deltaM = 0; %reset the variable
        sumAngle = 0;
        lenMod = 0;
        for L = 1:1:M
            deltaM = (normWi(L).*deltaP(:,L))+deltaM;
            lenMod = sqrt((posVc(1,L)-posVc(1,L+1))^2+(posVc(2,L)-posVc(2,L+1))^2)+lenMod;
        end
        if ov ~=0
            lenMod = lenMod/(M/2);
            n = 360/abs(ov);
            rc = lenMod/(2*sin(pi()/n));
            deltaS = sqrt(deltaM(1)^2+deltaM(2)^2);
            angleRot = deltaS/rc;
            deltaM(2) = sign(wh)*(rc-rc*cos(angleRot));
            deltaM(1) = sign(wh)*(rc*sin(angleRot));
            sumAngle = -sign(wh)*atan(deltaM(2)/deltaM(1));
        end
        %--- Body frame rotation ---%
%         sumAngle = 0;
%         for L = 1:1:M
%             di(:,L) = cross([0;0;1],posCurrent(:,L));
%             normDi(:,L) = di(:,L)/rssq(di(:,L));
%             sumAngle = (normWi(L)*dot(deltaP(:,L),normDi(:,L))/rssq(posCurrent(:,L)))+sumAngle;
%         end
%         sumAngle = -sumAngle;
        %--- Forming the full body frame motion matrix
        K; % I'm printing this to see that every gate cycle the transformation matrix should be the same 
        Tt = [cos(sumAngle),-sin(sumAngle),0,deltaM(1);... %look at K = 2 and K = 9, they are the same.
            sin(sumAngle),cos(sumAngle),0,deltaM(2);... %so it doesn't not look like the error comes from here.
            0,0,1,0;...
            0,0,0,1];
        %Applying the transformation 
        Tp = Tp*Tt; % Sum up the transformation displacements and rotations.
        posT = Tp*posVc; % Applying the accumulated transformation to the snake robot in the  virtual chassis frame.
        posBT = Tp*posBM; % Didn't use this in the plot
        
        posFrontX(K) = posT(1,2);
        posFrontY(K) = posT(2,2);
        posMidX(K) = posT(1,6);
        posMidY(K) = posT(2,6);
        posRearX(K) = posT(1,M);
        posRearY(K) = posT(2,M);       
    end
end
%%
%Plot the trace of the snake robot
lenData =length(posRearX);
posFrontY = posFrontY - posMidY(1);
posRearY = posRearY - posMidY(1);
posMidY = posMidY - posMidY(1);

posFrontX = posFrontX - posMidX(1);
posRearX = posRearX - posMidX(1);
posMidX = posMidX - posMidX(1);

[dRed, rRed] = cart2pol(posFrontX(1), posFrontY(1));
[dBlue, rBlue] = cart2pol(posRearX(1), posRearY(1));
[dGreen, rGreen] = cart2pol(posMidY(1), posMidY(1));
fixAngle = (abs(dRed)-(abs(dRed)+pi()-abs(dBlue))/2);
%fixAngle =0;
posMid = [posMidX',posMidY',ones(lenData,1)];
posRear = [posRearX',posRearY',ones(lenData,1)];
posFront = [posFrontX',posFrontY',ones(lenData,1)];
posMid = (rot(fixAngle)*posMid')';
posRear = (rot(fixAngle)*posRear')';
posFront = (rot(fixAngle)*posFront')';
figure
hold on
axis equal
plot(posFront(:,2),posFront(:,1), 'r')
plot(posMid(:,2),posMid(:,1), 'g')
plot(posRear(:,2),posRear(:,1), 'b')
xlabel('X (cm)')
ylabel('Y (cm)')
hold off
legend('Front', 'Middle', 'Rear')
[dMid, rMid] = cart2pol(posMid(lenData,2), posMid(lenData,1));
[dFront1, rFront1] = cart2pol(posFront(1,2),posFront(1,1));
[dFront2, rFront2] = cart2pol(posFront(lenData,2)-posMid(lenData,2), posFront(lenData,1)-posMid(lenData,1));
rotFront = dFront2-dFront1;
posCycle = [rMid dMid*180/pi() dFront2*180/pi()];
rSum = (rMid-rActual)/rActual
dSum = (dMid*180/pi()-dActual)/dActual
rotSum = abs(rotFront*180/pi()-rotActual)/abs(rotActual)
error = sqrt(1*rSum^2+1*dSum^2+1*rotSum^2);
