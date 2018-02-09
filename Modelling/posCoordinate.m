function [X,Y,Z] = posCoordinate(numModule, radius, posModule,posMarker)
%Plot snake robot position
    for J = 1:1:numModule
        [X,Y,Z] = cylinder2P(radius,25,posModule(1:3,J)',posModule(1:3,J+1)');
        [X1,Y1,Z1] = sphere(10);
        X1 = (radius.*X1)+posModule(1,J);
        Y1 = (radius.*Y1)+posModule(2,J); 
        Z1 = (radius.*Z1)+posModule(3,J);
    end
    [X1,Y1,Z1] = sphere(10);
    X1 = (radius.*X1)+posModule(1,numModule+1);
    Y1 = (radius.*Y1)+posModule(2,numModule+1); 
    Z1 = (radius.*Z1)+posModule(3,numModule+1);
end

