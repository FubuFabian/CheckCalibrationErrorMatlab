function [error] = CalibrationValidate_v5()
%%Get the points of estimated centers estimating circles 
clc
%%%%Get File Names
[filePoints, pathPoints]= uigetfile('*.txt','Load Points');
[fileTrackerRot, pathTrackerRot] = uigetfile('*.txt','Load Tracker Rotations');
[fileTrackerTrans, pathTrackerTrans] = uigetfile('*.txt','Load Tracker Translations');
[fileEstimatedParameters, pathEstimatedParameters] = uigetfile('*.txt','Load Estimated Parameters');
[fileCenterTrans, pathCenterTrans] = uigetfile('*.txt','Tracked Center');

%%%%Open Files
pf = fopen(strcat(pathPoints,filePoints));
rf = fopen(strcat(pathTrackerRot,fileTrackerRot));
tf = fopen(strcat(pathTrackerTrans,fileTrackerTrans));
cf = fopen(strcat(pathEstimatedParameters,fileEstimatedParameters));
nf = fopen(strcat(pathCenterTrans,fileCenterTrans));

%%%%Calc Transformation Matrix from Estimated Parameters
transCalibration = fscanf(cf,'%f',3);
rotCalibration = fscanf(cf,'%f',3);
scaleCalibration = fscanf(cf,'%f',2);

rotCalibrationMatrix = angle2dcm(rotCalibration(1),rotCalibration(2),rotCalibration(3))';

rTp = zeros(4,4);
rTp(1:3, 1:3) = rotCalibrationMatrix(1:3, 1:3);
rTp(1:3, 4) = transCalibration(1:3);
rTp(4,4) = 1;

%%%%Calc Mean Center from Tracked Sphere

centerX = 0;
centerY = 0;
centerZ = 0;
nLines = 0;

while (fgets(nf) ~= -1),
  center = fscanf(nf, '%f',3);
  centerX = centerX + center(1,1); 
  centerY = centerY + center(2,1); 
  centerZ = centerZ + center(3,1); 
  nLines = nLines + 1;
end

centerX = centerX/nLines;
centerY = centerY/nLines;
centerZ = centerZ/nLines;

disp('Mean Center in X')
disp(centerX)

disp('Mean Center in Y')
disp(centerY)

disp('Mean Center in Z')
disp(centerZ)

nPoints = 0;

while (fgets(pf) ~= -1)
    
    nPoints =  nPoints + 1;
    point = fscanf(pf,'%f',2);
    
    rotTracker = fscanf(rf,'%f',4);
    transTracker = fscanf(tf,'%f',3);
    
    rotTrackerMatrix = quat2dcm(rotTracker')';

    tTr = zeros(4,4);
    tTr(1:3, 1:3) = rotTrackerMatrix(1:3, 1:3);
    tTr(1:3, 4) = transTracker(1:3);
    tTr(4,4) = 1;
    
    scaledPoint = [point(1,1)*scaleCalibration(1) point(2,1)*scaleCalibration(2) 0 1]';
    transformedPoints(:,nPoints) = tTr*rTp*scaledPoint;  
     
end

display('Transformed Points')
display(transformedPoints);

eCenterX = mean(transformedPoints(1,:));
eCenterY = mean(transformedPoints(2,:));
eCenterZ = mean(transformedPoints(3,:));

errorX = sqrt((centerX-eCenterX)^2)
errorY = sqrt((centerY-eCenterY)^2)
errorZ = sqrt((centerZ-eCenterZ)^2)


error = sqrt((centerX-eCenterX)^2 + (centerY-eCenterY)^2 + (centerZ-eCenterZ)^2)

