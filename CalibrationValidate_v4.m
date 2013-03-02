function [rms] = CalibrationValidate_v4()
%%Get binary images with the surface of the sphere
clc
%%%%Get File Names
[fileImages, pathImages, filterindex] = uigetfile('*.bmp','Load Images','MultiSelect', 'on');
[fileTrackerRot, pathTrackerRot] = uigetfile('*.txt','Load Tracker Rotations');
[fileTrackerTrans, pathTrackerTrans] = uigetfile('*.txt','Load Tracker Translations');
[fileEstimatedParameters, pathEstimatedParameters] = uigetfile('*.txt','Load Estimated Parameters');
[fileCenterTrans, pathCenterTrans] = uigetfile('*.txt','Tracked Center');

%%%%Open Files
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

%%%%Vaiable Init
[wtv nImages] = size(fileImages);
nTotal = 0;

%%%%Pick Pixel and Calculate Radius Error
for i=1:nImages
    %%%%Calc Transformation Matrix for Image i
    rotTracker = fscanf(rf,'%f',4);
    transTracker = fscanf(tf,'%f',3);
    
    rotTrackerMatrix = quat2dcm(rotTracker')';

    tTr = zeros(4,4);
    tTr(1:3, 1:3) = rotTrackerMatrix(1:3, 1:3);
    tTr(1:3, 4) = transTracker(1:3);
    tTr(4,4) = 1;

    %%%%Read Image
    img_name = strcat(pathImages,fileImages{i});
    im = imread(img_name);
    
    [y x] = find(im);
    [nPoints wtv] = size(y);

    for j=1:nPoints
        
        nTotal = nTotal + 1;
        points(nTotal, :) = [x(j,1) y(j,1)];
        %%%%Transform Point
        point = [x(j,1)*scaleCalibration(1) y(j,1)*scaleCalibration(2) 0 1]';
        transformedPoint(:,nTotal) = tTr*rTp*point;        
    end

end


global X;
X = transformedPoint(1:3,:);

global n;
n = nTotal 

%Y = XA
% Y = zeros(n,1);
% C = zeros(n,4);
% 
% for i=1:n
%     Y(i,1) = X(:,i)'*X(:,i);
%     C(i,1) = X(1,i);
%     C(i,2) = X(2,i);
%     C(i,3) = X(3,i);
%     C(i,4) = 1;
% end
% 
% res = lsqnonneg(C,Y)

promCx = mean(X(1,:));
promCy = mean(X(2,:));
promCz = mean(X(3,:));

errorPX = sqrt((centerX-promCx)^2)
errorPY = sqrt((centerY-promCy)^2)
errorPZ = sqrt((centerZ-promCz)^2)

errorP = sqrt((centerX-promCx)^2 + (centerY-promCy)^2 + (centerZ-promCz)^2)



for i=1:n
    promR(i) = sqrt((X(1)-promCx)^2 + (X(2)-promCy)^2 + (X(3)-promCz)^2);
end

promRad = mean(promR);

sphere = [promCx promCy promCz promRad]'


center= lsqnonlin(@centerEstimator,sphere)

errorX = sqrt((centerX-center(1))^2)
errorY = sqrt((centerY-center(2))^2)
errorZ = sqrt((centerZ-center(3))^2)

error = sqrt((centerX-center(1))^2 + (centerY-center(2))^2 + (centerZ-center(3))^2)

save('Points', 'transformedPoint', 'points');