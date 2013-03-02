function [rms] = CalibrationValidate_v2()
%%Get images and select a line of each image, it measures radius
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
[wtv n] = size(fileImages);
nTotal = 0;
radius = 5.75;

%%%%Pick Pixel and Calculate Radius Error
for i=1:n
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
    imshow(im);
    
    points = imfreehand(gca,'Closed',0);
    pointPos = getPosition(points);
    [nPoints wtv] = size(pointPos);
    
    for j=1:nPoints
        nTotal = nTotal + 1;
        point = pointPos(j,:);
        %%%%Transform Point
        point = [point(1)*scaleCalibration(1) point(2)*scaleCalibration(2) 0 1]';
        transformedPoint(:,nTotal) = tTr*rTp*point;
    
        %%%%Calc Radius from TransformedPoint and calc error
        estimatedRadius(1,nTotal) = sqrt((transformedPoint(1,nTotal)-centerX)^2 + (transformedPoint(2,nTotal)-centerY)^2 + (transformedPoint(3,nTotal)-centerZ)^2);
        error(1,nTotal) = abs(estimatedRadius(1,nTotal) - radius);
        
    end

end


%%%%Calc RMS error
sum = 0;
for k=1:nTotal
    temp = error(1,k)^2;    
    sum = sum + temp;
end

rms = sqrt(sum/nTotal);

disp('RMS')
disp(rms)

%%%%Graph error
xaxis=1:nTotal;
plot(xaxis,error(1,:),'b:+');
xlabel('Point');
ylabel('Error');
