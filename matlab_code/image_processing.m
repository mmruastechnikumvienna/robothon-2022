%% begin programm
%clear all;
%close all;

%% Establish TCP/IP connection
%TCP Objekt
robot_tcp = tcpclient('192.168.0.1',55000);
robot_tcp.InputBufferSize = 1;

% Start communication
fopen(robot_tcp);
fprintf('Roboter kommunikation initialisieren\n');
%% Initialise gripper
%grip = RobotiqGripper;
%grip.init('COM6');
%grip.Position=255;
%fprintf('Gripper initialisieren\n');
%% Make Pipeline object to manage streaming
pipe = realsense.pipeline();
fprintf('Kamera starten\n');

% Make Colorizer object to prettify depth output
colorizer = realsense.colorizer();
% define point cloud object
pcl_obj = realsense.pointcloud();

% Start streaming with default settings
profile = pipe.start();
align_to = realsense.stream.color;
alignedFs = realsense.align(align_to);

 
% Get streaming device's name
dev = profile.get_device();
name = dev.get_info(realsense.camera_info.name);

%% Get frames. We discard the first couple to allow
% the camera time to settle
for i = 1:10
fs = pipe.wait_for_frames();
end


%% create a point cloud player
player1 = pcplayer([-1 1],[-1 1],[-1 1]);

frameCount = 0;



%% loop through
while isOpen(player1) && frameCount < 10
frameCount = frameCount+1;
fs = pipe.wait_for_frames();

%align the depth frames to the color stream
aligned_frames = alignedFs.process(fs);
depth = aligned_frames.get_depth_frame();
color = fs.get_color_frame();

%get the points cloud based on the aligned depth stream
pnts = pcl_obj.calculate(depth);
pcl_obj.map_to(color);
colordata = color.get_data();
colordatavector = [colordata(1:3:end)',colordata(2:3:end)',colordata(3:3:end)'];

vertices = pnts.get_vertices();

%view(player1,vertices,colordatavector)

view()
end

%% show and save color image
        color = fs.get_color_frame();
        color_data = color.get_data();
        color_img = permute(reshape(color_data',[3,color.get_width(),color.get_height()]),[3 2 1]);
        %imshow(color_img);
        title(sprintf("Color image frame"));
        imwrite(color_img, "Bild.jpg");

fprintf('Bild speichern \n');

%% show and save pointcloud image        
        %pcwrite(pcl_obj, "PointCloud.pcd");
        %pc = pcread("PointCloud.pcd");
        %pcshow(pc);

%% image processing find box
fprintf('finding Stuff\n');
RGB = imread("Bild.jpg");
GRAY = rgb2gray(RGB);
threshold = graythresh(GRAY);
BW = im2bw(GRAY,threshold);
BW = ~ BW;
[B,L] = bwboundaries(BW, 'noholes');

edges = edge(L,"canny"); %edge detection

%figure(1), subplot(2,2,1),imshow(RGB),title('Original');%Bild anzeigen
%figure(1), subplot(2,2,3),imshow(L),title('L Bild');%Bild anzeigen
%figure(1), subplot(2,2,2),imshow(edges),title('Kanten');%Bild anzeigen

[H,T,R] = hough(edges,'RhoResolution',5,'Theta',-90:.5:89);

P = houghpeaks(H,4);%detects maxima 
x = T(P(:,2));
y = R(P(:,1));

lines = houghlines(edges,T,R,P,'FillGap',30,'MinLength',275); 
lines(1,1).theta
max_len = 0;
%figure(1),subplot(2,2,4), imshow(RGB), hold on
%impixelinfo()
for k = 1:length(lines)
    xy = [lines(k).point1; lines(k).point2];
%    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
    
% Plot beginnings and ends of lines
%    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red'); 
end



%% ------------------------------------
%https://de.mathworks.com/matlabcentral/answers/506239-how-to-rotate-a-rectangle
%rotate rectanlge

%% g find oragen area
Irgb  = imread("Bild.jpg");
Ihsv  = rgb2hsv(Irgb);
low = 0.04;      
high = 0.07;
h = (Ihsv(:,:,1) >= low) & (Ihsv(:,:,1) <= high);
s = (Ihsv(:,:,2) >= 0.6) & (Ihsv(:,:,2) <= 0.8);
v = (Ihsv(:,:,3) >= 0.6) & (Ihsv(:,:,3) <= 0.8);
mask = uint8(h & s & v);
mask  = imclose(mask, strel('disk', 5));
mR = mask .* Irgb(:,:,1);
mG = mask .* Irgb(:,:,2);
mB = mask .* Irgb(:,:,3);
Icolor = cat(3, mR, mG, mB);
Igray = rgb2gray(Icolor);
Ibw = im2bw(Igray,graythresh(Igray));
Ilabel = bwlabel(Ibw);
stat = regionprops(Ilabel,'boundingbox','centroid');
%
%figure(3)
%imshow(Irgb); hold on;
%for x = 1 : length(stat)
%    bb = stat(x).BoundingBox;
%    xc = stat(x).Centroid(1);
%    yc = stat(x).Centroid(2);
%    txt = sprintf('x : %.2f\ny : %.2f',xc,yc);
%    rectangle('position',bb,'edgecolor','b','linewidth',2);    
%    plot(xc,yc,'bo');
%    text(xc,yc+30,txt,'color','y','fontweight','normal');
%end

%% finding the point near the orange start button
coordinates = [lines(1).point1; lines(1).point2;lines(2).point1; lines(2).point2;];
%min_dis = 2000;
%startpoint = [0 0];
%for x = 1 : 4
%tmp_dis = sqrt((coordinates(x,1) - xc)^2 + (coordinates(x,2) - yc)^2); 
%
 %   if tmp_dis < min_dis 
  %      min_dis = tmp_dis
   %     startpoint(1,1) = coordinates(x,1)
    %    startpoint(1,2) = coordinates(x,2)
    %end
%
%end

%% calculating mid of box
mid_x = (coordinates(1,1)+coordinates(2,1)+coordinates(3,1)+coordinates(4,1))/4;
mid_y = (coordinates(1,2)+coordinates(2,2)+coordinates(3,2)+coordinates(4,2))/4;
%% printing pisitions in picture
figure(5)
imshow(Irgb); hold on;
impixelinfo()
%plot(startpoint(1,1),startpoint(1,2),'x','LineWidth',5,'Color','red'); 
plot(mid_x,mid_y,'x','LineWidth',5,'Color','blue'); 
%% calculating 
height = 369;
width = 153;
%244 107
%166 333
%78  226
pixel_lange = sqrt(62^2 + 255^2);
pixel_faktor = width /pixel_lange;

abweichung_x = (mid_x - 320) * pixel_faktor;
abweichung_y = (mid_y - 240) * pixel_faktor;

pos_x = 723.96 -80 + abweichung_y;
pos_y = -32.5 + abweichung_x;
pos_z = 179.57-height;
fprintf('Posistionen berechnet\n');
if lines(1,1).theta > 0
anglee = 90 - lines(1,1).theta;
elseif lines(1,1).theta < 0
anglee = - 90 - lines(1,1).theta;
elseif lines(1,1).theta == 0
anglee = 0;
end

%% Prepare data for robot studio 
pause(0.5);
nmbr(1,1)= pos_x;
nmbr(1,2)= pos_y;
rad = deg2rad(anglee);
q = angle2quat(rad, 0, 0);
q_n = quatnormalize(q);
nmbr(1,3) = q(1,1);
nmbr(1,4) = q(1,2);
nmbr(1,5) = q(1,3);
nmbr(1,6) = q(1,4);



% Stop streaming
pipe.stop();

%% Communication
% Send camera data to robot

pause(0.5);
write(robot_tcp,nmbr,'single');
fprintf('Daten gesendet\n');


% Read data when available
while(robot_tcp.Status)
    if(get(robot_tcp, 'BytesAvailable') > 0)
        robot_tcp.BytesAvailable ;
        data = fread(robot_tcp);

        grip.Position = data;
 
        fprintf('%d \n',data);
        flush(robot_tcp, "input");      
    end
end

% Stop communication
fclose(robot_tcp); 
delete(robot_tcp); 
clear robot_tcp;