%% Data Collection: Set device info and visualization params %%

colorDevice = imaq.VideoDevice('kinect',1);
depthDevice = imaq.VideoDevice('kinect',2);
step(colorDevice);
step(depthDevice);
colorImage = step(colorDevice);
depthImage = step(depthDevice);
ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);
player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
	'VerticalAxis','y','VerticalAxisDir','down');

xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');

%% Take 30 frames of depth images and save into an array

colorImageArray = cast(zeros(480,640,3,30), 'uint8');
depthImageArray = cast(zeros(480,640,30), 'uint16');

for j = 1:1:30

    colorImageArray(:,:,:,j) = step(colorDevice);  
    depthImageArray(:,:,j) = step(depthDevice);
    

end

pointCloudArray = pointCloud.empty(0,30);

%% Loop through all images to convert to PointClouds and filter out everything but backpack

for k = 1:1:30

    colorImage = colorImageArray(:,:,:,k);
    depthImage = depthImageArray(:,:,k);

    depthImage(depthImage > 1600) = nan;
   
    depthImage(depthImage < 1000) = nan;
   
   ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);
    
    [labels,numClusters] = pcsegdist(ptCloud,0.01); 
    
    temp = labels(200:300,200:300);

    temp = cast(reshape(temp,[1, 101*101]),'double');
    
    [n,bin] = histogram(temp,unique(temp));
    [~,idx] = sort(-n);
    output = bin(idx);
    
    if output(1,1)==0
       
        out = output(1,2);
        
    else
        
        out = output(1,1);
        
    end
    
    idxValidPoints = find(labels == out);
    labelColorIndex2 = labels(idxValidPoints);

    pointCloud1 = select(ptCloud,idxValidPoints);
    PC_cell{k} = pointCloud1;

    
end

%% Method 1: Loop through all clouds. Each one will calculate a transform to the previous frame.
% The transform is then inverted, saved, and applied to the previous one to
% get next one

PC_method1 = load('newBackpackPCA.mat');
PC_cell = PC_method1.PC_cell;

PC_model = load('newscalePC_real.mat');
newscaledPC_real = PC_model.newscaledPC_real;
pc_temp = newscaledPC_real;

for p = 1:30

    input1 = pc_temp;
    input2 = PC_cell{p};
    figure(1);
    pcshowpair(input1,input2);
    
    tform = pcregistericp(input2,input1,'Extrapolate',true,'MaxIterations',1000);
    T_cell{p} = invert(tform);
    figure(2)
    ptCloudtest = pctransform(input2,tform);
    pc_temp = pctransform(pc_temp,invert(tform))
    
    pcshowpair(pc_temp,input2)
    title(strcat('altered\_', num2str(p) , '\_to\_', num2str(p+1)));
    
    pause(0.5);
    
end

%% Method 1: View backpack motion using player tool. Loops till user does CTL-C

player = pcplayer([-.4 .4],[-.5 .5],[1 1.6]);
player2 = pcplayer([-.4 .4],[-.5 .5],[1 1.6]);

while(1)

T_final = eye(4);
    for i = 1:1:30
         T_final = T_final * T_cell{i}.T;
         trigid = rigid3d(T_final);
         ptCloud = pctransform(newscaledPC_real,trigid);
         view(player,ptCloud);     

         ptCloud2 = PC_cell{i};
         view(player2, ptCloud2.Location);
         pause(.1)
    end
end


%% Method 2: Loops through N+1 clouds and combines all transforms to get Tfinal going from N+1 to 1
PC_spin = load('spinning360.mat');
PC_cell = PC_spin.PC_cell;

rot = eye(3);
trans = [0, 0, 0];
tformFinal = rigid3d(rot,trans);
ptCloudFinal = PC_cell{1};

for p = 1:1:30
    
    input1 = PC_cell{p};
    input2 = PC_cell{p+1};
    
    input1_trimmed = pcdownsample(input1,'random',0.5);
    input2_trimmed = pcdownsample(input2,'random',0.5);
    tform = pcregistericp(input2_trimmed,input1_trimmed,'Extrapolate',true,'MaxIterations',100);
    
    tformFinal_t = tform.T * tformFinal.T
    tformFinal = affine3d(tformFinal_t);
    
    T_cell{p} = tformFinal;
    

    ptCloudTransformed = pctransform(input2,tformFinal);
    
    ptCloudFinal = pcmerge(ptCloudFinal, ptCloudTransformed, 0.001);
    
    %Display different views while building
    theta = -pi/2;
    rot = [cos(theta) 0 sin(theta); ...
          0 1 0; ...
          -sin(theta)          0  cos(theta)];
    trans = [0, 0, 0];
    tform_view = rigid3d(rot,trans);
    ptCloudFinal_view = pctransform(ptCloudFinal, tform_view);
    figure(4);
    pcshow(ptCloudFinal_view.Location);
    view(180,90);
    
    theta = pi/2 - pi/8;
    rot = [1 0 0; ...
          0 cos(theta) -sin(theta); ...
          0          sin(theta)  cos(theta)];
    trans = [0, 0, 0];
    tform_view = rigid3d(rot,trans);
    ptCloudFinal_view = pctransform(ptCloudFinal, tform_view);
    figure(5);
    pcshow(ptCloudFinal_view.Location);
    view(180,90);
    
    theta = pi/2;
    rot = [cos(theta) 0 sin(theta); ...
          0 1 0; ...
          -sin(theta)          0  cos(theta)];
    trans = [0, 0, 0];
    tform_view = rigid3d(rot,trans);
    ptCloudFinal_view = pctransform(ptCloudFinal, tform_view);
    figure(6);
    pcshow(ptCloudFinal_view.Location);
    view(180,90);
    
end

figure(7);
pcshow(ptCloudFinal_view);


%% View backpack motion using player tool. Loops till user does CTL-C

player = pcplayer([-.4 .4],[-.5 .5],[1 1.6]);
player2 = pcplayer([-.4 .4],[-.5 .5],[1 1.6]);

while(1)

T_final = eye(4);
    for i = 1:1:30
         ptCloud = pctransform(ptCloudFinal,invert(T_cell{i}));
         view(player,ptCloud);     

         ptCloud2 = PC_cell{i};
         view(player2, ptCloud2.Location);
         pause(.1)
    end
end




%% Release devices 

release(colorDevice);
release(depthDevice);

%% No longer used function to try to eliminate outliers for icp alignment improvement
function [trimmed_B,trimmed_points] = trim_pc(A,B)
    A_pts = A.Location;
    B_pts = B.Location;
    [k, dist] = dsearchn(A_pts,B_pts);
    dist_bool = dist < 0.04;
    trimmed_B = pointCloud(B_pts(dist_bool,:));
    trimmed_points = pointCloud(B_pts(~dist_bool,:));
end







