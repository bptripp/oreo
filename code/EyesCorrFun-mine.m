function [eyeLcorr, eyeRcorr] = EyesCorrFun(LeyeImg,ReyeImg,varargin)

%%% Input/Load two images from Camera1 and Camera2, taked with cameras spaced
%%% ~55mm apart and ~ 40cm back from central target object.

%%% Output: Nx,Ny pixels each camera must shift to align with other. 
%%% Dividing each by 2 gives the amount to shift both cams to meet in middle.
%%% Added check that each camera shifting opposite direction. 
%%% Should not shift both in same direction!?
%%%
%%% Remaining issue: To shift the center of the cameras image requires
%%% changing the camera pitch and yaw angles, but the required amount
%%% depends on the distance of the target from the cameras!
%%% Maybe assume distance to make correction, and see if it improves

%im10 = rgb2gray(imread('IMG_1245.jpg'));
%im20 = rgb2gray(imread('IMG_1246.jpg'));

im10 = rgb2gray(LeyeImg);
im20 = rgb2gray(ReyeImg);

rescale_factor = 0.5;

Tsize = 250;                % Set width/hight of square "Fovea" Targets
im1 = imresize(im10,rescale_factor);   % Resize the images
im2 = imresize(im20,rescale_factor);
[nR,nC] = size(im1);        % Get the image size
y1 = nR/2-Tsize/2;          % Get the upper left and lower right corners of the Targets
y2 = nR/2+Tsize/2;
x1 = nC/2-Tsize/2;
x2 = nC/2+Tsize/2;
im1c = im1(y1:y2,x1:x2);    % Create Target images from the original images.
im2c = im2(y1:y2,x1:x2);


%%% Calculate the cross-correlations
[c12] = normxcorr2(im1c,im2);   % Correlation of Target1 with Camera2
[c21] = normxcorr2(im2c,im1);   % Correlation of Target2 with Camera1
xvect = [1:size(c12,1)];        % Get the size of the correlograms
yvect = [1:size(c12,2)];
[c1] = normxcorr2(im1c,im1);    % Correlation of Target1 with Camera1 (should have max=1)
[c2] = normxcorr2(im2c,im2);    % Correlation of Target2 with Camera2 (should have max=1)

%% apply prior expectation
s = size(c12);
y_dist_from_centre = repmat((1:s(1))'-s(1)/2-.5, 1, s(2));
x_dist_from_centre = repmat((1:s(2))-s(2)/2-.5, s(1), 1);
squared_dist_from_centre = x_dist_from_centre.^2 + y_dist_from_centre.^2;
sigma = 100;
prior = exp(-squared_dist_from_centre / 2 / sigma^2);
c12 = c12 .* prior;
c21 = c21 .* prior;

[max_c12, imax12] = max((c12(:)));  % Find the peak locations in correlograms
[max_c21, imax21] = max((c21(:)));
[max_c1, imax1] = max(abs(c1(:)));
[max_c2, imax2] = max(abs(c2(:)));

[ypeak12, xpeak12] = ind2sub(size(c12),imax12(1));  % Convert peak locations into x,y coords
[ypeak21, xpeak21] = ind2sub(size(c21),imax21(1));
[ypeak1, xpeak1] = ind2sub(size(c1),imax1(1));
[ypeak2, xpeak2] = ind2sub(size(c2),imax2(1));

Cam1corr0 = [ypeak1-ypeak12,xpeak1-xpeak12]; % How far 1 camera should move (in pixels) to match other camera.
Cam2corr0 = [ypeak2-ypeak21,xpeak2-xpeak21]; % If you want to move 1 eye/camera to match the other, use these. 

OppSignCheck(1) = sign(Cam1corr0(1))~=sign(Cam2corr0(1));
OppSignCheck(2) = sign(Cam1corr0(2))~=sign(Cam2corr0(2));

Cam1corrAvg(1) = Cam1corr0(1)/2;    % If you want to move both cameras to meet in the "middle", move each by half their offset amount. 
Cam2corrAvg(1) = Cam2corr0(1)/2;    % This makes most sense when the offset direction is opposite for the two eye, 
Cam1corrAvg(2) = Cam1corr0(2)/2;    % ie sign(Cam1corr0)~=sign(Cam2corr0)
Cam2corrAvg(2) = Cam2corr0(2)/2;    % Perhaps only do corrections if OppSignCheck = 1, otherwise ignore. 
                                    % Shifts with opposite sign should also have similar magnitude. Add check for that? 
Cam1corrAvg = Cam1corrAvg.*OppSignCheck; % Set corrections to 0 if cam1 & cam2 shifts have same sign.
Cam2corrAvg = Cam2corrAvg.*OppSignCheck;

eyeLcorr = Cam1corrAvg / rescale_factor;
eyeRcorr = Cam2corrAvg / rescale_factor;

if ~isempty(varargin)
    %%% Plot Figures %%%                       
    figure(1);clf;
    subplot(2,2,1)
    imshow(im1);hold on;title({'Camera 1 with Target 1 (blue box)','Optimal pos of Tar1 on Cam1 (black dashed)'})
    h = gca;
    h.Visible = 'On';
    line([x1,x1],[y1,y2])
    line([x2,x2],[y1,y2])
    line([x1,x2],[y1,y1])
    line([x1,x2],[y2,y2])
    line([xpeak1-Tsize,xpeak1-Tsize],[ypeak1-Tsize,ypeak1],'LineStyle','--','Color','k')
    line([xpeak1,xpeak1],[ypeak1-Tsize,ypeak1],'LineStyle','--','Color','k')
    line([xpeak1-Tsize,xpeak1],[ypeak1-Tsize,ypeak1-Tsize],'LineStyle','--','Color','k')
    line([xpeak1-Tsize,xpeak1],[ypeak1,ypeak1],'LineStyle','--','Color','k')
    %plot(x1,y1,'kx')
    plot3(xpeak1,ypeak1,max_c1,'rx','MarkerSize',18)
    plot3(xpeak1 - Tsize,ypeak1-Tsize,max_c1,'kx')
    %figure(2);clf;
    subplot(2,2,2)
    imshow(im2);hold on;title({'Camera 2 with Target 2 (blue box)','Optimal pos of Tar2 on Cam2 (black dashed)'})
    h = gca;
    h.Visible = 'On';
    line([x1,x1],[y1,y2])
    line([x2,x2],[y1,y2])
    line([x1,x2],[y1,y1])
    line([x1,x2],[y2,y2])
    line([xpeak2-Tsize,xpeak2-Tsize],[ypeak2-Tsize,ypeak2],'LineStyle','--','Color','k')
    line([xpeak2,xpeak2],[ypeak2-Tsize,ypeak2],'LineStyle','--','Color','k')
    line([xpeak2-Tsize,xpeak2],[ypeak2-Tsize,ypeak2-Tsize],'LineStyle','--','Color','k')
    line([xpeak2-Tsize,xpeak2],[ypeak2,ypeak2],'LineStyle','--','Color','k')
    %plot(x1,y1,'*')
    plot3(xpeak2,ypeak2,max_c2,'rx','MarkerSize',18)
    plot3(xpeak2 - Tsize,ypeak2-Tsize,max_c2,'kx')
    %figure(11);clf;
    subplot(2,2,3)
    surf(yvect,xvect,(c1),'EdgeColor','none');title({'NormXcorr2(Target 1,Camera 1)'})
    colorbar;view(0,90);axis tight;hold on;
    plot3(xpeak1,ypeak1,max_c1,'rx','MarkerSize',18)
    %figure(22);clf;
    subplot(2,2,4)
    surf(yvect,xvect,(c2),'EdgeColor','none');title({'NormXcorr2(Target 2,Camera 2)'})
    colorbar;view(0,90);axis tight;hold on;
    plot3(xpeak2,ypeak2,max_c2,'rx','MarkerSize',18)

    figure(12);clf;
    subplot(2,2,2)
    imshow(im2);hold on;title({'Camera 2 with Target 2 (green box)','Optimal pos of Tar1 on Cam2 (red dashed)'})
    h = gca;
    h.Visible = 'On';
    line([x1,x1],[y1,y2],'Color','g')
    line([x2,x2],[y1,y2],'Color','g')
    line([x1,x2],[y1,y1],'Color','g')
    line([x1,x2],[y2,y2],'Color','g')
    line([xpeak12-Tsize,xpeak12-Tsize],[ypeak12-Tsize,ypeak12],'LineStyle','--','Color','r')
    line([xpeak12,xpeak12],[ypeak12-Tsize,ypeak12],'LineStyle','--','Color','r')
    line([xpeak12-Tsize,xpeak12],[ypeak12-Tsize,ypeak12-Tsize],'LineStyle','--','Color','r')
    line([xpeak12-Tsize,xpeak12],[ypeak12,ypeak12],'LineStyle','--','Color','r')
    text(100,nR-50, ['Cam1 offset: ',num2str(Cam1corr0(1)),' down, ',num2str(Cam1corr0(2)),' right'])
    text(100,nR-20, ['Move Cam1: ',num2str(Cam1corrAvg(1)),' down, ',num2str(Cam1corrAvg(2)),' right'],'Color','r')
    %plot(x1,y1,'kx')
    plot3(xpeak12,ypeak12,max_c12,'rx','MarkerSize',18)
    plot3(xpeak12-Tsize,ypeak12-Tsize,max_c12,'gx')
    %figure(21);clf;
    subplot(2,2,1)
    imshow(im1);hold on;title({'Camera 1 with Target 1 (red box)','Optimal pos of Tar2 on Cam1 (green dashed)'})
    h = gca;
    h.Visible = 'On';
    line([x1,x1],[y1,y2],'Color','r')
    line([x2,x2],[y1,y2],'Color','r')
    line([x1,x2],[y1,y1],'Color','r')
    line([x1,x2],[y2,y2],'Color','r')
    line([xpeak21-Tsize,xpeak21-Tsize],[ypeak21-Tsize,ypeak21],'LineStyle','--','Color','g')
    line([xpeak21,xpeak21],[ypeak21-Tsize,ypeak21],'LineStyle','--','Color','g')
    line([xpeak21-Tsize,xpeak21],[ypeak21-Tsize,ypeak21-Tsize],'LineStyle','--','Color','g')
    line([xpeak21-Tsize,xpeak21],[ypeak21,ypeak21],'LineStyle','--','Color','g')
    text(100,nR-50, ['Cam2 offset: ',num2str(Cam2corr0(1)),' down, ',num2str(Cam2corr0(2)),' right'])
    text(100,nR-20, ['Move Cam2: ',num2str(Cam2corrAvg(1)),' down, ',num2str(Cam2corrAvg(2)),' right'],'Color','r')
    %plot(x1,y1,'kx')
    plot3(xpeak21,ypeak21,max_c21,'rx','MarkerSize',18)
    plot3(xpeak21 - Tsize,ypeak21-Tsize,max_c21,'gx')
    %figure(13);clf;
    subplot(2,2,4)
    surf(yvect,xvect,c12,'EdgeColor','none');title({'NormXcorr2(Target 1,Camera 2)'})
    colorbar;view(0,90);axis tight;hold on;
    plot3(xpeak12,ypeak12,max_c12,'rx','MarkerSize',18)
    %plot3(y1,x1,max_c1,'*')
    %figure(31);clf;
    subplot(2,2,3)
    surf(yvect,xvect,c21,'EdgeColor','none');title({'NormXcorr2(Target 2,Camera 1)'})
    colorbar;view(0,90);axis tight;hold on;
    plot3(xpeak21,ypeak21,max_c21,'rx','MarkerSize',18)
    %plot3(y1,x1,max_c1,'*')
end

end
