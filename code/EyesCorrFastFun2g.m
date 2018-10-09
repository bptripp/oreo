function [eyeLcorr, eyeRcorr,T] = EyesCorrFastFun2g(LeyeImg,ReyeImg,Sredux,Ts,Ms,Sig)
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

%im100 = imread('IMG_1245.jpg');
%im200 = imread('IMG_1246.jpg');

tic

im10 = rgb2gray(LeyeImg);
im20 = rgb2gray(ReyeImg);

Tsize = Ts;                % Set width/hight of square "Fovea" Targets
MaxShift = Ms;
im1 = imresize(im10,Sredux);   % Resize the images
im2 = imresize(im20,Sredux);
[nR,nC] = size(im1);        % Get the image size
G = gauss2d(nR,nC,Sig,Sig,nR/2,nC/2,0);
im1g = G.*double(im1);
im2g = G.*double(im2);
y1 = nR/2-Tsize/2;          % Get the upper left and lower right corners of the Targets
y2 = nR/2+Tsize/2;
x1 = nC/2-Tsize/2;
x2 = nC/2+Tsize/2;
y1ms = nR/2-Tsize/2-MaxShift;          % Get the upper left and lower right corners of the Shift Area
y2ms = nR/2+Tsize/2+MaxShift;
x1ms = nC/2-Tsize/2-MaxShift;
x2ms = nC/2+Tsize/2+MaxShift;
im1c = im1g(y1:y2,x1:x2);    % Create Target images from the original images.
im2c = im2g(y1:y2,x1:x2);
im1ms = im1g(y1ms:y2ms,x1ms:x2ms);    % Create Target images from the original images.
im2ms = im2g(y1ms:y2ms,x1ms:x2ms);
% im1cg = im1g(y1:y2,x1:x2);    % Create Target images from the original images.
% im2cg = im2g(y1:y2,x1:x2);
% im1msg = im1g(y1ms:y2ms,x1ms:x2ms);    % Create Target images from the original images.
% im2msg = im2g(y1ms:y2ms,x1ms:x2ms);



%%% Calculate the cross-correlations
% [c12] = normxcorr2(im1c,im2);   % Correlation of Target1 with Camera2
% [c21] = normxcorr2(im2c,im1);   % Correlation of Target2 with Camera1
[c12ms] = normxcorr2(im1c,im2ms);   % Correlation of Target1 with Camera2
[c21ms] = normxcorr2(im2c,im1ms);   % Correlation of Target2 with Camera1
% [c12msg] = normxcorr2(im1cg,im2msg);   % Correlation of Target1 with Camera2
% [c21msg] = normxcorr2(im2cg,im1msg);   % Correlation of Target2 with Camera1
% xvect = [1:size(c12,1)];        % Get the size of the correlograms
% yvect = [1:size(c12,2)];
% xvectms = [1:size(c12ms,1)];        % Get the size of the correlograms
% yvectms = [1:size(c12ms,2)];
% [c1] = normxcorr2(im1c,im1);    % Correlation of Target1 with Camera1 (should have max=1)
% [c2] = normxcorr2(im2c,im2);    % Correlation of Target2 with Camera2 (should have max=1)

% [max_c12, imax12] = max((c12(:)));  % Find the peak locations in correlograms
% [max_c21, imax21] = max((c21(:)));
[max_c12ms, imax12ms] = max((c12ms(:)));  % Find the peak locations in correlograms
[max_c21ms, imax21ms] = max((c21ms(:)));
% [max_c12msg, imax12msg] = max((c12msg(:)));  % Find the peak locations in correlograms
% [max_c21msg, imax21msg] = max((c21msg(:)));
%[max_c1, imax1] = max(abs(c1(:)));
%[max_c2, imax2] = max(abs(c2(:)));

% [ypeak12, xpeak12] = ind2sub(size(c12),imax12(1));  % Convert peak locations into x,y coords
% [ypeak21, xpeak21] = ind2sub(size(c21),imax21(1));
[ypeak12ms0, xpeak12ms0] = ind2sub(size(c12ms),imax12ms(1));  % Convert peak locations into x,y coords
[ypeak21ms0, xpeak21ms0] = ind2sub(size(c21ms),imax21ms(1));
ypeak12ms = ypeak12ms0 + (nR/2-Tsize/2-MaxShift);
ypeak21ms = ypeak21ms0 + (nR/2-Tsize/2-MaxShift);
xpeak12ms = xpeak12ms0 + (nC/2-Tsize/2-MaxShift);
xpeak21ms = xpeak21ms0 + (nC/2-Tsize/2-MaxShift);
% [ypeak12msg0, xpeak12msg0] = ind2sub(size(c12msg),imax12msg(1));  % Convert peak locations into x,y coords
% [ypeak21msg0, xpeak21msg0] = ind2sub(size(c21msg),imax21msg(1));
% ypeak12msg = ypeak12msg0 + (nR/2-Tsize/2-MaxShift);
% ypeak21msg = ypeak21msg0 + (nR/2-Tsize/2-MaxShift);
% xpeak12msg = xpeak12msg0 + (nC/2-Tsize/2-MaxShift);
% xpeak21msg = xpeak21msg0 + (nC/2-Tsize/2-MaxShift);
%[ypeak1, xpeak1] = ind2sub(size(c1),imax1(1));
%[ypeak2, xpeak2] = ind2sub(size(c2),imax2(1));
ypeak1=y2;
ypeak2=y2;
xpeak1=x2;
xpeak2=x2;


% Cam1corr0 = [ypeak1-ypeak12,xpeak1-xpeak12]; % How far 1 camera should move (in pixels) to match other camera.
% Cam2corr0 = [ypeak2-ypeak21,xpeak2-xpeak21]; % If you want to move 1 eye/camera to match the other, use these. 
Cam1corr0ms = [ypeak1-ypeak12ms,xpeak1-xpeak12ms]; % How far 1 camera should move (in pixels) to match other camera.
Cam2corr0ms = [ypeak2-ypeak21ms,xpeak2-xpeak21ms]; % If you want to move 1 eye/camera to match the other, use these. 
% Cam1corr0msg = [ypeak1-ypeak12msg,xpeak1-xpeak12msg]; % How far 1 camera should move (in pixels) to match other camera.
% Cam2corr0msg = [ypeak2-ypeak21msg,xpeak2-xpeak21msg]; % If you want to move 1 eye/camera to match the other, use these. 

% OppSignCheck(1) = sign(Cam1corr0(1))~=sign(Cam2corr0(1));
% OppSignCheck(2) = sign(Cam1corr0(2))~=sign(Cam2corr0(2));
OppSignCheckMs(1) = sign(Cam1corr0ms(1))~=sign(Cam2corr0ms(1));
OppSignCheckMs(2) = sign(Cam1corr0ms(2))~=sign(Cam2corr0ms(2));
% OppSignCheckMsg(1) = sign(Cam1corr0msg(1))~=sign(Cam2corr0msg(1));
% OppSignCheckMsg(2) = sign(Cam1corr0msg(2))~=sign(Cam2corr0msg(2));

% Cam1corrAvg(1) = Cam1corr0(1)/2;    % If you want to move both cameras to meet in the "middle", move each by half their offset amount. 
% Cam2corrAvg(1) = Cam2corr0(1)/2;    % This makes most sense when the offset direction is opposite for the two eye, 
% Cam1corrAvg(2) = Cam1corr0(2)/2;    % ie sign(Cam1corr0)~=sign(Cam2corr0)
% Cam2corrAvg(2) = Cam2corr0(2)/2;    % Perhaps only do corrections if OppSignCheck = 1, otherwise ignore. 

Cam1corrAvgMs(1) = Cam1corr0ms(1)/2;    % If you want to move both cameras to meet in the "middle", move each by half their offset amount. 
Cam2corrAvgMs(1) = Cam2corr0ms(1)/2;    % This makes most sense when the offset direction is opposite for the two eye, 
Cam1corrAvgMs(2) = Cam1corr0ms(2)/2;    % ie sign(Cam1corr0)~=sign(Cam2corr0)
Cam2corrAvgMs(2) = Cam2corr0ms(2)/2;    % Perhaps only do corrections if OppSignCheck = 1, otherwise ignore. 
% Cam1corrAvgMsg(1) = Cam1corr0msg(1)/2;    % If you want to move both cameras to meet in the "middle", move each by half their offset amount. 
% Cam2corrAvgMsg(1) = Cam2corr0msg(1)/2;    % This makes most sense when the offset direction is opposite for the two eye, 
% Cam1corrAvgMsg(2) = Cam1corr0msg(2)/2;    % ie sign(Cam1corr0)~=sign(Cam2corr0)
% Cam2corrAvgMsg(2) = Cam2corr0msg(2)/2;    % Perhaps only do corrections if OppSignCheck = 1, otherwise ignore. 

% Shifts with opposite sign should also have similar magnitude. Add check for that? 
% Cam1corrAvg = Cam1corrAvg.*OppSignCheck; % Set corrections to 0 if cam1 & cam2 shifts have same sign.
% Cam2corrAvg = Cam2corrAvg.*OppSignCheck;
Cam1corrAvgMs = Cam1corrAvgMs.*OppSignCheckMs; % Set corrections to 0 if cam1 & cam2 shifts have same sign.
Cam2corrAvgMs = Cam2corrAvgMs.*OppSignCheckMs;
% Cam1corrAvgMsg = Cam1corrAvgMsg.*OppSignCheckMsg; % Set corrections to 0 if cam1 & cam2 shifts have same sign.
% Cam2corrAvgMsg = Cam2corrAvgMsg.*OppSignCheckMsg;

eyeLcorr = Cam1corrAvgMs;
eyeRcorr = Cam2corrAvgMs;

T=toc
end

% %%% Plot Figures %%%                       
% figure(13);clf;
% subplot(2,2,2)
% imshow(im2);hold on;title({'Camera 2 with Target 2 (green box)','Optimal pos of Tar1 on Cam2 (red dashed)'})
% h = gca;
% h.Visible = 'On';
% line([x1,x1],[y1,y2],'Color','g')
% line([x2,x2],[y1,y2],'Color','g')
% line([x1,x2],[y1,y1],'Color','g')
% line([x1,x2],[y2,y2],'Color','g')
% line([x1ms,x1ms],[y1ms,y2ms],'Color','y')
% line([x2ms,x2ms],[y1ms,y2ms],'Color','y')
% line([x1ms,x2ms],[y1ms,y1ms],'Color','y')
% line([x1ms,x2ms],[y2ms,y2ms],'Color','y')
% line([xpeak12ms-Tsize,xpeak12ms-Tsize],[ypeak12ms-Tsize,ypeak12ms],'LineStyle','--','Color','r')
% line([xpeak12ms,xpeak12ms],[ypeak12ms-Tsize,ypeak12ms],'LineStyle','--','Color','r')
% line([xpeak12ms-Tsize,xpeak12ms],[ypeak12ms-Tsize,ypeak12ms-Tsize],'LineStyle','--','Color','r')
% line([xpeak12ms-Tsize,xpeak12ms],[ypeak12ms,ypeak12ms],'LineStyle','--','Color','r')
% text(100,nR-50, ['Cam1 offset: ',num2str(Cam1corr0ms(1)),' down, ',num2str(Cam1corr0ms(2)),' right'])
% text(100,nR-20, ['Move Cam1: ',num2str(Cam1corrAvgMs(1)),' down, ',num2str(Cam1corrAvgMs(2)),' right'],'Color','r')
% %plot(x1,y1,'kx')
% plot3(xpeak12ms,ypeak12ms,max_c12ms,'rx','MarkerSize',18)
% plot3(xpeak12ms-Tsize,ypeak12ms-Tsize,max_c12ms,'gx')
% %figure(21);clf;
% subplot(2,2,1)
% imshow(im1);hold on;title({'Camera 1 with Target 1 (red box)','Optimal pos of Tar2 on Cam1 (green dashed)'})
% h = gca;
% h.Visible = 'On';
% line([x1,x1],[y1,y2],'Color','r')
% line([x2,x2],[y1,y2],'Color','r')
% line([x1,x2],[y1,y1],'Color','r')
% line([x1,x2],[y2,y2],'Color','r')
% line([x1ms,x1ms],[y1ms,y2ms],'Color','y')
% line([x2ms,x2ms],[y1ms,y2ms],'Color','y')
% line([x1ms,x2ms],[y1ms,y1ms],'Color','y')
% line([x1ms,x2ms],[y2ms,y2ms],'Color','y')
% line([xpeak21ms-Tsize,xpeak21ms-Tsize],[ypeak21ms-Tsize,ypeak21ms],'LineStyle','--','Color','g')
% line([xpeak21ms,xpeak21ms],[ypeak21ms-Tsize,ypeak21ms],'LineStyle','--','Color','g')
% line([xpeak21ms-Tsize,xpeak21ms],[ypeak21ms-Tsize,ypeak21ms-Tsize],'LineStyle','--','Color','g')
% line([xpeak21ms-Tsize,xpeak21ms],[ypeak21ms,ypeak21ms],'LineStyle','--','Color','g')
% text(100,nR-50, ['Cam2 offset: ',num2str(Cam2corr0ms(1)),' down, ',num2str(Cam2corr0ms(2)),' right'])
% text(100,nR-20, ['Move Cam2: ',num2str(Cam2corrAvgMs(1)),' down, ',num2str(Cam2corrAvgMs(2)),' right'],'Color','r')
% %plot(x1,y1,'kx')
% plot3(xpeak21ms,ypeak21ms,max_c21ms,'rx','MarkerSize',18)
% plot3(xpeak21ms - Tsize,ypeak21ms-Tsize,max_c21ms,'gx')
% %figure(13);clf;
% subplot(2,2,4)
% surf(yvectms,xvectms,c12ms,'EdgeColor','none');title({'NormXcorr2(Target 1,Camera 2)'})
% colorbar;view(0,90);axis tight;hold on;
% plot3(xpeak12ms0,ypeak12ms0,max_c12ms,'rx','MarkerSize',18)
% %plot3(y1,x1,max_c1,'*')
% %figure(31);clf;
% subplot(2,2,3)
% surf(yvectms,xvectms,c21ms,'EdgeColor','none');title({'NormXcorr2(Target 2,Camera 1)'})
% colorbar;view(0,90);axis tight;hold on;
% plot3(xpeak21ms0,ypeak21ms0,max_c21ms,'rx','MarkerSize',18)
% %plot3(y1,x1,max_c1,'*')

% figure(12);clf;
% subplot(2,2,2)
% imshow(im2);hold on;title({'Camera 2 with Target 2 (green box)','Optimal pos of Tar1 on Cam2 (red dashed)'})
% h = gca;
% h.Visible = 'On';
% line([x1,x1],[y1,y2],'Color','g')
% line([x2,x2],[y1,y2],'Color','g')
% line([x1,x2],[y1,y1],'Color','g')
% line([x1,x2],[y2,y2],'Color','g')
% line([xpeak12-Tsize,xpeak12-Tsize],[ypeak12-Tsize,ypeak12],'LineStyle','--','Color','r')
% line([xpeak12,xpeak12],[ypeak12-Tsize,ypeak12],'LineStyle','--','Color','r')
% line([xpeak12-Tsize,xpeak12],[ypeak12-Tsize,ypeak12-Tsize],'LineStyle','--','Color','r')
% line([xpeak12-Tsize,xpeak12],[ypeak12,ypeak12],'LineStyle','--','Color','r')
% text(100,nR-50, ['Cam1 offset: ',num2str(Cam1corr0(1)),' down, ',num2str(Cam1corr0(2)),' right'])
% text(100,nR-20, ['Move Cam1: ',num2str(Cam1corrAvg(1)),' down, ',num2str(Cam1corrAvg(2)),' right'],'Color','r')
% %plot(x1,y1,'kx')
% plot3(xpeak12,ypeak12,max_c12,'rx','MarkerSize',18)
% plot3(xpeak12-Tsize,ypeak12-Tsize,max_c12,'gx')
% %figure(21);clf;
% subplot(2,2,1)
% imshow(im1);hold on;title({'Camera 1 with Target 1 (red box)','Optimal pos of Tar2 on Cam1 (green dashed)'})
% h = gca;
% h.Visible = 'On';
% line([x1,x1],[y1,y2],'Color','r')
% line([x2,x2],[y1,y2],'Color','r')
% line([x1,x2],[y1,y1],'Color','r')
% line([x1,x2],[y2,y2],'Color','r')
% line([xpeak21-Tsize,xpeak21-Tsize],[ypeak21-Tsize,ypeak21],'LineStyle','--','Color','g')
% line([xpeak21,xpeak21],[ypeak21-Tsize,ypeak21],'LineStyle','--','Color','g')
% line([xpeak21-Tsize,xpeak21],[ypeak21-Tsize,ypeak21-Tsize],'LineStyle','--','Color','g')
% line([xpeak21-Tsize,xpeak21],[ypeak21,ypeak21],'LineStyle','--','Color','g')
% text(100,nR-50, ['Cam2 offset: ',num2str(Cam2corr0(1)),' down, ',num2str(Cam2corr0(2)),' right'])
% text(100,nR-20, ['Move Cam2: ',num2str(Cam2corrAvg(1)),' down, ',num2str(Cam2corrAvg(2)),' right'],'Color','r')
% %plot(x1,y1,'kx')
% plot3(xpeak21,ypeak21,max_c21,'rx','MarkerSize',18)
% plot3(xpeak21 - Tsize,ypeak21-Tsize,max_c21,'gx')
% %figure(13);clf;
% subplot(2,2,4)
% surf(yvect,xvect,c12,'EdgeColor','none');title({'NormXcorr2(Target 1,Camera 2)'})
% colorbar;view(0,90);axis tight;hold on;
% plot3(xpeak12,ypeak12,max_c12,'rx','MarkerSize',18)
% %plot3(y1,x1,max_c1,'*')
% %figure(31);clf;
% subplot(2,2,3)
% surf(yvect,xvect,c21,'EdgeColor','none');title({'NormXcorr2(Target 2,Camera 1)'})
% colorbar;view(0,90);axis tight;hold on;
% plot3(xpeak21,ypeak21,max_c21,'rx','MarkerSize',18)
% %plot3(y1,x1,max_c1,'*')

% figure(1);clf;
% subplot(2,2,1)
% imshow(im1);hold on;title({'Camera 1 with Target 1 (blue box)','Optimal pos of Tar1 on Cam1 (black dashed)'})
% h = gca;
% h.Visible = 'On';
% line([x1,x1],[y1,y2])
% line([x2,x2],[y1,y2])
% line([x1,x2],[y1,y1])
% line([x1,x2],[y2,y2])
% line([x1ms,x1ms],[y1ms,y2ms],'Color','y')
% line([x2ms,x2ms],[y1ms,y2ms],'Color','y')
% line([x1ms,x2ms],[y1ms,y1ms],'Color','y')
% line([x1ms,x2ms],[y2ms,y2ms],'Color','y')
% line([xpeak1-Tsize,xpeak1-Tsize],[ypeak1-Tsize,ypeak1],'LineStyle','--','Color','k')
% line([xpeak1,xpeak1],[ypeak1-Tsize,ypeak1],'LineStyle','--','Color','k')
% line([xpeak1-Tsize,xpeak1],[ypeak1-Tsize,ypeak1-Tsize],'LineStyle','--','Color','k')
% line([xpeak1-Tsize,xpeak1],[ypeak1,ypeak1],'LineStyle','--','Color','k')
% %plot(x1,y1,'kx')
% plot3(xpeak1,ypeak1,max_c1,'rx','MarkerSize',18)
% plot3(xpeak1 - Tsize,ypeak1-Tsize,max_c1,'kx')
% %figure(2);clf;
% subplot(2,2,2)
% imshow(im2);hold on;title({'Camera 2 with Target 2 (blue box)','Optimal pos of Tar2 on Cam2 (black dashed)'})
% h = gca;
% h.Visible = 'On';
% line([x1,x1],[y1,y2])
% line([x2,x2],[y1,y2])
% line([x1,x2],[y1,y1])
% line([x1,x2],[y2,y2])
% line([x1ms,x1ms],[y1ms,y2ms],'Color','y')
% line([x2ms,x2ms],[y1ms,y2ms],'Color','y')
% line([x1ms,x2ms],[y1ms,y1ms],'Color','y')
% line([x1ms,x2ms],[y2ms,y2ms],'Color','y')
% line([xpeak2-Tsize,xpeak2-Tsize],[ypeak2-Tsize,ypeak2],'LineStyle','--','Color','k')
% line([xpeak2,xpeak2],[ypeak2-Tsize,ypeak2],'LineStyle','--','Color','k')
% line([xpeak2-Tsize,xpeak2],[ypeak2-Tsize,ypeak2-Tsize],'LineStyle','--','Color','k')
% line([xpeak2-Tsize,xpeak2],[ypeak2,ypeak2],'LineStyle','--','Color','k')
% %plot(x1,y1,'*')
% plot3(xpeak2,ypeak2,max_c2,'rx','MarkerSize',18)
% plot3(xpeak2 - Tsize,ypeak2-Tsize,max_c2,'kx')
% %figure(11);clf;
% subplot(2,2,3)
% surf(yvect,xvect,(c1),'EdgeColor','none');title({'NormXcorr2(Target 1,Camera 1)'})
% colorbar;view(0,90);axis tight;hold on;
% plot3(xpeak1,ypeak1,max_c1,'rx','MarkerSize',18)
% %figure(22);clf;
% subplot(2,2,4)
% surf(yvect,xvect,(c2),'EdgeColor','none');title({'NormXcorr2(Target 2,Camera 2)'})
% colorbar;view(0,90);axis tight;hold on;
% plot3(xpeak2,ypeak2,max_c2,'rx','MarkerSize',18)

%end