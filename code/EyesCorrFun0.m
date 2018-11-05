function [eyeLcorr, eyeRcorr,T] = EyesCorrFun0(LeyeImg,ReyeImg)
%%% Input: Images from left and right eye cameras.
%%% Output: Nx,Ny pixels each camera must shift to align with other. 
%%% Dividing each by 2 gives the amount to shift both cams to meet in middle.
%%% 
%%% Remaining issue: To shift the center of the cameras image requires
%%% changing the camera pitch and yaw angles, but the required amount
%%% depends on the distance of the target from the cameras!
%%% Maybe assume distance to make correction, and see if it improves

tic

im10 = rgb2gray(LeyeImg);
im20 = rgb2gray(ReyeImg);

Tsize = 140;                % Set width/hight of square "Fovea" Targets
im1 = imresize(im10,0.5);   % Resize the images
im2 = imresize(im20,0.5);
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

[~, imax12] = max((c12(:)));  % Find the peak locations in correlograms
[~, imax21] = max((c21(:)));

[ypeak12, xpeak12] = ind2sub(size(c12),imax12(1));  % Convert peak locations into x,y coords
[ypeak21, xpeak21] = ind2sub(size(c21),imax21(1));
ypeak1=y2;
ypeak2=y2;
xpeak1=x2;
xpeak2=x2;

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

eyeLcorr = Cam1corrAvg;
eyeRcorr = Cam2corrAvg;
T=toc;
end
