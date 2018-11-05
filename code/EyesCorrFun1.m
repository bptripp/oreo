function [eyeLcorr, eyeRcorr,T] = EyesCorrFun1(LeyeImg,ReyeImg)
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

Sredux = 0.5;               % Set the amount to reduce the image size by to speed up calculations
Tsize = 140;                % Set width/hight of square "Fovea" Targets
MaxShift = 80;              % Set the maximum number of pixel shifts to consider
im1 = imresize(im10,Sredux);   % Resize the images
im2 = imresize(im20,Sredux);
[nR,nC] = size(im1);        % Get the image size
y1 = nR/2-Tsize/2;          % Get the upper left and lower right corners of the Targets
y2 = nR/2+Tsize/2;
x1 = nC/2-Tsize/2;
x2 = nC/2+Tsize/2;
y1ms = nR/2-Tsize/2-MaxShift;          % Get the upper left and lower right corners of the Shift Area
y2ms = nR/2+Tsize/2+MaxShift;
x1ms = nC/2-Tsize/2-MaxShift;
x2ms = nC/2+Tsize/2+MaxShift;
im1c = im1(y1:y2,x1:x2);    % Create Target images from the original images.
im2c = im2(y1:y2,x1:x2);
im1ms = im1(y1ms:y2ms,x1ms:x2ms);    % Create Target images from the original images.
im2ms = im2(y1ms:y2ms,x1ms:x2ms);


%%% Calculate the cross-correlations
[c12ms] = normxcorr2(im1c,im2ms);   % Correlation of Target1 with Camera2
[c21ms] = normxcorr2(im2c,im1ms);   % Correlation of Target2 with Camera1

[~, imax12ms] = max((c12ms(:)));  % Find the peak locations in correlograms
[~, imax21ms] = max((c21ms(:)));

[ypeak12ms0, xpeak12ms0] = ind2sub(size(c12ms),imax12ms(1));  % Convert peak locations into x,y coords
[ypeak21ms0, xpeak21ms0] = ind2sub(size(c21ms),imax21ms(1));
ypeak12ms = ypeak12ms0 + (nR/2-Tsize/2-MaxShift);
ypeak21ms = ypeak21ms0 + (nR/2-Tsize/2-MaxShift);
xpeak12ms = xpeak12ms0 + (nC/2-Tsize/2-MaxShift);
xpeak21ms = xpeak21ms0 + (nC/2-Tsize/2-MaxShift);
ypeak1=y2;
ypeak2=y2;
xpeak1=x2;
xpeak2=x2;

Cam1corr0ms = [ypeak1-ypeak12ms,xpeak1-xpeak12ms]; % How far 1 camera should move (in pixels) to match other camera.
Cam2corr0ms = [ypeak2-ypeak21ms,xpeak2-xpeak21ms]; % If you want to move 1 eye/camera to match the other, use these. 

OppSignCheckMs(1) = sign(Cam1corr0ms(1))~=sign(Cam2corr0ms(1));
OppSignCheckMs(2) = sign(Cam1corr0ms(2))~=sign(Cam2corr0ms(2));

Cam1corrAvgMs(1) = Cam1corr0ms(1)/2;    % If you want to move both cameras to meet in the "middle", move each by half their offset amount. 
Cam2corrAvgMs(1) = Cam2corr0ms(1)/2;    % This makes most sense when the offset direction is opposite for the two eye, 
Cam1corrAvgMs(2) = Cam1corr0ms(2)/2;    % ie sign(Cam1corr0)~=sign(Cam2corr0)
Cam2corrAvgMs(2) = Cam2corr0ms(2)/2;    % Perhaps only do corrections if OppSignCheck = 1, otherwise ignore. 

% Shifts with opposite sign should also have similar magnitude. Add check for that? 
Cam1corrAvgMs = Cam1corrAvgMs.*OppSignCheckMs; % Set corrections to 0 if cam1 & cam2 shifts have same sign.
Cam2corrAvgMs = Cam2corrAvgMs.*OppSignCheckMs;

eyeLcorr = Cam1corrAvgMs/Sredux; % We reduced the image by a factor Sredux, so the pixel corrections for the original images have to be scaled too. 
eyeRcorr = Cam2corrAvgMs/Sredux;

T=toc;
end

