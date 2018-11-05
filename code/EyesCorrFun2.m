function [eyeLcorr, eyeRcorr,T] = EyesCorrFun2(LeyeImg,ReyeImg)
%%% Input: Images from left and right eye cameras.
%%% Output: Nx,Ny pixels each camera must shift to align with other. 
%%% Dividing each by 2 gives the amount to shift both cams to meet in middle.
%%% 
%%% Simplified by checking xcorr only once between im1 and im2 to get the
%%% shifts needed, divide by two and move each camera in opposite
%%% directions.
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
im2ms = im2(y1ms:y2ms,x1ms:x2ms);


%%% Calculate the cross-correlations
[c12ms] = normxcorr2(im1c,im2ms);   % Correlation of Target1 with Camera2

[~, imax12ms] = max((c12ms(:)));  % Find the peak locations in correlograms

[ypeak12ms0, xpeak12ms0] = ind2sub(size(c12ms),imax12ms(1));  % Convert peak locations into x,y coords
ypeak12ms = ypeak12ms0 + (nR/2-Tsize/2-MaxShift);
xpeak12ms = xpeak12ms0 + (nC/2-Tsize/2-MaxShift);
ypeak1=y2;
xpeak1=x2;

Cam1corr0ms = [ypeak1-ypeak12ms,xpeak1-xpeak12ms]; % How far 1 camera should move (in pixels) to match other camera.

Cam1corrAvgMs(1) = Cam1corr0ms(1)/2;    % If you want to move both cameras to meet in the "middle", move each by half the offset amount. 
Cam2corrAvgMs(1) = -Cam1corr0ms(1)/2;    % Make sure each camera moves in opposite direction 
Cam1corrAvgMs(2) = Cam1corr0ms(2)/2;   
Cam2corrAvgMs(2) = -Cam1corr0ms(2)/2;    

eyeLcorr = Cam1corrAvgMs/Sredux; % We reduced the image by a factor Sredux, so the pixel corrections for the original images have to be scaled too. 
eyeRcorr = Cam2corrAvgMs/Sredux;

T=toc;
end

