function [left_correction, right_correction, image_left, image_right] = get_image_offset()
    % Collects an image from each of the robot's cameras and compares
    % them to estimate how they should be moved in order to align well
    % at the centre of the image. 
    % 
    % left_correction: suggested corrective movement for left camera 
    %   to meet right camera in the middle ([ver hor] pixels)

    vid_left = videoinput('pointgrey', 1, 'F7_BayerRG8_1328x1048_Mode0');
    vid_right = videoinput('pointgrey', 2, 'F7_BayerRG8_1328x1048_Mode0');

    image_left = getsnapshot(vid_left);
    image_right = getsnapshot(vid_right);

%     figure
%     subplot(1,2,1), imagesc(image_left); %set(gca, 'Visible', 'off')
%     subplot(1,2,2), imagesc(image_right); %set(gca, 'Visible', 'off')
%     set(gcf, 'Position', [261 228 1022 420])
    
    tic
    [left_correction, right_correction] = EyesCorrFun(image_left, image_right);
    toc
    
    % direction cameras should move
    left_correction(1) = -left_correction(1);
    right_correction(1) = -right_correction(1);
    
end