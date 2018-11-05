function [left_correction, right_correction, image_left, image_right] = get_image_offset()
    % Collects an image from each of the robot's cameras and compares
    % them to estimate how they should be moved in order to align well
    % at the centre of the image. 
    % 
    % left_correction: suggested corrective movement for left camera 
    %   to meet right camera in the middle ([ver hor] pixels)

    vid_left = videoinput('pointgrey', 1, 'F7_BayerRG8_1328x1048_Mode0');
    vid_right = videoinput('pointgrey', 2, 'F7_BayerRG8_1328x1048_Mode0');

    % getting some near-black images; try multiple times as a workaround 
    for i = 1:10
        image_left = getsnapshot(vid_left);
        if mean(image_left) > 10, break, end
        disp('retrying left image aquisition')
        pause(.05)
    end
    for i = 1:10
        image_right = getsnapshot(vid_right);
        if mean(image_right) > 10, break, end
        disp('retrying right image aquisition')
        pause(.05)
    end

%     figure
%     subplot(1,2,1), imagesc(image_left); %set(gca, 'Visible', 'off')
%     subplot(1,2,2), imagesc(image_right); %set(gca, 'Visible', 'off')
%     set(gcf, 'Position', [261 228 1022 420])
    
    tic
    [left_correction, right_correction] = EyesCorrFun(image_left, image_right);
%    [left_correction, right_correction,~] = EyesCorrFun0(image_left, image_right); speed should be faster for 0, then 1, fastest for 2
%    [left_correction, right_correction,~] = EyesCorrFun1(image_left, image_right);
%    [left_correction, right_correction,~] = EyesCorrFun2(image_left, image_right);
    toc
    
    % direction cameras should move
    left_correction(1) = -left_correction(1);
    right_correction(1) = -right_correction(1);
    
end
