function [ pos, dactdtheta, dactdphi] = inverse_kin_jac( theta, phi, act)
%function to find the actuator positions, jacobian for given workspace 
%angles of the gimbal

%inputs- 
%theta = gimbal yaw 
%phi = gimbal pitch
%act = struct holding gimbal cal information
%cam = sphereical joint centers on camera in neutral pos
%neut = neutral position of end of actuators (i.e. when commanded to 0)
%dir = actuator direction
%link = link length

%outputs-
%pos = actuator displacement from neutral
%dactdtheta, dactdphi = row of jacobian matrix mapping from workspace 
%velocities to joint velocities

%%%%%%%%%%%%%%%%Gimbal Orientation to Joint Position Mapping%%%%%%%%%%%%%%%

%rotation matrix based on commanded theta, phi
Rcam=[cos(theta), sin(phi)*sin(theta), cos(phi)*sin(theta);
      0, cos(phi), -sin(phi);
      -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

%new location of camera head spherical joints
a=Rcam*act.cam;

%find actuator pos
x=a(1)-act.neut(1);
y=a(2)-act.neut(2);
z=a(3)-act.neut(3);
l2=act.link^2 - x^2 - y^2 - z^2;

if ((-y*act.dir(2) - z*act.dir(3))^2 + l2) < 0
    quad_sqrt = nan;
    pos = nan;
else
    quad_sqrt = sqrt((-y*act.dir(2) - z*act.dir(3))^2 + l2);
    pos = z*act.dir(3) + y*act.dir(2) - quad_sqrt;
end

%%%%%%%%%%%%END Gimbal Orientation to Joint Position Mapping%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%Gimbal Velocity to Joint Velocity Mapping%%%%%%%%%%%%%%%%%%
%calculate the jacobian
dxdtheta=-sin(theta)*act.cam(1) + sin(phi)*cos(theta)*act.cam(2) + cos(phi)*cos(theta)*act.cam(3);
dxdphi=cos(phi)*sin(theta)*act.cam(2) - sin(phi)*sin(theta)*act.cam(3);

dydtheta=0;
dydphi=-sin(phi)*act.cam(2) - cos(phi)*act.cam(3);

dzdtheta=-cos(theta)*act.cam(1) - sin(phi)*sin(theta)*act.cam(2) - cos(phi)*sin(theta)*act.cam(3);
dzdphi=cos(phi)*cos(theta)*act.cam(2) - sin(phi)*cos(theta)*act.cam(3);

dl2dtheta=-2*x*dxdtheta - 2*y*dydtheta - 2*z*dzdtheta;
dl2dphi=-2*x*dxdphi - 2*y*dydphi - 2*z*dzdphi;

dactdtheta = dzdtheta*act.dir(3) + dydtheta*act.dir(2) - ...
            (2*act.dir(2)*y*dydtheta + 2*act.dir(3)*act.dir(2)*(z*dydtheta + y*dzdtheta) + ...
             2*act.dir(3)*z*dzdtheta + dl2dtheta) / (2*quad_sqrt);
dactdphi   = dzdphi*act.dir(3) + dydphi*act.dir(2) - ...
            (2*act.dir(2)*y*dydphi + 2*act.dir(3)*act.dir(2)*(z*dydphi + y*dzdphi) + ... 
            2*act.dir(3)*z*dzdphi + dl2dphi) / (2*quad_sqrt);
        
%%%%%%%%%%%END Gimbal Velocity to Joint Velocity Mapping%%%%%%%%%%%%%%%%%%%

end

