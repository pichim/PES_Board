function pry = quat2pry(q)
% Converts a quaternion [w, x, y, z] to pitch-roll-yaw (ZXY Tait-Bryan angles)
% Input:  q - 1x4 vector [w, x, y, z], normalized
% Output: pry - 1x3 vector [pitch, roll, yaw] in radians

% Extract quaternion components
qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

% Intermediate terms
sinR = 2.0 * (qw * qx + qy * qz);              % sin(roll)
sinP_cosR = 2.0 * (qw * qy - qx * qz);         % sin(pitch) * cos(roll)
cosP_cosR = 1.0 - 2.0 * (qx^2 + qy^2);         % cos(pitch) * cos(roll)
sinY_cosR = 2.0 * (qw * qz - qx * qy);         % sin(yaw) * cos(roll)
cosY_cosR = 1.0 - 2.0 * (qx^2 + qz^2);         % cos(yaw) * cos(roll)

% Clamp sin(roll) to [-1, 1] to avoid invalid asin
sinR = max(-1.0, min(1.0, sinR));
roll = asin(sinR);

% Compute pitch and yaw
pitch = atan2(sinP_cosR, cosP_cosR);
yaw   = atan2(sinY_cosR, cosY_cosR);

% Output as [pitch, roll, yaw]
pry = [pitch, roll, yaw];
end
