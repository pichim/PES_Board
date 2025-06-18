function rpy = quat2rpy(q)
% Converts a quaternion [w, x, y, z] to roll-pitch-yaw (ZYX Tait-Bryan angles)
% Input:  q - 1x4 vector [w, x, y, z], normalized
% Output: rpy - 1x3 vector [roll, pitch, yaw] in radians

% Extract quaternion components
qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

% Intermediate terms
sinP = -2.0 * (qx * qz - qw * qy);              % sin(pitch)
sinR_cosP = 2.0 * (qw * qx + qy * qz);          % sin(roll) * cos(pitch)
cosR_cosP = 1.0 - 2.0 * (qx^2 + qy^2);          % cos(roll) * cos(pitch)
sinY_cosP = 2.0 * (qw * qz + qx * qy);          % sin(yaw) * cos(pitch)
cosY_cosP = 1.0 - 2.0 * (qy^2 + qz^2);          % cos(yaw) * cos(pitch)

% Clamp sin(pitch) to [-1, 1] to avoid invalid asin
sinP = max(-1.0, min(1.0, sinP));
pitch = asin(sinP);

% Compute roll and yaw
roll = atan2(sinR_cosP, cosR_cosP);
yaw  = atan2(sinY_cosP, cosY_cosP);

% Output as [roll, pitch, yaw]
rpy = [roll, pitch, yaw];
end
