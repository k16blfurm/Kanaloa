rosshutdown
rosinit

quatSub = rossubscriber('/odometry/filtered');
while(~isEmpty(quatSub))
quatData = receive(quatSub);

X = quatData.Pose.Pose.Orientation.X;
Y = quatData.Pose.Pose.Orientation.Y;
Z = quatData.Pose.Pose.Orientation.Z;
W = quatData.Pose.Pose.Orientation.W;

quatMatrix = [X,Y,Z,W];

[r1,r2,r3] = quat2angle(quatMatrix);

xDeg = rad2deg(r1);
yDeg = rad2deg(r2);
zDeg = rad2deg(r3);
end
