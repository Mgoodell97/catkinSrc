rosshutdown
rosinit
uav1_sub.gp = rossubscriber('/ground_truth_to_tf/pose');

pos_uav1 = [0,0];
idx = 1;
figure
plt1 = plot(0,0, 'r','LineWidth',5 );

while 1

   msg = receive(uav1_sub.gp);
   pos_uav1(idx,:) = [msg.Pose.Position.X, msg.Pose.Position.Y];
   idx = idx +1;
   set(plt1, 'XData', pos_uav1(:,1), 'YData', pos_uav1(:,2));

end