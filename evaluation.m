% eavulation script
figure
hold on
axis equal
%plot3(posesGT(1,1:140),posesGT(2,1:140),posesGT(3,1:140),'g');
plot3(posesGT(1,1:400),posesGT(2,1:400),posesGT(3,1:400),'g');
plot3(poses(1,1:120),poses(2,1:120),poses(3,1:120),'r');
scatter3(poses(1,120),poses(2,120),poses(3,120),'r*')
scatter3(posesGT(1,400),posesGT(2,400),posesGT(3,400),'g*')
hold off


figure
hold on
axis equal
%plot3(posesGT(1,1:140),posesGT(2,1:140),posesGT(3,1:140),'g');
plot3(posesGT(1,1:700),posesGT(2,1:700),posesGT(3,1:700),'g');
plot3(poses(1,1:214),poses(2,1:214),poses(3,1:214),'r');
scatter3(poses(1,214),poses(2,214),poses(3,214),'r*')
scatter3(posesGT(1,700),posesGT(2,700),posesGT(3,700),'g*')
scatter3(posesGT(1,1),posesGT(2,1),posesGT(3,1),'g*')
hold off


figure
hold on
axis equal
%plot3(posesGT(1,1:140),posesGT(2,1:140),posesGT(3,1:140),'g');
plot3(posesGT(1,:),posesGT(2,:),posesGT(3,:),'g');
plot3(poses(1,:),poses(2,:),poses(3,:),'r');
scatter3(poses(1,end),poses(2,end),poses(3,end),'r*')
scatter3(posesGT(1,end),posesGT(2,end),posesGT(3,end),'g*')
scatter3(posesGT(1,1),posesGT(2,1),posesGT(3,1),'g*')
hold off


