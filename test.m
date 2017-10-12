
obstacle=zeros(1,3,4);
obstacle(1,:,1)=[60,-300,170];
obstacle(1,:,2)=[60,100,150];
obstacle(1,:,3)=[210,-200,120];
obstacle(1,:,4)=[210,100,250];

T = [0 0 1 292.10/25.4;0 1 0 0; 1 0 0 222.25/25.4;0 0 0 1]; %Unreachable
[q,T]=IK_lynx(T);
lynxServo(q);
T = [1 0 0 60/25.4;0 1 0 -300/25.4; 0 0 1 (170-70)/25.4;0 0 0 1]; %Unreachable
[q,T]=IK_lynx(T);
lynxServo(q);


% T = [0 0 1 2.36;0 1 0 100/25.4; 1 0 0 150/25.4;0 0 0 1]; %Unreachable
% [q,T]=IK_lynx(T);
% lynxServo(q);
% T = [0 0 1 210/25.4;0 1 0 -200/25.4; 1 0 0 120/25.4;0 0 0 1]; %Unreachable
% [q,T]=IK_lynx(T);
% lynxServo(q);
% T = [0 0 1 210/25.4;0 1 0 100/25.4; 1 0 0 250/25.4;0 0 0 1]; %Unreachable
% [q,T]=IK_lynx(T);
% lynxServo(q);
%home T = [0 0 1 292.10/25.4;0 1 0 0; 1 0 0 222.25/25.4;0 0 0 1]; %Unreachable


