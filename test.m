
obstacle=zeros(1,3,4);
obstacle(1,:,1)=[60,-300,170];
obstacle(1,:,2)=[60,100,150];
obstacle(1,:,3)=[210,-200,120];
obstacle(1,:,4)=[210,100,250];
xw=100;
yw=100;
th1 = atan(yw/xw);
T = [0 cos(th1) sin(th1) xw/25.4;0 -sin(th1) cos(th1) yw/25.4; 1 0 0 150/25.4;0 0 0 1]; %Unreachable
 [q,T]=IK_lynx(T);
 lynxServo(q);
 
 
 function F=collchecksphere(x,y,z)
 F=1;
 if((square(x-60)+square(y+300)+square(z-170))<64 || (square(x-60)+square(y-100)+square(z-150))<64 || (square(x-210)+square(y+200)+square(z-120))<64 || (square(x-210)+square(y-100)+square(z-150))<64)
 flag=0;
 end

%          q=[1,0,0,0,0,0];
%          [X, T]=updateQ(q);
%          lynxServo(q);         
% T = [1 0 0 60/25.4;0 1 0 -300/25.4; 0 0 1 (170-70)/25.4;0 0 0 1]; %Unreachable
% [q,T]=IK_lynx(T);
% lynxServo(q);
% for (i=-1.4:0.3:1.4)
%     for (j=-1.2:0.3:1.4)
%         for(k=-1.8:0.3:1.7)
%             for(l=-1.9:0.2:1.7)
%                 for(m=-2:0.2:1.5)
%                     q=[i,j,k,l,m,0];
%                     X=updateQ(q);
%                     for(n=1:5)
%                     X(:,n)
%                     end    
%                 end
%             end
%         end
%     end
% end

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


