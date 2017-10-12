% obstacle(1,:,1)=[60,-300,170];
% obstacle(1,:,2)=[60,100,150];
% obstacle(1,:,3)=[210,-200,120];
% obstacle(1,:,4)=[210,100,250];

% xw=180;
% yw=200;
% zw=150;
% F=spherecol(xw,yw,zw);
% if(F)
% th1 = atan(yw/xw);
% T = [0 cos(th1) sin(th1) xw/25.4;0 -sin(th1) cos(th1) yw/25.4; 1 0 0 zw/25.4;0 0 0 1]; %Unreachable
%  [q,T]=IK_lynx(T);
%  lynxServo(q);
% end

X_1=zeros(1,(15.^2)*(18.^2)*16);
Y_1=zeros(1,(15.^2)*(18.^2)*16);
Z_1=zeros(1,(15.^2)*(18.^2)*16);


X_2=zeros(1,(15.^2)*(18.^2)*16);
Y_2=zeros(1,(15.^2)*(18.^2)*16);
Z_2=zeros(1,(15.^2)*(18.^2)*16);

count_in = 1;
count_in2 = 1;
for (i=-1.4:0.3:1.4)
    for (j=-1.2:0.3:1.4)
        for(k=-1.8:0.3:1.7)
            for(l=-1.9:0.2:1.7)
                for(m=-2:0.2:1.5)
                    q=[i,j,k,l,m,0];
                    X=updateQ(q);
                    totalFlag=0;
                    for(n=1:6)
                    F=spherecol(X(n,:));
                    if(F==1)
                        totalFlag=1;
                    end
                    end
                    if(totalFlag==0)
                        X_1(1,count_in)=X(6,1);
                        Y_1(1,count_in)=X(6,2);
                        Z_1(1,count_in)=X(6,3);
                        count_in=count_in+1;
                        
                        %append q to c space 6 by n matrix
                    else
                        X_2(1,count_in2)=X(6,1);
                        Y_2(1,count_in2)=X(6,2);
                        Z_2(1,count_in2)=X(6,3);
                        count_in2=count_in2+1;
                        
                    end
                    
                end
            end
        end
    end
end

scatter3(X_1,Y_1,Z_1,1,'filled');
hold on;
scatter3(X_2,Y_2,Z_2,2,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[0 .75 .75]);

 
 function F=spherecol(coodr)
 x=coodr(1,1);
 y=coodr(1,2);
 z=coodr(1,3);
 F=0;
 if(((x-60)^2+(y+300)^2+(z-170)^2)<80^2 || ((x-60)^2+(y-100)^2+(z-150)^2)<80^2 || ((x-210)^2+(y+200)^2+(z-120)^2)<80^2 || ((x-210)^2+(y-100)^2+(z-150)^2)<80^2)
 F=1;
 end
 end
 