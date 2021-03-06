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

%% Calculation of free c-space of robot
pospoint=zeros(3,1);
count_in = 1;
count_in2 = 1;
for i=-1.4:0.4:1.4
    for j=-1.2:0.4:1.4
        for k=-1.8:0.4:1.7
            for l=-1.9:0.4:1.7
                for m=-2:0.4:1.5
                    q=[i,j,k,l,m,0];
                    X=updateQ(q);
                    totalFlag=0;
                    for n=1:6
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
                        posp=[X(6,1);X(6,2);X(6,3)];
                        pospoint=horzcat(pospoint,posp);
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

%% Call lynxplanner to find path using RRTs
% Generate random points in 3D c-space
random_pts=datasample(pospoint.',200);

% Send random points to lynxPlanner
[path is_possible] = lynxPlanner(random_points, qstart, qend);

% Run FK on the path points and simulate using lynxServoSim

%% Checking for collision with obstacles
function F=spherecol(coodr)
x=coodr(1,1);
y=coodr(1,2);
z=coodr(1,3);
F=0;
if(((x-60)^2+(y+300)^2+(z-170)^2)<80^2 || ((x-60)^2+(y-100)^2+(z-150)^2)<80^2 || ((x-210)^2+(y+200)^2+(z-120)^2)<80^2 || ((x-210)^2+(y-100)^2+(z-150)^2)<80^2)
    F=1;
end
end

%% To find path between points qstart and qend using RRTs
function [path, is_possible] =lynxPlanner(random_pts, qstart, qend)
qstart_arr = repmat(qstart, length(random_pts), 1);
qend_arr = repmat(qend, length(random_pts), 1);

% Finding closest random points from qstart and qend
[qa, I] = squareform(pdist2(qstart_arr, random_pts,'euclidean', 'Smallest'));
[qb, I] = squareform(pdist2(qend_arr, random_pts,'euclidean', 'Smallest'));

x_start = [qstart(1) qa(1)];
y_start = [qstart(2) qa(2)];
z_start = [qstart(3) qa(3)];
line(x_start, y_start, z_start);
hold on;

% Find path from qa to qb using RRTs
path = zeros();
iter = 0;
newqa = zeros();
subtrees = 2;
for i = 1:1:length(random_pts)
    % Loop through every index to find random point to extend tree
    [closest5, I] = pdist2(qstart_arr, random_pts,'euclidean', 'Smallest', 5);
    rand_index = randsample(1:length(closest5q),2);
    newqa(:,end) = closest5(rand_index);
    for j = 1:1:subtrees % CHECK SUBTREES
        % Plot RRTs
        x = [qa(1) newqa(1)];
        y = [qa(2) newqa(2)];
        z = [qa(3) newqa(3)];
        line(x, y, z)
        % If qa = qb, stop searching for path
        if (newqa == qb)
            is_possible = 1;
            break;
        else
            qa = newqa;
        end
        iter = iter + 1;
        % For too many iterations, stop computation
        if iter > 500
            is_possible = 0;
        end
    end
    subtrees = subtrees*2;
    
    % Add every qa selected to paths
    path = horzcat(path, [qstart qa qend]);
end
end