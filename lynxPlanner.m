% pstart and pend point values can be modified in variables qstart and qend
qstart=[0.6,1,0,0,0,10];
qsend=[1.4,1,0.5,0,0,10];

% Initial check to ensure the initial points are not in collision space
fl1=checkpointforcol(qstart);
fl2=checkpointforcol(qsend);

% If qstart and qend are initially not in collision space, plan path
% between them
if fl1==0 && fl2==0
% To find execution time of planner
tic 
total_path = pathplan(qstart,qsend);
toc
[nop v]=size(total_path);

% To visualise the robot movement using simulator
lynxStart();
% pause(20) % To allow time for simulator to start
for i=1:nop
    q = total_path(i,:);
    lynxServo(q)
    hold on;
    % pause(10) %Uncomment this command and view different trajectory smoothness
end

else
    fprintf('No path found- given initial q values are in collision space');
end

%% Main function to plan the path between given start and end points
function total_path=pathplan(qstart,qend)

qstarttree =qstart;
qendtree=qend;
qstart_edge_tree=zeros(1,2);
qend_edge_tree=zeros(1,2);

pathcheck=checklinecol(qstarttree,qendtree);
pathfound=0;
ittration=0;
noittration=10000;

if pathcheck == 1           %colides
    % Until path is found, or if maximum number of iterations has not been
    % breached
    while pathfound ~=1 && ittration<noittration
        % To generate random point within joint limits
        q=randomq();
        % Keep adding random points to closest non-collision tree until
        % path is found
        [qstarttree,qendtree,qstart_edge_tree,qend_edge_tree,pathfound]=addpointtolist(qstarttree,qendtree,qstart_edge_tree,qend_edge_tree,q);
        ittration=ittration+1;
    end
    
    secondval=0;
    [index,w]=size(qstart_edge_tree);
    pathforward=zeros(1,6);
    
    while secondval~=1
        firstval=qstart_edge_tree(index,1);
        secondval=qstart_edge_tree(index,2);
        pathforward=vertcat(pathforward,qstarttree(firstval,:));
        index=qstart_edge_tree(firstval,2);
    end
    
    pathforward=vertcat(pathforward,qstart);
    secondvalrev=0;
    [indexrev,w]=size(qend_edge_tree);
    pathrev=zeros(1,6);
    
    while secondvalrev~=1
        firstvalrev=qstart_edge_tree(indexrev,1);
        secondvalrev=qstart_edge_tree(indexrev,2);
        pathrev=vertcat(pathrev,qendtree(firstvalrev,:));
        indexrev=qstart_edge_tree(firstvalrev,2);
    end
    
    pathrev=vertcat(pathrev,qend);
    pathforward=pathforward(2:end,:);
    
    % These variables contain forward path (between qstart and random
    % point) and reverse path (between qend and random point)
    pathrev=pathrev(3:end,:);
    pathforward=flipud(pathforward);
    
    % Contains the traversal path from qstart to qend
    total_path=vertcat(pathforward,pathrev)
    fprintf('path_found');
else
    % If direct path between qstart and qend exists, no need to generate
    % random points as path has been found
    total_path=vertcat(qstart,qend);
end

end

%% Function to generate random q point within joint limits of lynx
function q=randomq()
q1=-1.4+(1.4+1.4)*rand;
q2=-1.2+(1.4+1.2)*rand;
q3=-1.8+(1.7+1.8)*rand;
q4=-1.9+(1.7+1.9)*rand;
q5=-2+(1.5+2)*rand;
q=[q1,q2,q3,q4,q5,10];
end

%% Function to add random points to the closest (non-collision) growing tree 
function [qstarttree,qendtree,qstart_edge_tree,qend_edge_tree,pathfound]=addpointtolist(qstarttree,qendtree,qstart_edge_tree,qend_edge_tree,q)

[lenqst mstrt]=size(qstarttree);
[lenen mend]=size(qendtree);
pathfound=0;
dist_index1=zeros(1,2);
dist_index2=zeros(1,2);
flag_st_total=1;
flag_et_total=1;

% For every point in qstart tree
for n=1:lenqst
    qintstart=qstarttree(n,:);
    % Check for collision of every point in tree
    flagstart=checklinecol(qintstart,q);
    % Identify the distances between random point generated and every point
    % in qstart tree
    if flagstart==0
        dist=distcheck(qintstart,q);
        dist_index1=vertcat(dist_index1,[dist,n]);
        flag_st_total=0;
    end
end
dist1 = sortrows(dist_index1);

% For every point in qstart tree
for n=1:lenen
    qintend=qendtree(n,:);
    % Check for collision of every point in tree
    flagend=checklinecol(qintend,q);
    % Identify the distances between random point generated and every point
    % in qstart tree
    if flagend==0
        dist=distcheck(qintend,q);
        dist_index2=vertcat(dist_index2,[dist,n]);
        flag_et_total=0;
    end
end
dist2 = sortrows(dist_index2);

% If collision free path between a point in qstart tree and qend tree is
% found, random point can be added to both trees before we can complete
% path
if flag_st_total==0 && flag_et_total==0
    qstarttree=vertcat(qstarttree,q);
    [lenqst mstrt]=size(qstarttree);
    qstart_edge_tree=vertcat(qstart_edge_tree,[lenqst,dist1(2,2)]);
    qendtree=vertcat(qendtree,q);
    [lenen mend]=size(qendtree);
    qend_edge_tree=vertcat(qend_edge_tree,[lenen,dist2(2,2)]);
    pathfound=1;
elseif flag_st_total==0 
    % If collision free path is found between a point in qstart tree , random point can be added to qstart tree before we can complete
    % path
    qstarttree=vertcat(qstarttree,q);
    [lenqst mstrt]=size(qstarttree);
    qstart_edge_tree=vertcat(qstart_edge_tree,[lenqst,dist1(2,2)]);
elseif flag_et_total==0
    % If collision free path is found between a point in qend tree , random point can be added to qend tree before we can complete
    % path
    qendtree=vertcat(qendtree,q);
    [lenen mend]=size(qendtree);
    qend_edge_tree=vertcat(qend_edge_tree,[lenen,dist2(2,2)]);
end
end


%% Function to find the euclidean distance between 2 q points in 5D space
function dist=distcheck(q1,q2)
dist=sqrt(((q2(1)-q1(1))^2)+((q2(2)-q1(2))^2)+((q2(3)-q1(3))^2)+((q2(4)-q1(4))^2)+((q2(5)-q1(5))^2));
end


%% Function discretizes the following:
% 1. Discretize each robot link and check each point for collision
% 2. Discretize the 'line' joining the random points for collision
% Function returns a 0 if there is no collision
function flag=checklinecol(q1,q2)
discretization=10;
linkdiscretization=10;

% To discretize a pair 5D q values
stepq1=linspace(q1(1),q2(1),discretization);
stepq2=linspace(q1(2),q2(2),discretization);
stepq3=linspace(q1(3),q2(3),discretization);
stepq4=linspace(q1(4),q2(4),discretization);
stepq5=linspace(q1(5),q2(5),discretization);

% For every discretized point run loop
for n=1:discretization
    flageachpoint=0;
    
    % Find q value for point in loop and find forward kinematic solution
    % for the same
    q=[stepq1(n),stepq2(n),stepq3(n),stepq4(n),stepq5(n),10];
    X=updateQ(q);
    
    % Discretize every robot link to check for collision
    xdesclink1=linspace(X(2,1),X(3,1),linkdiscretization);
    ydesclink1=linspace(X(2,2),X(3,2),linkdiscretization);
    zdesclink1=linspace(X(2,3),X(3,3),linkdiscretization);
    link1mat1=[xdesclink1',ydesclink1',zdesclink1'];
    
    xdesclink2=linspace(X(3,1),X(4,1),linkdiscretization);
    ydesclink2=linspace(X(3,2),X(4,2),linkdiscretization);
    zdesclink2=linspace(X(3,3),X(4,3),linkdiscretization);
    link1mat2=[xdesclink2',ydesclink2',zdesclink2'];
    
    X=vertcat(X,link1mat1);
    X=vertcat(X,link1mat2);
    
    [len wid]=size(X);
    totalFlag=0;
    
    % For every row in X matrix, check for collision with obsatcle space
    for np=1:len
        F=spherecol(X(np,:));
        if(F==1)
            totalFlag=1;
        end
    end
    
    if(totalFlag==1)
        flageachpoint=1;
        break;
    end
end

flag=flageachpoint;
end

%% Function checks if initial qstart and qend values are in collision space
% Returns a '0' if no collision
function totalFlag=checkpointforcol(q)
    linkdiscretization=10;
    totalFlag=0;
    X=updateQ(q);
    
    % To discretize the path between qstart and qend
    xdesclink1=linspace(X(2,1),X(3,1),linkdiscretization);
    ydesclink1=linspace(X(2,2),X(3,2),linkdiscretization);
    zdesclink1=linspace(X(2,3),X(3,3),linkdiscretization);
    link1mat1=[xdesclink1',ydesclink1',zdesclink1'];
    
    xdesclink2=linspace(X(3,1),X(4,1),linkdiscretization);
    ydesclink2=linspace(X(3,2),X(4,2),linkdiscretization);
    zdesclink2=linspace(X(3,3),X(4,3),linkdiscretization);
    link1mat2=[xdesclink2',ydesclink2',zdesclink2'];
    
    X=vertcat(X,link1mat1);
    X=vertcat(X,link1mat2);
    [len wid]=size(X);
    
    % For every row in X matrix, check for collision with obsatcle space    
    for np=1:len
        F=spherecol(X(np,:));
        if(F==1)
            totalFlag=1;
        end
    end
end

%% Function to check collision of co-ordinates in 3D space with spherical obstacle and string suspending them
function F=spherecol(coodr)
% Define the geometry of obstacles in these variables
sphere_size=40;
cylinder_size=10;

x=coodr(1,1);
y=coodr(1,2);
z=coodr(1,3);
F=0;

% Collision check with sphere
if(((x-60)^2+(y+300)^2+(z-170)^2)<sphere_size^2 || ((x-60)^2+(y-100)^2+(z-150)^2)<sphere_size^2 || ((x-210)^2+(y+200)^2+(z-120)^2)<sphere_size^2 || ((x-210)^2+(y-100)^2+(z-150)^2)<sphere_size^2)
    F=1;
end

% Collision check with string suspending the sphere 
if (((((x-60)^2+(y+300)^2)<cylinder_size^2) && z>170) || ((((x-60)^2+(y-100)^2)<cylinder_size^2) && z>150)|| ((((x-210)^2+(y-200)^2)<cylinder_size^2) && z>120) || ((((x-210)^2+(y-100)^2)<cylinder_size^2) && z>150))
    F=1;
end
end