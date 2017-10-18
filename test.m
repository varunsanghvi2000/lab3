qstart=[0,0,0,0,0,10];
qsend=[1.4,0,0,0,0,10];
pathplan(qstart,qsend);

function pathplan(qstart,qend)

% Will eventually hold all random point associations of tree from qstart?
qstarttree =qstart;
% Will eventually hold all random point associations of tree from qend?
qendtree=qend;
qstart_edge_tree=zeros(1,2);
qend_edge_tree=zeros(1,2);
% Checks if qstart and qend fed are in collision with each other without
% tree generation. Returns value 1 if in collision.
pathcheck=checklinecol(qstarttree,qendtree);
pathfound=0;

% If collision between qstart and qend, generate random point to find
% alternate path
if pathcheck == 1           %collides
    while pathfound ~=1          %or no of iterations reached
        % Generate random q set within joint limits
        q=randomq();
        % Adds the random point to the growing tree list to either
        % start_tree or end_tree
        [qstarttree,qendtree,qstart_edge_tree,qend_edge_tree,pathfound]=addpointtolist(qstarttree,qendtree,qstart_edge_tree,qend_edge_tree,q);
        %fprintf('test');
    end
    qstart_edge_tree
    qend_edge_tree
    qstarttree(end,:)
    qendtree(end,:)
    fprintf('path_found');
else
    lynxServo(qstart);
    lynxServo(qend);
end
end

function q=randomq()
% Generates random q set within joint limits
q1=-1.4+(1.4+1.4)*rand;
q2=-1.4+(1.4+1.4)*rand;
q3=-1.4+(1.4+1.4)*rand;
q4=-1.4+(1.4+1.4)*rand;
q5=-1.4+(1.4+1.4)*rand;
% Assumes end effector theta to be 10. End-effector configuration does not
% contribute to the end effector position
q=[q1,q2,q3,q4,q5,10];
end

function [qstarttree,qendtree,qstart_edge_tree,qend_edge_tree,pathfound]=addpointtolist(qstarttree,qendtree,qstart_edge_tree,qend_edge_tree,q)
[lenqst mstrt]=size(qstarttree);
[lenen mend]=size(qendtree);
pathfound=0;
dist_index1=zeros(1,2);
dist_index2=zeros(1,2);
flag_st_total=1;
flag_et_total=1;

% Traverse through every point in the existing qstarttree to find the
% closest one in start tree from random point generated
for n=1:lenqst
    qintstart=qstarttree(n,:);
    flagstart=checklinecol(qintstart,q);
    if flagstart==0
        dist=distcheck(qintstart,q);
        dist_index1=vertcat(dist_index1,[dist,n]);
        flag_st_total=0;
    end
end
dist1 = sortrows(dist_index1);

% Traverse through every point in the existing qstarttree to find the
% closest one in start tree from random point generated
for n=1:lenen
    qintend=qendtree(n,:);
    % To check if random point generated is in collision with existing
    % start qtree
    flagend=checklinecol(qintend,q);
    if flagend==0
        % Finds distance between new q and each q in tree
        dist=distcheck(qintend,q);
        dist_index2=vertcat(dist_index2,[dist,n]);
        flag_et_total=0;
    end
end
dist2 = sortrows(dist_index2);

% Traverse through every point in the existing qstarttree to find the
% closest one in start tree from random point generated
if flag_st_total==0 && flag_et_total==0
    % If no collision path between new q and qstart and qend, append to
    % both trees
    qstarttree=vertcat(qstarttree,q);
    [lenqst mstrt]=size(qstarttree);
    qstart_edge_tree=vertcat(qstart_edge_tree,[lenqst,dist1(2,2)]);
    qendtree=vertcat(qendtree,q);
    [lenen mend]=size(qendtree);
    qend_edge_tree=vertcat(qend_edge_tree,[lenen,dist2(2,2)]);
    pathfound=1;
elseif flag_st_total==0
    % If free path from one of the points in qstart only, add point to qstarttree
    qstarttree=vertcat(qstarttree,q);
    [lenqst mstrt]=size(qstarttree);
    qstart_edge_tree=vertcat(qstart_edge_tree,[lenqst,dist1(2,2)]);
elseif flag_et_total==0
    % If free path from one of the points in qend only, add point to qendtree
    qendtree=vertcat(qendtree,q);
    [lenen mend]=size(qendtree);
    qend_edge_tree=vertcat(qend_edge_tree,[lenen,dist2(2,2)]);
end
% % if dist1(2,1)
% %     qendtree=vertcat(qendtree,q);
% %     %add edge
% % else
% %     qstarttree=vertcat(qstarttree,q);
% %     %add edge
% % end


% % flag=checklinecol(qendtree,q);
% % for n=1:len
% %     dist=distcheck(cspacepoints(n,:),q);
% %     dist_index=[dist,index];
% % end

% % flag=checklinecol(cspacepoints(B(1:2)),q);
% % if flag == 0
% %     cspacepoints=vertcat(cspacepoints,qend);
% %     %add to path plan
% % end
end

% Function to find distance between 2 sets of q points
function dist=distcheck(q1,q2)
dist=sqrt(((q2(1)-q1(1))^2)+((q2(2)-q1(2))^2)+((q2(3)-q1(3))^2)+((q2(4)-q1(4))^2)+((q2(5)-q1(5))^2));
end


%checklinecol function returns a flag value of 0 if there is no collision between set of q points with
%obstacle space
function flag=checklinecol(q1,q2)
discretization=100;
linkdiscretization=10;

% Discretise the qstart and qend values into 100 discretisations and check if every one of these points is in collision with obstacle space  
stepq1=linspace(q1(1),q2(1),discretization);
stepq2=linspace(q1(2),q2(2),discretization);
stepq3=linspace(q1(3),q2(3),discretization);
stepq4=linspace(q1(4),q2(4),discretization);
stepq5=linspace(q1(5),q2(5),discretization);

% For each of the 100 discretisations run loop to check for collision
for n=1:discretization
    flageachpoint=0;
    % q values for the 100 points on robot link
    q=[stepq1(n),stepq2(n),stepq3(n),stepq4(n),stepq5(n),10];
    % Find x,y and z positions for each of these points using forward
    % kinematics
    X=updateQ(q);
    xdesclink1=linspace(X(2,1),X(3,1),linkdiscretization);
    ydesclink1=linspace(X(2,2),X(3,2),linkdiscretization);
    zdesclink1=linspace(X(2,3),X(3,3),linkdiscretization);
    % Stores x y z values for this path point from qstart
    link1mat1=[xdesclink1',ydesclink1',zdesclink1']
    xdesclink2=linspace(X(3,1),X(4,1),linkdiscretization);
    ydesclink2=linspace(X(3,2),X(4,2),linkdiscretization);
    zdesclink2=linspace(X(3,3),X(4,3),linkdiscretization);
    % Stores x y z values for this path point from qend
    link1mat2=[xdesclink2',ydesclink2',zdesclink2'];
    % Concatenates x y z values for previous iterations path points from
    % start
    X=vertcat(X,link1mat1);
    X=vertcat(X,link1mat2);
    [len wid]=size(X);
    totalFlag=0;
    
    % For every row in X matrix check for collision with obstacle space
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

function F=spherecol(coodr)
sphere_size=40;
cylinder_size=20;
x=coodr(1,1);
y=coodr(1,2);
z=coodr(1,3);
F=0;

% Finds if x, y and z points fall within the geometric space (or volume) of
% the spherical obstacles
if(((x-60)^2+(y+300)^2+(z-170)^2)<sphere_size^2 || ((x-60)^2+(y-100)^2+(z-150)^2)<sphere_size^2 || ((x-210)^2+(y+200)^2+(z-120)^2)<sphere_size^2 || ((x-210)^2+(y-100)^2+(z-150)^2)<sphere_size^2)
    F=1;
end

% Finds if x, y and z points fall within the geometric space (or volume) of
% the strings that the spheres are suspended by
if((((x-60)^2+(y+300)^2)<cylinder_size^2 && z>170) || (((x-60)^2+(y-100)^2)<cylinder_size^2 && z>150)|| (((x-210)^2+(y-200)^2)<cylinder_size^2 && z>120) || (((x-210)^2+(y-100)^2)<cylinder_size^2 && z>150))
    F=1;
end
end