%Tstart=[enter value of t for start point];
%Tend=[enter value of t for end point];
% qstart=IK_lynx(Tstart);
% qsend=IK_lynx(Tend);

qstart=[0.6,1,0,0,0,10];
qsend=[1.4,1,0.5,0,0,10];

fl1=checkpointforcol(qstart);
fl2=checkpointforcol(qsend);
if fl1==0 && fl2==0
total_path = pathplan(qstart,qsend);
[nop v]=size(total_path);
for i=1:nop
    q = total_path(i,:);
    lynxServo(q)
    hold on;
    %pause(10)
end
else
    fprintf('invalid points');
end
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
    
    while pathfound ~=1 && ittration<noittration
        q=randomq();
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
    %reversepath
   
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
    pathrev=pathrev(3:end,:);
    
    
    
    pathforward=flipud(pathforward);
    total_path=vertcat(pathforward,pathrev)
% % %     pathrev
% % %     qstart_edge_tree;
% % %     qend_edge_tree;
% % %     qstarttree(end,:);
% % %     qendtree(end,:);
    fprintf('path_found');
else
    total_path=vertcat(qstart,qend);
end

end

% Check that angular limits are satisfied
    %lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15]; % Lower joint limits in radians (grip in inches)
    %upperLim = [1.4 1.4 1.7 1.7 1.5 30]; % Upper joint limits in radians (grip in inches)
    
function q=randomq()
q1=-1.4+(1.4+1.4)*rand;
q2=-1.2+(1.4+1.2)*rand;
q3=-1.8+(1.7+1.8)*rand;
q4=-1.9+(1.7+1.9)*rand;
q5=-2+(1.5+2)*rand;
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
for n=1:lenen
    qintend=qendtree(n,:);
    flagend=checklinecol(qintend,q);
    if flagend==0
        dist=distcheck(qintend,q);
        dist_index2=vertcat(dist_index2,[dist,n]);
        flag_et_total=0;
    end
end
dist2 = sortrows(dist_index2);
if flag_st_total==0 && flag_et_total==0
    qstarttree=vertcat(qstarttree,q);
    [lenqst mstrt]=size(qstarttree);
    qstart_edge_tree=vertcat(qstart_edge_tree,[lenqst,dist1(2,2)]);
    qendtree=vertcat(qendtree,q);
    [lenen mend]=size(qendtree);
    qend_edge_tree=vertcat(qend_edge_tree,[lenen,dist2(2,2)]);
    pathfound=1;
elseif flag_st_total==0
    qstarttree=vertcat(qstarttree,q);
    [lenqst mstrt]=size(qstarttree);
    qstart_edge_tree=vertcat(qstart_edge_tree,[lenqst,dist1(2,2)]);
elseif flag_et_total==0
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


function dist=distcheck(q1,q2)
dist=sqrt(((q2(1)-q1(1))^2)+((q2(2)-q1(2))^2)+((q2(3)-q1(3))^2)+((q2(4)-q1(4))^2)+((q2(5)-q1(5))^2));
end


%if flag=0 no coll
function flag=checklinecol(q1,q2)
discretization=10;
linkdiscretization=10;
stepq1=linspace(q1(1),q2(1),discretization);
stepq2=linspace(q1(2),q2(2),discretization);
stepq3=linspace(q1(3),q2(3),discretization);
stepq4=linspace(q1(4),q2(4),discretization);
stepq5=linspace(q1(5),q2(5),discretization);
for n=1:discretization
    flageachpoint=0;
    q=[stepq1(n),stepq2(n),stepq3(n),stepq4(n),stepq5(n),10];
    X=updateQ(q);
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

%flag=0 if no coll
function totalFlag=checkpointforcol(q)
    linkdiscretization=10;
    totalFlag=0;
    X=updateQ(q);
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
    for np=1:len
        F=spherecol(X(np,:));
        if(F==1)
            totalFlag=1;
        end
    end
end


function F=spherecol(coodr)
sphere_size=40;
cylinder_size=10;
x=coodr(1,1);
y=coodr(1,2);
z=coodr(1,3);
F=0;
if(((x-60)^2+(y+300)^2+(z-170)^2)<sphere_size^2 || ((x-60)^2+(y-100)^2+(z-150)^2)<sphere_size^2 || ((x-210)^2+(y+200)^2+(z-120)^2)<sphere_size^2 || ((x-210)^2+(y-100)^2+(z-150)^2)<sphere_size^2)
    F=1;
end
if (((((x-60)^2+(y+300)^2)<cylinder_size^2) && z>170) || ((((x-60)^2+(y-100)^2)<cylinder_size^2) && z>150)|| ((((x-210)^2+(y-200)^2)<cylinder_size^2) && z>120) || ((((x-210)^2+(y-100)^2)<cylinder_size^2) && z>150))
    F=1;
end
end