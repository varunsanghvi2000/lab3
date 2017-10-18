qstart=[0,1.4,1,1,1,10];
qsend=[1,-1.4,1.2,0,0.2,10];
pathplan(qstart,qsend);

function pathplan(qstart,qend)
qstarttree =qstart;
qendtree=qend;
pathcheck=checklinecol(qstarttree,qendtree);
if pathcheck == 1           %colides
   q=randomq();
   addpointtolist(qstarttree,qendtree,q)
else
    lynxServo(qstart);
    
    lynxServo(qend);
end

end

function q=randomq()
q1=-1.4+(1.4+1.4)*rand;
q2=-1.4+(1.4+1.4)*rand;
q3=-1.4+(1.4+1.4)*rand;
q4=-1.4+(1.4+1.4)*rand;
q5=-1.4+(1.4+1.4)*rand;
q=[q1,q2,q3,q4,q5];
end

function [qstarttree,qendtree]=addpointtolist(qstarttree,qendtree,q)
[lenqst mstrt]=size(qstarttree);
[lenen mend]=size(qendtree);

dist_index1=zeros(lenqst,2);
dist_index2=zeros(lenen,2);
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
    %pathfound
elseif flag_st_total==0
    qstarttree=vertcat(qstarttree,q);
elseif flag_et_total==0
    qstarttree=vertcat(qendtree,q);
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
stepq1=linspace(q1(1),q2(1),discretization);
stepq2=linspace(q1(2),q2(2),discretization);
stepq3=linspace(q1(3),q2(3),discretization);
stepq4=linspace(q1(4),q2(4),discretization);
stepq5=linspace(q1(5),q2(5),discretization);
    for n=1:discretization
        flageachpoint=0;
        q=[stepq1(n),stepq2(n),stepq3(n),stepq4(n),stepq5(n),10];
        X=updateQ(q);
                    totalFlag=0;
                    for np=1:6
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
sphere_size=20;
x=coodr(1,1);
y=coodr(1,2);
z=coodr(1,3);
F=0;
if(((x-60)^2+(y+300)^2+(z-170)^2)<sphere_size^2 || ((x-60)^2+(y-100)^2+(z-150)^2)<sphere_size^2 || ((x-210)^2+(y+200)^2+(z-120)^2)<sphere_size^2 || ((x-210)^2+(y-100)^2+(z-150)^2)<sphere_size^2)
    F=1;
end
end