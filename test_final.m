qstart=[0,1.4,1,1,1,10];
qsend=[0.5,-1.4,1.2,0,0.2,10];
pathplan(qstart,qsend);

function pathplan(qstart,qend)
qstarttree =qstart;
qendtree=qend;
pathcheck=checklinecol(qstart,qend);
if pathcheck == 1           %colides
   q=randomq();
   addpointtolist(qstarttree,qendtree,q)
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

function cspacepoints=addpointtolist(qstarttree,qendtree,q)
[lenqst mstrt]=size(qstarttree);
[lenen mend]=size(qendtree);

for n=1:lenqst
    qint=qstarttree(n:1);
    flag=checklinecol(qint,q);
    if flag==0
    dist=distcheck(qstarttree(n,:),q);
    dist_index1=vertcat(dist_index,[dist,n]);
    end
end
B = sortrows1(dist_index);
for n=1:lenqst
    flag=checklinecol(qstarttree,q);
    if flag==0
    dist=distcheck(qstarttree(n,:),q);
    dist_index2=[dist,index];
    end
end
B = sortrows(dist_index);

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
x=coodr(1,1);
y=coodr(1,2);
z=coodr(1,3);
F=0;
if(((x-60)^2+(y+300)^2+(z-170)^2)<80^2 || ((x-60)^2+(y-100)^2+(z-150)^2)<80^2 || ((x-210)^2+(y+200)^2+(z-120)^2)<80^2 || ((x-210)^2+(y-100)^2+(z-150)^2)<80^2)
    F=1;
end
end