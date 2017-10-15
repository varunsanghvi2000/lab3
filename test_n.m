random_pts = [1 1 1; 2 2 2; 3 3 3; 4 4 4; 5 5 5; 6 6 6];
qstart = [1 2 3];
qend = [5 5 6];
[path, is_possible] = lynxPlanner(random_pts, qstart, qend);

function [path, is_possible] =lynxPlanner(random_pts, qstart, qend)
qstart_arr = repmat(qstart, length(random_pts), 1);
qend_arr = repmat(qend, length(random_pts), 1);

% Finding closest random points from qstart and qend 
[dist2qa, Ia]= pdist2(qstart, random_pts, 'euclidean', 'Smallest', 1);
[dist2qb, Ib]= pdist2(random_pts, qend_arr, 'euclidean', 'Smallest', 1);
qa = random_pts(min(Ia));
qb = random_pts(min(Ib));

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
    for j = 1:1:subtrees % CHECK SUB-TREES
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
        % For too many itierations, stop computation
        if iter > 500
            is_possible = 0;
        end
    end
    subtrees = subtrees*2;
    
    % Add every qa selected to paths
    path = horzcat(path, [qstart qa qend]);
end
end