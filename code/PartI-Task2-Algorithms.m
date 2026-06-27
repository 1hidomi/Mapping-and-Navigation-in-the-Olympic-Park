names={'K1','K2','K3','K4','K5','K6','K7',...
       'S1','S2','S3','S4','S5','S6','S7','S8','S9','S10'};
N=numel(names);
name2id=containers.Map(names,1:N);
edges = {'K1','S2';
         'S2','K5';
         'S2','S3';
         'S3','K7';
         'S3','S1';
         'K7','S4';
         'S4','S5';
         'S5','K6';
         'K6','S7';
         'S7','S6';
         'S7','K4';
         'S7','S8';
         'K4','S8';
         'S8','S9';
         'S9','K2';
         'S9','S1';
         'K2','S10';
         'S10','K3';
         'K3','S1'};

function d = Distance(p1,p2)
r=6371000;
la1=deg2rad(p1(1));
lo1=deg2rad(p1(2));
la2=deg2rad(p2(1));
lo2=deg2rad(p2(2));
dla=la2-la1;
dlo=lo2-lo1;
a = sin(dla/2)^2 + cos(la1)*cos(la2)*sin(dlo/2)^2;
c = 2*atan2(sqrt(a),sqrt(1-a));
d = r*c;
end

adj=cell(N,1);
for i = 1:size(edges,1)
    u=name2id(edges{i,1});
    v=name2id(edges{i,2});
    w=Distance(gps_points(u,:),gps_points(v,:));
    adj{u}=[adj{u}; v w];
    adj{v}=[adj{v}; u w];
end

for u = 1:N
    A = adj{u};
    fprintf('%s -> ', names{u});
    for k = 1:size(A,1)
        v = round(A(k,1));
        w = A(k,2);
        fprintf('%s(%.1fm) ', names{v}, w);
    end
    fprintf('\n');
end

%segW:distance of the segment
%totalW:distance of all segments
%pathIDs:node IDs
%pathN:node names
function [pathN,segW,totalW] = pathDistance(pathIDs,adj,names)
if isempty(pathIDs)
    pathN={};
    segW=[];
    totalW=inf;
    return;
end
L=numel(pathIDs);
pathN=names(pathIDs);
segW=zeros(1,max(0,L-1));
for i=1:(L-1)
    u=pathIDs(i);
    v=pathIDs(i+1);
    A=adj{u};
    idx=find(round(A(:,1))==v,1);
    if isempty(idx)
        error("Edge not found from %d -> %d",u,v);
    end
    segW(i)=A(idx,2);
end
totalW=sum(segW);
end

function [pathIDs,pathN,segW,totalW] = bfsPath(adj,names,name2id,startName, ...
          goalName)
N=numel(names);
s=name2id(startName);
t=name2id(goalName);

visited=false(N,1);
parent=zeros(N,1);
parent(s)=0;
Q=zeros(N,1);
head=1;tail=1;
Q(tail)=s;
visited(s)=true;
found=false;

while head<=tail
    u=Q(head);
    head=head+1;
    if u==t
        found=true;
        break;
    end
    A=adj{u};
    for k=1:size(A,1)
        v=round(A(k,1));
        if ~visited(v)
            visited(v)=true;
            parent(v)=u;
            tail=tail+1;
            Q(tail)=v;
        end
    end
end
if ~found
    pathIDs=[];
    [pathN,segW,totalW]=pathDistance(pathIDs,adj,names);
    return;
end
pathIDs=t;
cur=t;
while cur~=s
    cur=parent(cur);
    pathIDs=[cur,pathIDs];
end
[pathN,segW,totalW]=pathDistance(pathIDs,adj,names);
end


function [pathIDs,pathN,segW,totalW,distance]=dijktraArray(adj,names,name2id, ...
             startName,goalName)
N=numel(names);
s=name2id(startName);
e=name2id(goalName);
distance=inf(N,1);
prev=zeros(N,1);
used=false(N,1);
distance(s)=0;

for iter=1:N
    best=inf;
    u=-1;
    for i=1:N
        if ~used(i)&&distance(i)<best
            best=distance(i);
            u=i;
        end
    end
    if u==-1||isinf(dist(u))
        break;
    end
    used(u)=true;
    if u==e
        break;
    end
    A=adj{u};
    for k=1:size(A,1)
        v=round(A(k,1));
        w=A(k,2);
        if ~used(v)
            alt=distance(u)+w;
            if alt<dist(v)
                dist(v)=alt;
                prev(V)=u;
            end
        end
    end
end
if isinf(dist(e))
    pathIDs=[];
    [pathN,segW,totalW]=pathDistance(pathIDs,adj,names);
    return;
end

pathIDs=e;
cur=e;
while cur~=s
    cur=prev(cur);
    pathIDs=[cur,pathIDs];
end
[pathN,segW,totalW]=pathDistance(pathIDs,adj,names);
end

function [pathIDs, pathNames, segW, totalW, distance] = dijkstraHeap(adj, names, name2id, startName, goalName)
    N = numel(names);
    s = name2id(startName);
    t = name2id(goalName);

    distance = inf(N,1);
    prev = zeros(N,1);
    distance(s) = 0;

    heap = zeros(0,2);
    heap = heapPush(heap, 0, s);

    visited = false(N,1);

    while ~isempty(heap)
        [heap, d_curr, curr] = heapPop(heap);
        if d_curr > distance(curr)
            continue;
        end

        if visited(curr)
            continue;
        end
        visited(curr) = true;

        if curr == t
            break;
        end

        A = adj{curr};
        for k = 1:size(A,1)
            v = round(A(k,1));
            w = A(k,2);
            if ~visited(v)
                alt = distance(curr) + w;
                if alt < distance(v)
                    distance(v) = alt;
                    prev(v) = curr;
                    heap = heapPush(heap, alt, v);
                end
            end
        end
    end

    if isinf(distance(t))
        pathIDs = [];
        [pathNames, segW, totalW] = pathDistance(pathIDs, adj, names);
        return;
    end
    pathIDs = t;
    cur = t;
    while cur ~= s
        cur = prev(cur);
        pathIDs = [cur, pathIDs];
    end

    [pathNames, segW, totalW] = pathDistance(pathIDs, adj, names);
end

function heap = heapPush(heap, key, node)
    heap(end+1,:) = [key node];
    i = size(heap,1);
    while i > 1
        p = floor(i/2);
        if heap(p,1) <= heap(i,1), break; end
        heap([p i],:) = heap([i p],:);
        i = p;
    end
end

function [heap, key, node] = heapPop(heap)
    key = heap(1,1);
    node = round(heap(1,2));
    last = heap(end,:);
    heap(end,:) = [];

    if ~isempty(heap)
        heap(1,:) = last;
        i = 1;
        n = size(heap,1);
        while true
            l = 2*i; r = 2*i + 1;
            if l > n, break; end
            c = l;
            if r <= n && heap(r,1) < heap(l,1)
                c = r;
            end
            if heap(i,1) <= heap(c,1), break; end
            heap([i c],:) = heap([c i],:);
            i = c;
        end
    end
end

function printPath(pathN,segW,totalW,titleStr)
if isempty(pathN)
    fprintf('%s: No path found.\n', titleStr)
    return;
end
fprintf('%s:\n',titleStr);
for i=1:numel(pathN)
    if i==1
        fprintf(' %s', pathN{i});
    else
        fprintf(' --(%.1fm)--> %s',segW(i-1),pathN{i});
    end
end
fprintf('\n Total=%.1f m\n\n',totalW);
end

%KD-Tree
isKey = startsWith(names, 'K');      
keyIDs = find(isKey);                
keyXY  = points(keyIDs, :); 

function node = kdBuild(X, IDs, depth)
if nargin < 3, depth = 0; end
if isempty(X)
    node = [];
    return;
end
axis = mod(depth, 2) + 1;        
[~, order] = sort(X(:,axis), 'ascend');
X = X(order,:);
IDs = IDs(order);

mid = ceil(size(X,1)/2);

node.axis = axis;
node.point = X(mid,:);                
node.id = IDs(mid);                 
node.left  = kdBuild(X(1:mid-1,:), IDs(1:mid-1), depth+1);
node.right = kdBuild(X(mid+1:end,:), IDs(mid+1:end), depth+1);
end

function [bestID, bestDistance, bestPoint] = kdNearest(node, q, bestID, bestDistance, bestPoint)
    if isempty(node)
        return;
    end
    
    distanceS = sum((q - node.point).^2);
    if distanceS < bestDistance
        bestDistance = distanceS;
        bestID = node.id;
        bestPoint = node.point;
    end

    axis = node.axis;
    diff = q(axis) - node.point(axis);
    if diff <= 0
        near = node.left;  far = node.right;
    else
        near = node.right; far = node.left;
    end
    [bestID, bestDistance, bestPoint] = kdNearest(near, q, bestID, bestDistance, bestPoint);
    if diff^2 < bestDistance
        [bestID, bestDistance, bestPoint] = kdNearest(far, q, bestID, bestDistance, bestPoint);
    end
end