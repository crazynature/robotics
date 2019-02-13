% Smooth path given in qMilestones
% input: qMilestones -> nx4 vector of n milestones. 
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xm vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You should output a number of
%                    milestones m<=n.
function qMilestonesSmoothed = Q3(rob,qMilestones,sphereCenter,sphereRadius)
s=size(qMilestones);
len=s(1,1);
qEnd = qMilestones(len,:);
record = 1;
while(record<len)
    qCurrent = qMilestones(record,:);
    for i=len:-1:record
         collision = Q1(rob,qCurrent,qMilestones(i,:),sphereCenter,sphereRadius);
         if(collision==0)
             qMilestones([record+1:i-1],:)=[];
             break;
         end
    end
    s=size(qMilestones);
    
    len=s(1,1);
    record = record +1;
end

qMilestonesSmoothed=qMilestones;

end
