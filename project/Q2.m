% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end effector at xGoal.
function qMilestones = Q2(rob,sphereCenter,sphereRadius,qStart,xGoal)
    qMilestones = qStart;
    lim = rob.qlim;
    min = lim(1,1);
    max = lim(1,2);
    qEnd = rob.ikine(transl(xGoal),qStart,[1,1,1,0,0,0]);
    qCurrent = qStart;
    qVisited = qStart;
    while(norm(qCurrent-qEnd)>0.05)
    %for z=1:100
        %temp =  rand(3,4)*(0.001)+qCurrent;

          randomConfig =growing(rob,qVisited);
          qVisited = [qVisited;randomConfig];
          temp = randomConfig - qCurrent;
         temp = temp/norm(temp);
         result = norm(qCurrent-qEnd);
        for i = 1:20
            qNext = qCurrent + temp*0.05*i;
                collision = Q1(rob,qCurrent,qNext,sphereCenter,sphereRadius);
                if(collision ==1)
                    break
                end
                    a = norm(qNext-qEnd);
                if(a>result)
                     break
                end
              qCurrent = qNext;
              result = norm(qCurrent-qEnd);  
      
        end
        
        qMilestones = [qMilestones;qCurrent];
        qVisited = [qVisited;qCurrent];
        
    end
end

function growingNode = growing(rob,qChecked)
     lim = rob.qlim;
    min = lim(1,1);
    max = lim(1,2);
   
    growingNode= rand(1,4)*(max-min)+min;
  while(ismember(growingNode,qChecked,'rows')==1)
     growingNode= rand(1,4)*(max-min)+min;
  end
  
end

