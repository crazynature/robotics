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
function qMilestones = part3(rob,sphereCenter,sphereRadius,qStart,xGoal)
    qMilestones = qStart;
    qEnd = rob.ikine(transl(xGoal));
  %  qEnd = rob.ikine6s(transl(xGoal),qStart,[1,1,1,0,0,0]);
    sample = createSample(rob,qStart,qEnd,sphereCenter,sphereRadius);
 %  graph = createGraph(sample);
 %   openlist = [qStart];
%    closelist = cell2mat(graph(1,1));
    
    
    
       
   
   qMilestones = sample;
       
        
    
end

function graph = createGraph(sample,sphereCenter,sphereRadius)

graph=cell(1,length(sample));
     for i = 1:length(sample)
         edge = [];
         for j = 1:length(sample)
             collision = part1(rob,sample(i,:),sample(j,1:end),sphereCenter,sphereRadius);
             if(collision ==0)
                 edge = [edge;sample(j,1:end)]
             end
         end
         graph(i) = edge;
     end
end



function sample = createSample(rob,qStart,qEnd,sphereCenter,sphereRadius)

sample =zeros(300,length(rob.qlim));
sample = [qStart;qEnd;sample];
count = 0;
result = [0 0];
connected = 0;

    
    while(count <300)
        node = createNode(rob);
        
       if(robotCollision(rob,node,sphereCenter(1:end,1),sphereRadius(1))==0)
     
            count = count + 1;
            
            sample(count,1:end)=node;
           
        end
    end
    
    result = isConnected(rob,sample,qStart,qEnd,sphereCenter,sphereRadius);
    




count
end

function result = isConnected(rob,map,qStart,qEnd,sphereCenter,sphereRadius)
connectStart =0;
connectEnd = 0;

for k = 1:length(map)
  
    if(connectStart==0)
        collision = part1(rob,qStart,map(k,1:end),sphereCenter,sphereRadius);
        if(collision ==0)
            connectStart=1;
        end
    end
    if(connectEnd ==0)
        collision = part1(rob,qEnd,map(k,1:end),sphereCenter,sphereRadius);
        if(collision ==0)
            connectEnd=1;
        end
        
    end
end


result = [connectStart connectEnd]


end

function node = createNode(rob)
    node = zeros(1,length(rob.qlim));
    for k = 1:length(rob.qlim)
        min = rob.qlim(k,1);
        max = rob.qlim(k,2);
        n = rand()*(max-min)+min;
        node(k) = n;
    end
end
