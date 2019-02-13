% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> result -> 2x2 vector of milestones. First element shows if
%        qStart is connected, second shows if qEnd is connected
function result = checkConnected(rob,sphereCenter,sphereRadius,qStart,xGoal)
   
    qEnd = [2.8368   -2.3679   -1.1261    3.1416    1.7453    0.3048];
   % qEnd = rob.ikine(transl(xGoal),qStart,[1,1,1,0,0,0])
 amount =50;
    sample = createSample(amount,rob,qStart,qEnd,sphereCenter,sphereRadius);
  result = isConnected(rob,sample,qStart,qEnd,sphereCenter,sphereRadius)
  
end

    
function cost = calculateCost(q,qStart,qEnd)
    cost = norm(qStart - q);
    end





function sample = createSample(amount,rob,qStart,qEnd,sphereCenter,sphereRadius)

sample =zeros(amount,length(rob.qlim));
sample = [qStart;qEnd;sample];
count = 2;

connected = 0;

    
    while(count <amount)
        node = createNode(rob);
        
       if(mutiObstacles(rob,node,sphereCenter,sphereRadius)==0)
     
            count = count + 1;
            
            sample(count,1:end)=node;
           
        end
    end
    
 %   result = isConnected(rob,sample,qStart,qEnd,sphereCenter,sphereRadius);
    




end

function result = isConnected(rob,map,qStart,qEnd,sphereCenter,sphereRadius)
connectStart =0;
connectEnd = 0;

for k = 1:length(map)
  
    if(connectStart==0)
        collision = checkCollision(rob,qStart,map(k,1:end),sphereCenter,sphereRadius);
        if(collision ==0)
            connectStart=1;
        end
    end
    if(connectEnd ==0)
        collision = checkCollision(rob,qEnd,map(k,1:end),sphereCenter,sphereRadius);
        if(collision ==0)
            connectEnd=1;
        end
        
    end
end


result = [connectStart connectEnd];


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
