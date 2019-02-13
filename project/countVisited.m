% Calculate a path from qStart to xGoal
% input: qStart -> 1x6 joint vector describing starting configuration of
%                   arm
%        xGoal ->  1x6 joint vector describing desired position of end effector
%        sphereCenter -> 3xn position of center of spherical obstacle
%        sphereRadius -> 1x n for radius of obstacle
% output -> result -> 2x2 shows the number of visited points for h(n)=0 and
%         h(n) = norm(qEnd - qCurrent

function result = countVisited(rob,sphereCenter,sphereRadius,qStart,qEnd)


result = [];
amount = 48;
sample = createSample(rob,qStart,qEnd,sphereCenter,sphereRadius, amount);
amount = amount +2;
for m= 1:2
    qCurrent = qStart;
    count = 1;
    cost = zeros(amount,amount)-1;
    path = zeros(1,amount);
    
    for k = 1 : amount
        collision = checkCollision(rob,qStart,sample(k,:),sphereCenter,sphereRadius);
        if(collision == 1)
            cost(:,k)=100;
        else
            cost(:,k)=calculateCost(sample(k,:),qStart,qEnd,m);
            path(k)=1;
        end
    end
    cost(:,1)=99;
    [val, index]=min(cost(1,:));
    qCurrent=sample(index,:);
    currentCost = val;
    while(index~=2)
         
        count=count+1;
        if(count==(amount+1))
            break;
        end
        
        for k = 1 : amount
            if(cost(count,k)~=99)
                collision = checkCollision(rob,qCurrent,sample(k,:),sphereCenter,sphereRadius);
                if(collision == 1)
                    temp=100;
                else
                    temp=currentCost+calculateCost(sample(k,:),qCurrent,qEnd,m);
                    
                end
                
                if(cost(count,k)>temp)
                    cost(count:end,k)=temp;
                    path(k)=index;
                    %      [count,k,path(k),length(find(path==0)),index]
                end
                
            end
        end
        cost(:,index)=99;
        [val, index]=min(cost(count,:));
        
        if(val == 99)
            count =(amount+1);
            break;
        end
        qCurrent=sample(index,:);
        currentCost =val;
        
    end
    
    result = [result count];
end


end

function cost = calculateCost(q,qStart,qEnd,num)
if (num==1)
    cost = norm(qStart - q);
else if (num == 2)
        cost = norm(qStart - q)+norm(qEnd - q);
    end
end
end




    function sample = createSample(rob,qStart,qEnd,sphereCenter,sphereRadius,amount)
        
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
