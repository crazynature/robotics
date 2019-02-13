% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% input: questionNum -> Integer between 1 and 4 to run.
%         1 is check if the checkCollision works
%         2 is check if the qStart and qEnd can be connected to sample
%         3 is draw the path for PRM, when sample is 50
%         4 is count the visited points for different h(n) 
function project(questionNum)

     close all;
    

    if nargin < 1
        error('Error: please enter a question number as a parameter');
%         questionNum = 2;
    end
    
    % set up robot and initial joint configuration
    mdl_puma560;
	rob = p560;
	qStart = rob.a;
	qEnd = [2.8368   -2.3679   -1.1261    3.1416    1.7453    0.3048];
    sphereCenter = [0.6;0.0;-0.2];
    sphereCenter2 = [0.1;-0.5;0];
    sphereCenter3 = [0.0;0.5;-0.5];
    sphereRadius = 0.2;
    sphereRadius2 = 0.3;
    % plot robot and sphere
    
    rob.plot(qStart);
    hold on;	
    drawSphere(sphereCenter,sphereRadius);
    drawSphere(sphereCenter2,sphereRadius2);
    drawSphere(sphereCenter3,sphereRadius2);
     sphereCenter=[sphereCenter sphereCenter2 sphereCenter3];
     sphereRadius=[sphereRadius sphereRadius2 sphereRadius2];
  %   sphereCenter=[sphereCenter];
  %   sphereRadius=[sphereRadius];
    if questionNum == 1

        collision = checkCollision(rob,qStart,[-0.9391, -2.5286, 1.4566, -0.3478 0 0],sphereCenter,sphereRadius);
        display(['this should be 1: ',int2str(collision)])
        collision = checkCollision(rob,qStart,[0 -0.78 0 -1.5 0 0],sphereCenter,sphereRadius);
        display(['this should be 1: ',int2str(collision)])        
        collision = checkCollision(rob,qStart,[0 -0.78 0 -0.1 0 0],sphereCenter,sphereRadius);
        display(['this should be 1: ',int2str(collision)])     
    elseif questionNum == 2
       result = checkConnected(rob,sphereCenter,sphereRadius,qStart,qEnd);
        
        % interpolate and plot direct traj from start to goal
    
        
     elseif questionNum == 3   
       qMilestones = drawPath(rob,sphereCenter,sphereRadius,qStart,qEnd);
        
        % interpolate and plot direct traj from start to goal
        
        qTraj = interpMilestones(qMilestones);
        rob.plot(qTraj);
        
    elseif questionNum == 4
        for x = 1:10
        result = countVisited(rob,sphereCenter,sphereRadius,qStart,qEnd)
        end
    else
        error('Error: question number out of range.');        
    end
    

end

function traj = interpMilestones(qMilestones)

    d = 0.05;
%     traj = qMilestones(1,:);
    traj = [];
    for i=2:size(qMilestones,1)
        
        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];
        
    end
end

function qPath = getPath(tree)

    m = 10;
    idx = size(tree,1);
    path = tree(end,1:end-1);
    
    while(idx ~= 1)
        
        curr = tree(idx,1:end-1);
        idx = tree(idx,end);
        next = tree(idx,1:end-1);
        path = [path;[linspace(curr(1),next(1),m)' linspace(curr(2),next(2),m)' linspace(curr(3),next(3),m)' linspace(curr(4),next(4),m)']];
        
    end
    qPath = path(end:-1:1,:);
    
end


function rob = createRobot()

    L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0 0 0.4318 1.571]);
%     L(5) = Link([0 0.4318 0 1.571]);
    
    rob = SerialLink(L, 'name', 'robot');

end

function drawSphere(position,diameter)

%     diameter = 0.1;
    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);
    %~ shading flat

end


