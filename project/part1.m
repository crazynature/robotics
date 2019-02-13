% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        sphereCenter -> 3xN position of center of sphere
%        r -> 1xN radius of sphere
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = part1(rob,q1,q2,sphereCenter,r)

qdiffer = q2 - q1;
collision = 0;

  for i = 1:size(sphereCenter,2)
    q = q1;
    for k = 0:10
      q = q1 + ( 0.1 * qdiffer)*k;
       if(robotCollision(rob,q,sphereCenter(1:end,i),r(i))==1)
           collision = 1;
       end
    end 
end


end
