
% 
% Evaluate whether the configuration <q> is in collision with a spherical
% obstacle centered at <sphereCenter> with radius <r>.
% 
% input: q -> 1x6 vector of joint angles
%        sphereCenter -> 6xn vector that denotes mutli sphere center
%        r -> 1xn vertor that denotes multi radius of sphere 
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.

function collision = mutiObstacles(rob,q,sphereCenter,r)
collision=0;
  for i = 1:size(sphereCenter,2)

       if(robotCollision(rob,q,sphereCenter(1:end,i),r(i))==1)
           collision = 1;
           break;
       end
  end
end