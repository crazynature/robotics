% TODO: You write this function!
% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First six joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)
function q = Q3(f1,f2,qInit,f1Target,f2Target)

p1=transl(f1Target);
p2=transl(f2Target);
q1=ikine(f1,p1,qInit(1:9));
q2=ikine(f2,p2,[qInit(1:7) qInit(10:11)]);
q=[q1 q2(8:9)];
end

    
