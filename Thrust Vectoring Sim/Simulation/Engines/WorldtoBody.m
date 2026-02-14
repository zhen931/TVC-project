function [x] = WorldtoBody(q, x)
%%%%%% Transforms world quaternion to body axes

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
x1 = x(1);
x2 = x(2); 
x3 = x(3);

x = [
    (q1^2+q2^2-q3^2-q4^2)*x1 + 2*(q2*q3+q1*q4)*x2 + 2*(q2*q4-q1*q3)*x3;
    2*(q2*q3-q1*q4)*x1 + (q1^2-q2^2+q3^2-q4^2)*x2 + 2*(q3*q4+q1*q2)*x3;
    2*(q2*q4+q1*q3)*x1 + 2*(q3*q4-q1*q2)*x2 + (q1^2-q2^2-q3^2+q4^2)*x3
];
end