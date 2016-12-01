syms d1 d4 d5 d6 a2 a3 q1 q2 q3 q4 q5 q6
a = [0, a2, a3, 0, 0, 0]
alpha = [pi / 2, 0, 0, pi/2, -pi/ 2, 0];
d = [d1 0 0 d4 d5 d6]
theta = [q1 q2 q3 q4 q5 q6]

T = eye(4)
i = 1;
T1 = [    cos(theta(i)),    0, sin(theta(i)) ,  0,
		  sin(theta(i)),    0, -cos(theta(i)),  0,
		  0,                1, 0,               d(i);
		  0,                0, 0,               1]
i = 2;
T2 = [    cos(theta(i)),    -sin(theta(i)), 0,  a(i) * cos(theta(i)),
		  sin(theta(i)),    cos(theta(i)) , 0,  a(i) * sin(theta(i)),
		  0,                0,              1,                 0;
		  0,                0,              0,                 1]
i = 3;
T3 = [    cos(theta(i)),    -sin(theta(i)), 0,  a(i) * cos(theta(i)),
		  sin(theta(i)),    cos(theta(i)) , 0,  a(i) * sin(theta(i)),
		  0,                0,              1,                 0;
		  0,                0,              0,                 1]
i = 4;	
T4 = [    cos(theta(i)),    0, sin(theta(i)) ,  0,
		  sin(theta(i)),    0, -cos(theta(i)),  0,
		  0,                1, 0,               d(i);
		  0,                0, 0,               1];

i = 5;
T5 = [    cos(theta(i)),    0,  -sin(theta(i)) ,  0,
		  sin(theta(i)),    0,  cos(theta(i)),    0,
		  0,                -1, 0,                d(i);
		  0,                0,  0,                1]

i = 6;
T6 = [    cos(theta(i)),    -sin(theta(i)),  0, 0,
		  sin(theta(i)),    cos(theta(i)) ,  0, 0,
		  0,                0,               1, d(i);
		  0,                0,               0, 1]

% T = T1 * T2 * T3 * T4 * T5 * T6
T = T2 * T3 * T4 * T5 * T6

T14 = T2 * T3 * T4

tt = T14 * [0; 0; -d4; 1]

%T13 = T2 * T3
return;

cos(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))), 
- sin(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))), 
cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))), 
d5*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + d4*sin(q1) + d6*(cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3)
- cos(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))),   
sin(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))), 
sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*cos(q5), 
d5*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - d4*cos(q1) - d6*(cos(q1)*cos(q5) - sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) + a2*cos(q2)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3)
sin(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + cos(q5)*cos(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),
cos(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - cos(q5)*sin(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                  
-sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),
d1 + a2*sin(q2) - d5*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + a3*cos(q2)*sin(q3) + a3*cos(q3)*sin(q2) - d6*sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))
0,                                                                                                                                                                                                                                                                                                       
0,                                                                                                                                                     
0,                                                                                                                                                                                                                                                                                                                                                                                       
1


 % T14 
[ cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)), 0, cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)), a2*cos(q2) + a3*cos(q2)*cos(q3) - a3*sin(q2)*sin(q3)]
[ cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)), 0, sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)), a2*sin(q2) + a3*cos(q2)*sin(q3) + a3*cos(q3)*sin(q2)]
[                                                                                         0, 1,                                                                                         0,                                                   d4]
[                                                                                         0, 0,                                                                                         0,                                                    1]
 