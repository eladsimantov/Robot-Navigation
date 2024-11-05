% A star algorithm for Q3
clearvars; clc; close all;
% Set of nodes
Xs = [1 10; 7 10; 0 7; 3 8; 8 8; 5 7; 10 7; 2 4; 6 3; 3 1; 9 2; 10 5];
target = repmat(Xs(end,:), [12, 1]); % target node
Ts = sqrt(sum((Xs - target).^2, 2)); % Hueristic
% step 0
t6 = Ts(6);
% step 1 from 6 to ...
t4 = Ts(4) + 2.24;
t5 = Ts(5) + 3.17;
t8 = Ts(8) + 6.2;
t9 = Ts(9) + 4.13;
% step 2 from 5
t2 = Ts(2) + 2.24 + 3.17;
t7 = Ts(7) + 2.24 + 3.17;
% step 3 
t12 = 2 + 2.24 + 3.17;