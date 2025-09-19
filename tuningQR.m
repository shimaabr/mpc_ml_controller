R = [.01,.05,.1,.15,.18,.2,.25,.5,1,10]
CostR = [7.85,7.996,8.19,8.4245,8.592,8.73,9.183,11.53,14.64,20.21]
ISER = [78.1644,78.3600,78.87,80.0,81.32,82.499,86.615,109.83,142.411,201.571]
ISUR=[3.371,3.199,3.031,2.788,2.55,2.401,2.087,1.098,.3998,.00523]


% cost with changing R
subplot(3,2,1);
plot(R,CostR, '-o');
xlim([0 1]);
xlabel('R'); ylabel('Cost'); grid on; title('Cost(0-1) ');

subplot(3,2,2);
plot(R, CostR, '-o');
xlabel('R'); ylabel('Cost'); grid on; title('Cost (all)');

%ISE with changing R
subplot(3,2,3);
plot(R, ISER, '-o');
xlim([0 1]);
xlabel('R'); ylabel('ISE'); grid on; title('ISE(0-1) ');

subplot(3,2,4);
plot(R, ISER, '-o');
xlabel('R'); ylabel('ISE'); grid on; title('ISE (all)');

%ISU with changing R
subplot(3,2,5);
plot(R, ISUR, '-o');
xlim([0 1]);
xlabel('R'); ylabel('ISU'); grid on; title('ISU(0-1) ');

subplot(3,2,6);
plot(R, ISUR, '-o');
xlabel('R'); ylabel('ISU'); grid on; title('ISU (all)');

%% data for Q 
Q     = [0.01 0.025 0.05 0.075 0.1 0.2 0.5 1 10 100];
CostQ = [1.464 2.669 4.365 6.254 8.19 15.99 39.56 78.5 781.2 7809];
ISEQ  = [142.4 101.092 82.5 79.538 78.87 78.3596 78.46 78.16 78.08161 78.08209];
ISUQ  = [0.3998 1.417 2.401 2.89 3.031 3.199 3.317 3.371 3.419 3.421];

figure;

% Cost (0-1)
subplot(3,2,1);
plot(Q, CostQ, '-o'); xlim([0 1]);
xlabel('Q'); ylabel('Cost'); grid on; title('Cost (0-1)');

% Cost (all)
subplot(3,2,2);
plot(Q, CostQ, '-o');
xlabel('Q'); ylabel('Cost'); grid on; title('Cost (all)');

% ISE (0-1)
subplot(3,2,3);
plot(Q, ISEQ, '-o'); xlim([0 1]);
xlabel('Q'); ylabel('ISE'); grid on; title('ISE (0-1)');

% ISE (all)
subplot(3,2,4);
plot(Q, ISEQ, '-o');
xlabel('Q'); ylabel('ISE'); grid on; title('ISE (all)');

% ISU (0-1)
subplot(3,2,5);
plot(Q, ISUQ, '-o'); xlim([0 1]);
xlabel('Q'); ylabel('ISU'); grid on; title('ISU (0-1)');

% ISU (all)
subplot(3,2,6);
plot(Q, ISUQ, '-o');
xlabel('Q'); ylabel('ISU'); grid on; title('ISU (all)');

