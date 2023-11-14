% System data for single-machine with a motor start-up 
% used for Example 11.6
% d1m1mot_st.m.m

% bus data format
% bus: number, voltage(pu), angle(degree), p_gen(pu), q_gen(pu),
%      p_load(pu), q_load(pu),G shunt,B shunt, bus_type
%      bus_type - 1, swing bus
%               - 2, generator bus (PV bus)
%               - 3, load bus (PQ bus)
%     # V       angle  Pg     Qg    PL    QL    G     B    type
bus = [ ...
      1 1.05    0.00   0.00   0.00  0.00  0.00  0.00  0.00 1 5 -5 1 1.5 0.5;
	  2 1.00    0.00   0.00   0.00  0.00  0.00  0.00  0.00 3 5 -5 1 1.5 0.5;
      3 1.00    0.00   0.00   0.00  5.00  0.00  0.00  0.00 3 5 -5 1 1.5 0.5;
      4 1.00    0.00   0.00   0.00  0.00  0.00  0.00  0.00 3 5 -5 1 1.5 0.5];
  
load_con = [ 3  0 0 1 0 ]; 

% line data format
% line: from bus, to bus, resistance(pu), reactance(pu),
%       line charging(pu), tap ratio
 %From  To  R     X      B      tap    tap angle
line = [ ...
    1    2   0.0   0.025  0.0    1.     0. ;
 	2    3   99.0   99.99   0.0    1.     0. ;
	2    3   0.0   0.02   0.0    1.     0. ;
	3    4   0.0   0.15   0.0    1.     0. ];



% Machine data format
% Machine data format
%       1. machine number,
%       2. bus number,
%       3. base mva,
%       4. leakage reactance x_l(pu),
%       5. resistance r_a(pu),
%       6. d-axis sychronous reactance x_d(pu),
%       7. d-axis transient reactance x'_d(pu),
%       8. d-axis subtransient reactance x"_d(pu),
%       9. d-axis open-circuit time constant T'_do(sec),
%      10. d-axis open-circuit subtransient time constant
%                T"_do(sec),
%      11. q-axis sychronous reactance x_q(pu),
%      12. q-axis transient reactance x'_q(pu),
%      13. q-axis subtransient reactance x"_q(pu),
%      14. q-axis open-circuit time constant T'_qo(sec),
%      15. q-axis open circuit subtransient time constant
%                T"_qo(sec),
%      16. inertia constant H(sec),
%      17. damping coefficient d_o(pu),
%      18. dampling coefficient d_1(pu),
%      19. bus number
%      20. S(1.0) - saturation factor
%      21. S(1.2) - saturation factor
%
% note: all the following machines use electro-mechanical model
mac_con = [1 1 100000 0.00 0  0.     0.01  0   0    0     ...
                   0      0     0   0    0     ...
                   999.0    2.0   0   0    0     0];
           

           
 %ibus_con = [1 1 1 0]; 

% induction motor data
% 1. Motor Number
% 2. Bus Number
% 3. Motor MVA Base
% 4. rs pu
% 5. xs pu - stator leakage reactance
% 6. Xm pu - magnetizing reactance
% 7. rr pu 
% 8. xr pu - rotor leakage reactance
% 9. H  s  - motor plus load inertia constant
% 15. fraction of bus power drawn by motor ( if zero motor statrts at t=0)
% M# B# MVA Rs    Xs   Xm    Rr    Xr     H            on
ind_con = [ ...
  1  4  15. .000 .0759 3.124 .0085 .0759  .4 0 0 0 0 0 0
% 2  9  25. .001 .01 3 .009 .01  .7 0 0 0 0 0 .15
];
% Motor Load Data 
% format for motor load data - mld_con
% 1 motor number
% 2 bus number
% 3 stiction load pu on motor base (f1)
% 4 stiction load coefficient (i1)
% 5 external load  pu on motor base(f2)
% 6 external load coefficient (i2)
% 
% load has the form
% tload = f1*slip^i1 + f2*(1-slip)^i2
mld_con = [ ...
1  4  .1  1  .5  2
%2  9  .1  1  .7  2
];

% exciter data dc12 model
%     1 - exciter type (1 for DC1, 2 for DC2)
%     2 - machine number
%     3 - input filter time constant T_R
%     4 - voltage regulator gain K_A
%     5 - voltage regulator time constant T_A
%     6 - voltage regulator time constant T_B
%     7 - voltage regulator time constant T_C
%     8 - maximum voltage regulator output V_Rmax
%     9 - minimum voltage regulator output V_Rmin
%    10 - exciter constant K_E
%    11 - exciter time constant T_E
%    12 - E_1
%    13 - saturation function S_E(E_1)
%    14 - E_2
%    15 - saturation function S_E(E_2)
%    16 - stabilizer gain K_F
%    17 - stabilizer time constant T_F

%   TR  KA  TA      Vmax  Vmin
%   KE  TE   E1  SE1  E2  SE2  KF   TF
% exc_con= [...
% 1 1 0   40. 0.1 0 0 999.0 -999.0  ...
%     0   0.05 2.2 0.1  3.2 0.4  0.05 1.0 ] ;

% exc_con= [...
% 1 1 0   40. 0.1 0 0 999.0 -999.0  ...
%     0   0.05 2.2 0.1  3.2 0.4  0.05 1.0 ] ;

% governor data
% 1. governor type = 1
% 2. machine number
% 3. speed set point , normally equal to 1
% 4. gain ( inverse of droop)
% 5. max power pu on gen base
% 6. servo time const. s
% 7. governor or HP time const s
% 8. transient gain time const s
% 9. T4 set to make T4/T5 = frac of HP
%10. T5 reheater time const

% %                 gain      servo HP
% tg_con=[...
%         1 1  1.0  25.0 1.0  0.5   0.4  0.0  0.3*7.5  7.5];
    
%     %                 gain      servo HP
% tg_con=[...
%         1 1  1.0  25.0 1.0  0.5   0.4  0.0  0  0];


    %Switching file defines the simulation control
% row 1 col1  simulation start time (s) (cols 2 to 6 zeros)
%       col7  initial time step (s)
% row 2 col1  fault application time (s)
%       col2  bus number at which fault is applied
%       col3  bus number defining far end of faulted line
%       col4  zero sequence impedance in pu on system base
%       col5  negative sequence impedance in pu on system base
%       col6  type of fault  - 0 three phase
%                            - 1 line to ground
%                            - 2 line-to-line to ground
%                            - 3 line-to-line
%                            - 4 loss of line with no fault
%                            - 5 loss of load at bus
%       col7  time step for fault period (s)
% row 3 col1  near end fault clearing time (s) (cols 2 to 6 zeros)
%       col7  time step for second part of fault (s)
% row 4 col1  far end fault clearing time (s) (cols 2 to 6 zeros)
%       col7  time step for fault cleared simulation (s)
% row 5 col1  time to change step length (s)
%       col7  time step (s)
%
%
%
% row n col1 finishing time (s)  (n indicates that intermediate rows may be inserted)

sw_con = [...
0     0    0    0    0    0    0.001;%sets intitial time step
5.0   2    3    0    0    4    0.001; %trip line 2-3
7.0   0    0    0    0    0    0.001; % increase time step 
10.0  0    0    0    0    0    0]; % end simulation


