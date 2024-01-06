#const m=13.

%%%%% INITIALIZE PARAMETERS %%%%%

node(N,pair(X,Y)) :- init(object(node,N),value(at,pair(X,Y))).
highway(N) :- init(object(highway,N),value(at,pair(X,Y))).
pickingStation(PS,N) :- init(object(pickingStation,PS),value(at,pair(X,Y))), init(object(node,N),value(at,pair(X,Y))).
robot(R,N,0) :- init(object(robot,R),value(at,pair(X,Y))), init(object(node,N),value(at,pair(X,Y))).
shelf(S,N,0) :- init(object(shelf,S),value(at,pair(X,Y))), init(object(node,N),value(at,pair(X,Y))).
product(PR,S,PQ,0) :- init(object(product,PR),value(on,pair(S,PQ))).
order(O,PS,PR,OQ,0) :- init(object(order,O),value(pickingStation,PS)), init(object(order,O),value(line,pair(PR,OQ))).


% Calculate world state counts
numRows(NN):- NN=#count{X:init(object(node,I),value(at,pair(X,Y)))}.
numCols(NN):- NN=#count{Y:init(object(node,I),value(at,pair(X,Y)))}.
% numShelves(NN):- NN=#count{I:init(object(shelf,I),value(at,pair(X,Y)))}.
% numProducts(NN):- NN=#count{I:init(object(product,I),value(on,pair(X,Y)))}.
% numOrders(NN):- NN=#count{I:init(object(order,I),value(pickingStation,J))}.
numRobots(NN):- NN=#count{I:init(object(robot,I),value(at,pair(X,Y)))}.


%%%%% GENERATE ACTION SPACE %%%%%

% possible move actions (orthogonal moves only)
move(0,1; 0,-1; -1,0; 1,0).

% possible actions that each robot can take
{robotMove(R,move(DX,DY),T) : move(DX,DY)} 1 :- R=1..NR, numRobots(NR), T=0..m-1.
{pickUpShelf(R,S,T) : shelf(S,N,T)} 1 :- R=1..NR, numRobots(NR), T=0..m-1.
{putDownShelf(R,S,T) : shelf(S,N,T)} 1 :- R=1..NR, numRobots(NR), T=0..m-1.
{deliverProduct(R,O,PS,items(S,PR,DQ),T) : order(O,PS,PR,OQ,T), product(PR,S,PQ,T), DQ=1..PQ} 1 :- R=1..NR, numRobots(NR), T=0..m-1.

% actions occur
occurs(object(robot,R),move(DX,DY),T) :- robotMove(R,move(DX,DY),T).
occurs(object(robot,R),pickup,T) :- pickUpShelf(R,S,T).
occurs(object(robot,R),putdown,T) :- putDownShelf(R,S,T).
occurs(object(robot,R),deliver(O,PR,DQ),T) :- deliverProduct(R,O,PS,items(S,PR,DQ),T).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% CONSTRAINTS %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% no robot can execute 2 actions on the same timestep
:- occurs(object(robot,R),A,T), occurs(object(robot,R),A1,T), A!=A1.


%%% MOVING %%%
% robots must stay inbounds
:- robot(R,N,T), node(N,pair(X,Y)), occurs(object(robot,R),move(DX,DY),T), X+DX<1.
:- robot(R,N,T), node(N,pair(X,Y)), occurs(object(robot,R),move(DX,DY),T), Y+DY<1.
:- robot(R,N,T), node(N,pair(X,Y)), occurs(object(robot,R),move(DX,DY),T), Y+DY>NR, numRows(NR).
:- robot(R,N,T), node(N,pair(X,Y)), occurs(object(robot,R),move(DX,DY),T), X+DX>NC, numCols(NC).

% no 2 robots on 1 node
:- robot(R1,N,T), robot(R2,N,T), node(N,pair(X,Y)), R1!=R2.

% robots cannot swap positions
:- robot(R1,N1,T), robot(R1,N2,T+1), robot(R2,N2,T), robot(R2,N1,T+1), N1!=N2, R1!=R2.

% no 2 shelves on the same node
:- shelf(S1,N,T), shelf(S2,N,T), node(N,pair(X,Y)), S1!=S2.



%%% Pickup/Putdown %%%
% robot and shelf must be in same position to pick up
:- robot(R,N1,T), shelf(S,N2,T), pickUpShelf(R,S,T), N1!=N2.

% can only put shelf down if a robot is carrying a shelf
:- robot(R,N,T), putDownShelf(R,S,T), not hasShelf(R,S,T).

% cannot pickup shelf if robot is carrying a shelf
:- robot(R,N,T), pickUpShelf(R,S,T), hasShelf(R,S,T).

% cannot put shelf down on highway
:- robot(R,N,T), putDownShelf(R,S,T), highway(N).


%%% Delivery %%%
% can only deliver to CORRECT picking station
:- deliverProduct(R,O,PS,items(S,PR,DQ),T), robot(R,N1,T), pickingStation(PS,N2), N1!=N2.
:- deliverProduct(R,O,PS1,items(S,PR,DQ),T), robot(R,N,T), pickingStation(PS2,N), PS1!=PS2.

% robot must have product to deliver
:- deliverProduct(R,O,PS,items(S,PR,DQ),T), robot(R,N,T), not hasShelf(R,S,T).

% limit deliveries by product qty
:- deliverProduct(R,O,PS,items(S,PR,DQ),T), product(PR,S,PQ,T), DQ>PQ.

% limit deliveries by product order qty
:- deliverProduct(R,O,PS,items(S,PR,DQ),T), order(O,PS,PR,OQ,T), DQ>OQ.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% ACTIONS %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% move robots to new node on T+1
robot(R,N_new,T+1) :- robot(R,N_old,T), occurs(object(robot,R), move(DX,DY),T), node(N_old,pair(X,Y)), node(N_new,pair(X+DX,Y+DY)).

% pickup and putdown boxes
hasShelf(R,S,T+1) :- pickUpShelf(R,S,T), shelf(S,N,T), robot(R,N,T).

% if robot has shelf then position of shelf is the position of robot
shelf(S,N,T) :- hasShelf(R,S,T), robot(R,N,T).

% effects of delivery: fulfills order and decreases product count
product(PR,S,PQ-DQ,T+1) :- deliverProduct(R,O,PS,items(S,PR,DQ),T), product(PR,S,PQ,T).
order(O,PS,PR,OQ-DQ,T+1) :- deliverProduct(R,O,PS,items(S,PR,DQ),T), order(O,PS,PR,OQ,T).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% DOMAIN INDEPENDANT AXIOMS %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% law of uniqueness
:- not {robot(R,N,T)}=1, R=1..NR, numRobots(NR), T=1..m.
:- not {shelf(S,N,T)}=1, S=1..NN, numShelves(NN), T=1..m.

% law of inertia
{robot(R,N,T+1)} :- robot(R,N,T), T=0..m-1.
{shelf(S,N,T+1)} :- shelf(S,N,T), T=0..m-1.
hasShelf(R,S,T+1) :- hasShelf(R,S,T), not putDownShelf(R,S,T), T=0..m-1.
order(O,PS,PR,OQ,T+1) :- order(O,PS,PR,OQ,T), not deliverProduct(_,_,_,items(_,PR,_),T), T=0..m-1.
product(PR,S,PQ,T+1) :- product(PR,S,PQ,T), not deliverProduct(_,_,_,items(S,PR,_),T), T=0..m-1.


%%%%%%%%%%%%%%%%%%%%%%
%%%%% GOAL STATE %%%%%
%%%%%%%%%%%%%%%%%%%%%%

:- order(O,PS,PR,OQ,m), OQ>0.

numActions(N) :- N=#sum{1,O,A,T : occurs(O,A,T)}.
totalTime(N) :- N=#count{T : occurs(O,A,T)}.

#minimize{1,O,A,T : occurs(O,A,T)}.    % minimize Action Count
#minimize{T : occurs(O,A,T), T>0}.     % minimize Timesteps

#show occurs/3.         % show actions
#show numActions/1.     % show total number of actions
#show totalTime/1.      % show total number of timesteps