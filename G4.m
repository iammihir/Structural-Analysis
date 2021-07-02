%Structural Analysis - Group 4
%To Check the Output directly, please type the following in the Command Window
%Run to check the plot and compile the code
%K = Global Stiffness Matrix
%N = Member Forces
%U = Displacements
%R = Support Reaction Forces
clear global;
clc;

%---INPUT---
NODES = [0 0; 2 -2*sqrt(3); 0 -8/sqrt(3); -2 -2*sqrt(3); -4 -4*sqrt(3); 4 -4*sqrt(3)];
CONN = [1 2; 1 3; 1 4; 2 3; 4 3; 2 6; 4 5; 3 6; 3 5];
NUM = [1 2; 3 4; 5 6; 7 8; 9 10; 11 12];
NR = 4;
NE = size(CONN,1);
NN = size(NODES,1);
EA = [8000/3 8000 8000/3 8000/3 8000/3 8000/3 8000/3 8000 8000]*10^5;
FEX = [100 -40 200 0 0 0 0 0]';
Ur = [0 0 0 0]';
scale = 50000;
%NODES - Coordinates of Nodes
%CONN - Connenctions between the nodes
%NUM - Numbering of x and y DoF Forces on each node
%NR - Number of Support Reaction Forces
%NE - Number of Elements(Truss Rods)
%NN - Number of Nodes
%EA - E*A value in SI Units for each Member
%FEX - External Forces on the Nodes (excluding the support reactions)
%scale - For Plotting the Structure


%---CALCULATION---

%Truss Information
NOS = NE+NR-2*NN;  %Static Indeterminacy
NOK = 2*NN-NR;     %Kinematic Indeterminacy

%Length of Element
L = zeros(NE,1);
for k = 1:NE
    i = CONN(k,1);
    j = CONN(k,2);
    dx = NODES(j,1) - NODES(i,1);
    dy = NODES(j,2) - NODES(i,2);
    L(k) = sqrt(dx^2+dy^2);
end
%L(k) - Length of each member

%ID Array
ID = zeros(NE,4);
for k = 1:NE
    i = CONN(k,1);
    j = CONN(k,2);
    ID(k,1:2) = NUM(i,1:2);
    ID(k,3:4) = NUM(j,1:2);
end
%ID - Defining the numbering for each Elemental Stiffness Matrix

%Stiffness Matrix
NDOF = 2*NN;
K = zeros(NDOF,NDOF);
for k = 1:NE
    i = CONN(k,1);
    j = CONN(k,2);
    dx = NODES(j,1) - NODES(i,1);
    dy = NODES(j,2) - NODES(i,2);
    L(k) = sqrt(dx^2+dy^2);
    a = [-dx/L(k) -dy/L(k) dx/L(k) dy/L(k)];
    ES = a'.*EA(k)/L(k)*a;
    for m =1:4
        for n = 1:4
            mi = ID(k,m);
            ni = ID(k,n);
            K(mi,ni) = K(mi,ni) + ES(m,n);
        end
    end
end
Kff(1:NOK,1:NOK) = K(1:NOK,1:NOK);
Kfr(1:NOK,1:NDOF-NOK) = K(1:NOK,NOK+1:NDOF);
Krf = Kfr';
Krr(1:NDOF-NOK,1:NDOF-NOK) = K(NOK+1:NDOF,NOK+1:NDOF);
%K - Global Stiffness Matrix
%Kff - Global Stiffness Matrix (excluding support reactions)
%NDOF - Length/Breadth of Global Stiffness Matrix

%Displacement
Uf = Kff\FEX;
U = [Uf; Ur];
%U - Displacement Matrix

%Internal Forces
N = zeros(NE,1);
for k =1:NE
    i = CONN(k,1);
    j = CONN(k,2);
    dx = NODES(j,1) - NODES(i,1);
    dy = NODES(j,2) - NODES(i,2);
    L(k) = sqrt(dx^2+dy^2);
    a = [-dx/L(k) -dy/L(k) dx/L(k) dy/L(k)];
    u = zeros(4,1);
    for m = 1:4
        u(m) = U(ID(k,m));
    end
    N(k) = EA(k)/L(k).*a*u;
end
%N - Member Force Matrix

%Reaction
R = Krf*Uf + Krr*Ur;
%R - Support Reaction Matrix

%---PLOTTING THE TRUSS---%
f1 = figure();
f1.WindowState = 'maximized';
NNODES = zeros(size(NODES));

for n = 1:NN
    NNODES(n,1) = NODES(n,1) + scale*U(NUM(n,1));
    NNODES(n,2) = NODES(n,2) + scale*U(NUM(n,2));
end
for k = 1:NE
    i = CONN(k,1);
    j = CONN(k,2);
    x = [NODES(i,1) NODES(j,1)];
    y = [NODES(i,2) NODES(j,2)];
    plot(x,y, 'Color', uint8([134 136 138]), 'LineWidth', 2);
    hold on;
    ux = [NNODES(i,1) NNODES(j,1)];
    uy = [NNODES(i,2) NNODES(j,2)];
    plot(ux,uy, 'b--', 'LineWidth', 1.5);
end
xlim([0-5, max(NODES(:,1))+5]);
ylim([0-5, max(NODES(:,2))+5]);
axis equal;
%To plot the Graph
%Grey Solid - Original Structure
%Blue Dashed - Deformed Structure due to External Forces 


%-----------------------------------------------------------------------------
%To Check the Output directly, please type the following in the Command window
%Run to check the plot and compile the code
%K = Global Stiffness Matrix
%N = Member Forces
%U = Displacements
%R = Support Reaction Forces
%-----------------------------------------------------------------------------



































