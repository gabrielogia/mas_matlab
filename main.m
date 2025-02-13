%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Developed by: João Gabriel Duarte %
%       E-mail: dacos033@umn.edu    %
%   Updated at: 02/13/2025          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear
close all

% define the input filename
filename = "data/input/truss_ca1.txt";

% simulation data object creation
sim_data = data;

% read the structure data from input file
sim_data = sim_data.read_structure_data(filename);

% set global coordinate system
sim_data = sim_data.set_global_coordinates();

% set truss members variables, such as: length, area, rotation and 
% stiffness matrices, etc
sim_data = sim_data.set_local_variables_bar();

% set the global stiffness matrix of the unconstrained degrees of freedom
sim_data = sim_data.set_global_stiffness_matrix_free_dofs();

% set the global force vector of the unconstrained degrees of freedom
sim_data = sim_data.set_global_force_vector();

% create solver object from simulation data
solver = static_solver(sim_data);

% solve the linear system F = KX
sim_data = solver.solve().data;

% print the results and plot the deformed configuration
writer = write_output(sim_data.model);
writer.print_results(sim_data);