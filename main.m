clc
clear
close all

filename = "data/input/truss_ca1.txt";

sim_data = data;
sim_data = sim_data.read_structure_data(filename);
sim_data = sim_data.set_global_coordinates();
sim_data = sim_data.set_local_variables_bar();
sim_data = sim_data.set_global_stiffness_matrix_free_dofs();
sim_data = sim_data.set_global_force_vector();

solver = static_solver(sim_data);
sim_data = solver.solve().data;

writer = write_output(sim_data.model);
writer.print_results(sim_data);