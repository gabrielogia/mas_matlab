classdef write_output
    properties
        model = ""
    end
    
    methods
        function obj = write_output(model)
            obj.model = model;
        end
        
        function print_results(obj,data)
            obj.print_forces(data);
            obj.print_displacements(data);
            obj.print_stress(data);
            obj.plot_results(data);
        end

        function print_forces(~,data)
            disp("--- CONVENTION ---")
            disp("↑ (positive) and → (positive)")
            disp("--- NODAL FORCES ---")

            for i = 1:numel(data.nodes)
                pos = data.global_coordinates(ismember(data.global_coordinates, data.nodes(i).global_coordinates));
                aux = sprintf("NODE %d :::", i);

                for j = 1:numel(pos)
                    if (pos(j) > data.free_dofs)
                        aux = strcat(aux, sprintf(" Global coordinate: %d | Reaction: %.2f psi | ", pos(j), data.reactions_vector(pos(j) - data.free_dofs)));
                    else
                        aux = strcat(aux, sprintf(" Global coordinate: %d | External force: %.2f psi | ", pos(j), data.global_forces_vector(pos(j))));
                    end
                end

                disp(aux);
            end
        end

        function print_displacements(~,data)
            disp("--- NODAL DISPLACEMENTS ---")

            for i = 1:numel(data.nodes)
                pos = data.global_coordinates(ismember(data.global_coordinates, data.nodes(i).global_coordinates));
                aux = sprintf("NODE %d :::", i);

                for j = 1:numel(pos)
                    if (pos(j) > data.free_dofs)
                        aux = strcat(aux, sprintf(" Global coordinate: %d | Displacement: 0.0000 ft | ", pos(j)));
                    else
                        aux = strcat(aux, sprintf(" Global coordinate: %d | Displacement: %.4f ft | ", pos(j), data.global_displacements_vector(pos(j))));
                    end
                end

                disp(aux);
            end
        end

        function print_stress(~,data)
            disp("--- AXIAL FORCES ---")

            for i = 1:numel(data.elements)
                aux = sprintf("ELEMENT %d :::", i);
                aux = strcat(aux, sprintf(" Axial force: %.2f psi ", data.elements(i).local_forces_local_coord(3)));
                disp(aux)
            end
        end

        function plot_results(obj,data)
            obj.plot_original_structure(data)
            obj.plot_deformed_structure(data)

            figure(1)
            title("Undeformed (blue) vs. deformed (red) configurations")
            k=8;
            xlim([0-max([data.nodes(:).x]/k) max([data.nodes(:).x]+max([data.nodes(:).x])/k)])
            ylim([0-max([data.nodes(:).y]/k) max([data.nodes(:).y]+max([data.nodes(:).y])/k)])
        end

        function plot_original_structure(~,data)
            figure(1);
            for i = 1:numel(data.nodes)
                scatter(data.nodes(i).x, data.nodes(i).y, 50, "filled", 'b')
                hold on
            end

            figure(1);
            for i = 1:numel(data.elements)
                x = [data.nodes(data.elements(i).init_node).x data.nodes(data.elements(i).end_node).x];
                y = [data.nodes(data.elements(i).init_node).y data.nodes(data.elements(i).end_node).y];
                plot(x, y, 'b');
                hold on
            end
        end

        function plot_deformed_structure(~,data)
            figure(1);
            for i = 1:numel(data.nodes)
                pos = data.global_coordinates(ismember(data.global_coordinates, data.nodes(i).global_coordinates));

                for j = 1:numel(pos)
                    if (pos(j) > data.free_dofs)
                        if j == 1
                           new_x(1,i) = data.nodes(i).x;
                        else
                           new_y(1,i) = data.nodes(i).y;
                        end
                    else
                        if j == 1
                            new_x(1,i) = data.nodes(i).x + data.global_displacements_vector(pos(j));
                        else
                            new_y(1,i) = data.nodes(i).y + data.global_displacements_vector(pos(j));
                        end
                    end
                end

                scatter(new_x(i), new_y(i), 50, "filled", 'r')
                hold on
            end

            figure(1);
            for i = 1:numel(data.elements)
                x = [new_x(data.elements(i).init_node) new_x(data.elements(i).end_node)];
                y = [new_y(data.elements(i).init_node) new_y(data.elements(i).end_node)];
                plot(x, y, 'r');
                hold on
            end
        end
    end
end

