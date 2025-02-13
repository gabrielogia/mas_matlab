classdef data
    properties
        global_coordinates = []
        free_dofs = []
        restrained_dofs = []
        nodes = []
        elements = []
        global_stiffness_matrix = []
        global_forces_vector = []
        global_displacements_vector = [];
        reactions_vector = []
        filename = ""
        model = ""
    end
    
    methods
        function obj = read_structure_data(obj,filename)

            % INPUT: class object
            %        filename (str)
            % OUTPUT: class object

            % Reads the input file and stores structure configuration

            obj.filename = filename;
            fid = fopen(obj.filename, 'r');
            
            if fid == -1
                error('Failed to open file.');
            end
            
            while ~feof(fid)
                line = strtrim(fgetl(fid));
                
                if startsWith(line, '#MODEL')
                    model_type = strtrim(fgetl(fid));
                    obj.model = model_type;
                    marker = "";
                end
                
                if (startsWith(line, '#NODES'))
                    marker = "NODES";
                    k = 1;
                    continue
                end
                
                if (startsWith(line, '#ELEMENTS'))
                    marker = "ELEMENTS";
                    k = 1;
                    continue
                end
            
                if (marker == "NODES")
                    temp = str2num(strtrim(line));
                    if isempty(temp)
                        continue
                    else
                        node_obj = node(temp(1), temp(2), temp(3), temp(4), temp(5), temp(6), temp(7), temp(8), temp(9), temp(10), temp(11), temp(12), model_type);
                        obj.nodes = [obj.nodes, node_obj];
                        k = k + 1;
                    end
                end
            
                if (marker == "ELEMENTS")
                    temp = str2num(strtrim(line));
                    if isempty(temp)
                        continue
                    else
                        if (obj.model == "truss")
                            element_obj = truss_element(temp(1), temp(2), temp(3), temp(4), temp(5), temp(6), temp(7), temp(8), temp(9));
                        else
                            continue %(TODO)
                        end
                        obj.elements = [obj.elements, element_obj];
                        k = k + 1; 
                    end
                end
            end
            
            fclose(fid);
        end
        
        function obj = set_global_coordinates(obj)

            % INPUT: class object
            % OUTPUT: class object

            % Stores the configuration of degrees of freedom, either free
            % or constrained (restrained), into global coordinates vector

            dofs_free = 0;
            dofs_restrained = 0;

            if (obj.model == "truss")
                obj.global_coordinates = zeros(1,numel(obj.nodes)*2);
            else
                % (TODO)
            end

            for i = 1:numel(obj.nodes)
                [f,r] = obj.nodes(i).count_degrees();
                dofs_free = dofs_free + f;
                dofs_restrained = dofs_restrained + r;
            end

            k = 1;
            for i = 1:numel(obj.nodes)
                [obj.nodes(i), k] = obj.nodes(i).set_free_global_coordinates(k);
            end

            k = 1 + dofs_free;
            for i = 1:numel(obj.nodes)
                [obj.nodes(i), k] = obj.nodes(i).set_restrained_global_coordinates(k);
            end

            for i = 1:numel(obj.nodes)
                obj.global_coordinates(2*i - 1) = obj.nodes(i).global_coordinates(1);
                obj.global_coordinates(2*i) = obj.nodes(i).global_coordinates(2);
            end

            obj.free_dofs = dofs_free;
            obj.restrained_dofs = dofs_restrained;
        end

        function obj = set_local_variables_bar(obj)
            
            % INPUT: class object
            % OUTPUT: class object

            % Stores elements configuration and properties, such as: 
            % L: length; cosine: value of cos(x); sine: value of sin(x);
            % connectivity_vector: which nodes are connected in the element; 
            % rotation_matrix: matrix with the rotation transformation; 
            % local_stiffness_local_coord: stiffness matrix in local coordinates; 
            % local_stiffness_global_coord: stiffness matrix in global coordinates.

            for i = 1:numel(obj.elements)
                obj.elements(i).L = sqrt((obj.nodes(obj.elements(i).end_node).x - obj.nodes(obj.elements(i).init_node).x)^2 ...
                                          + (obj.nodes(obj.elements(i).end_node).y - obj.nodes(obj.elements(i).init_node).y)^2);

                obj.elements(i).cosine = (obj.nodes(obj.elements(i).end_node).x - obj.nodes(obj.elements(i).init_node).x)/obj.elements(i).L;

                obj.elements(i).sine = (obj.nodes(obj.elements(i).end_node).y - obj.nodes(obj.elements(i).init_node).y)/obj.elements(i).L;

                obj.elements(i).connectivity_vector = obj.elements(i).set_connectivity_vector(obj.nodes(obj.elements(i).init_node), ...
                                                                                              obj.nodes(obj.elements(i).end_node)).connectivity_vector;
                obj.elements(i).rotation_matrix = obj.elements(i).set_rotation_matrix().rotation_matrix;

                obj.elements(i).local_stiffness_local_coord = obj.elements(i).set_element_stiffness_matrix_local_coordinates().local_stiffness_local_coord;

                obj.elements(i).local_stiffness_global_coord = obj.elements(i).rotation_matrix'*obj.elements(i).local_stiffness_local_coord*obj.elements(i).rotation_matrix;
            end
        end

        function obj = set_global_stiffness_matrix_free_dofs(obj)

            % INPUT: class object
            % OUTPUT: class object

            % Assembles the global stiffness matrix for unconstrained
            % degrees of freedom. 

            obj.global_stiffness_matrix = zeros(obj.free_dofs, obj.free_dofs);

            for e = 1:numel(obj.elements)
                for i = 1:numel(obj.elements(e).connectivity_vector)
                    for j = 1:numel(obj.elements(e).connectivity_vector)
                        if ((obj.elements(e).connectivity_vector(i) <= obj.free_dofs) && (obj.elements(e).connectivity_vector(j) <= obj.free_dofs))
                            obj.global_stiffness_matrix(obj.elements(e).connectivity_vector(i), obj.elements(e).connectivity_vector(j)) = ...
                                obj.global_stiffness_matrix(obj.elements(e).connectivity_vector(i), obj.elements(e).connectivity_vector(j)) ...
                                + obj.elements(e).local_stiffness_global_coord(i,j);
                        end
                    end
                end
            end
        end

        function obj = set_global_force_vector(obj)

            % INPUT: class object
            % OUTPUT: class object

            % Assembles the global force vector for unconstrained degrees
            % of freedom.

            obj.global_forces_vector = zeros(1,obj.free_dofs)';

            for i = 1:numel(obj.nodes)
                if (obj.nodes(i).global_coordinates(1) <= obj.free_dofs)
                    obj.global_forces_vector(obj.nodes(i).global_coordinates(1)) = obj.nodes(i).f_x;
                end

                if (obj.nodes(i).global_coordinates(2) <= obj.free_dofs)
                    obj.global_forces_vector(obj.nodes(i).global_coordinates(2)) = obj.nodes(i).f_y;
                end
            end
        end
    end
end

