classdef static_solver
    properties
        data
    end
    
    methods
        function obj = static_solver(data)
            obj.data = data;
        end

        function obj = solve(obj)
            obj = obj.get_nodal_displacements();
            obj = obj.get_members_forces_and_displacements();
            obj = obj.get_reactions();
        end

        function obj = get_nodal_displacements(obj)
            obj.data.global_displacements_vector = obj.data.global_stiffness_matrix\obj.data.global_forces_vector;
        end

        function obj = get_members_forces_and_displacements(obj)
            obj.data.elements = obj.get_displacements_global_coordinates().data.elements;
            obj.data.elements = obj.get_diplacements_local_coordinates().data.elements;
            obj.data.elements = obj.get_forces_local_coordinates().data.elements;
            obj.data.elements = obj.get_forces_global_coordinates().data.elements;
        end

        function obj = get_displacements_global_coordinates(obj)
            for i = 1:numel(obj.data.elements)
                obj.data.elements(i).local_displacement_global_coord = zeros(1, numel(obj.data.elements(i).connectivity_vector));

                for j = 1:numel(obj.data.elements(i).connectivity_vector)
                    if (obj.data.elements(i).connectivity_vector(j) <= obj.data.free_dofs)
                        obj.data.elements(i).local_displacement_global_coord(j) = obj.data.global_displacements_vector(obj.data.elements(i).connectivity_vector(j));
                    end
                end
            end
        end

        function obj = get_diplacements_local_coordinates(obj)
            for i = 1:numel(obj.data.elements)
                obj.data.elements(i).local_displacement_local_coord = obj.data.elements(i).rotation_matrix*obj.data.elements(i).local_displacement_global_coord';
            end
        end

        function obj = get_forces_local_coordinates(obj)
            for i = 1:numel(obj.data.elements)
                obj.data.elements(i).local_forces_local_coord = obj.data.elements(i).local_stiffness_local_coord*obj.data.elements(i).local_displacement_local_coord;
            end 
        end

        function obj = get_forces_global_coordinates(obj)
            for i = 1:numel(obj.data.elements)
                obj.data.elements(i).local_forces_global_coord = obj.data.elements(i).rotation_matrix'*obj.data.elements(i).local_forces_local_coord;
            end 
        end

        function obj = get_reactions(obj)
            obj.data.reactions_vector = zeros(1,obj.data.restrained_dofs)';

            for i = 1:numel(obj.data.elements)
                for j = 1:numel(obj.data.elements(i).connectivity_vector)
                    if (obj.data.elements(i).connectivity_vector(j) > obj.data.free_dofs)
                        obj.data.reactions_vector(obj.data.elements(i).connectivity_vector(j) - obj.data.free_dofs) = ...
                            obj.data.reactions_vector(obj.data.elements(i).connectivity_vector(j) - obj.data.free_dofs) ...
                            + obj.data.elements(i).local_forces_global_coord(j);
                    end
                end
            end
        end
    end
end

