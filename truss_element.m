classdef truss_element
    properties
        id
        init_node
        end_node
        E
        A
        I
        D
        q_x
        q_y
        L
        cosine
        sine
        local_stiffness_local_coord = []
        local_stiffness_global_coord = []
        connectivity_vector = []
        rotation_matrix = []
        local_displacement_local_coord = []
        local_displacement_global_coord = []
        local_forces_local_coord = []
        local_forces_global_coord = []
    end
    
    methods
        function obj = truss_element(id, init_node, end_node, E, A, I, D, Qx, Qy)
            obj.id = id;
            obj.init_node = init_node;
            obj.end_node = end_node;
            obj.E = E;
            obj.A = A;
            obj.I = I;
            obj.D = D;
            obj.q_x = Qx;
            obj.q_y = Qy;
            obj.L = 0;
            obj.cosine = 0;
            obj.sine = 0;
        end

        function obj = set_connectivity_vector(obj, init_node, end_node)
            obj.connectivity_vector = [init_node.global_coordinates, end_node.global_coordinates];
        end

        function obj = set_rotation_matrix(obj)
            obj.rotation_matrix = [obj.cosine obj.sine 0 0; 
                                   -obj.sine obj.cosine 0 0; 
                                   0 0 obj.cosine obj.sine; 
                                   0 0 -obj.sine obj.cosine];
        end

        function obj = set_element_stiffness_matrix_local_coordinates(obj)
            obj.local_stiffness_local_coord = (obj.E*obj.A/obj.L)*[1 0 -1 0; 
                                                                   0 0 0 0; 
                                                                   -1 0 1 0; 
                                                                   0 0 0 0];
        end
    end
end

