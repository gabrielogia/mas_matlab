classdef node    
    properties
        id
        x
        y
        sup_x
        sup_y
        sup_z
        k_x
        k_y
        k_z
        f_x
        f_y
        f_z
        global_coordinates = [];
        model
    end
    
    methods
        function obj = node(id, x, y, sup_x, sup_y, sup_z, k_x, k_y, k_z, f_x, f_y, f_z, model)
            obj.id = id;
            obj.x = x;
            obj.y = y;
            obj.sup_x = sup_x;
            obj.sup_y = sup_y;
            obj.sup_z = sup_z;
            obj.k_x = k_x;
            obj.k_y = k_y;
            obj.k_z = k_z;
            obj.f_x = f_x;
            obj.f_y = f_y;
            obj.f_z = f_z;
            obj.model = model;
            
            if (model == "truss")
                obj.global_coordinates = [obj.global_coordinates, zeros(1,2)];
            else
                obj.global_coordinates = [obj.global_coordinates, zeros(1,3)];
            end
        end

        function [f,r] = count_degrees(obj)
            f = 0;
            r = 0;

            if (obj.sup_x == 0)
                f = f+1;
            else
                r = r+1;
            end

            if (obj.sup_y == 0)
                f = f+1;
            else
                r = r+1;
            end
        end

        function [obj, k] = set_free_global_coordinates(obj,k)
            if (obj.sup_x == 0)
                obj.global_coordinates(1) = k;
                k = k + 1;
            end

            if (obj.sup_y == 0)
                obj.global_coordinates(2) = k;
                k = k + 1;
            end
        end

        function [obj, k] = set_restrained_global_coordinates(obj,k)
            if (obj.sup_x == 1)
                obj.global_coordinates(1) = k;
                k = k + 1;
            end

            if (obj.sup_y == 1)
                obj.global_coordinates(2) = k;
                k = k + 1;
            end
        end
    end
end

