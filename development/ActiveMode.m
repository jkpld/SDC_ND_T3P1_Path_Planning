classdef ActiveMode
    properties
        name
        data
    end
    methods
        function obj = ActiveMode(name, data)
            obj.name = name;
            obj.data = data;
        end
    end
end