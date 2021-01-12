classdef CellStorage < handle
    properties
        Offset
        Map
    end
    
    methods
        function obj = CellStorage(offset_size)
            if nargin == 0
                obj.Offset = 1000;
            else
                obj.Offset = offset_size;
            end
            obj.Map = cell(obj.Offset*2,obj.Offset*2);
        end
        function r = get(obj,x,y)
            x = round(x+obj.Offset);
            y = round(y+obj.Offset);
            if x >= obj.Offset*2 || y >= obj.Offset*2
                error('invalid CellStorage range');
            end
            r = 0;
            v = obj.Map{x,y};
            if ~isempty(v)
                r = v;
            end
        end
        function r = set(obj,x,y,v)
            x = round(x+obj.Offset);
            y = round(y+obj.Offset);
            if x >= obj.Offset*2 || y >= obj.Offset*2
                error('invalid CellStorage range');
            end
            obj.Map(x,y) = {v};
        end
        function r = getall(obj)
            r = obj.Map{find(~cellfun('isempty', obj.Map))};
        end
        function r = isempty(obj,x,y)
            x = round(x+obj.Offset);
            y = round(y+obj.Offset);
            v = obj.Map{x,y};
            r = isempty(v);
        end
    end
end