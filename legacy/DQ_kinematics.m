% LEGACY class. See also DQ_SerialManipulator
classdef DQ_kinematics < DQ_SerialManipulator
    methods
      function obj = DQ_kinematics(A,type)
         obj = obj@DQ_SerialManipulator(A,type);
         warning('Deprecated class. Please use DQ_SerialManipulator instead.');
      end
   end 
end