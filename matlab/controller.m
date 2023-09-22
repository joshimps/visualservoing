classdef controller < handle
   
   properties (Access=private)
        %Logger
        logger

        %Robot Objects
        UR3Object
   end
   
   methods (Access=public)
      
      % -------------------------------------------------------------------------- %
      %                                Constructor                                 %
      % -------------------------------------------------------------------------- %
      
      function self = Controller(uR3BaseTransform)
         hold on
         run('Peter Cork Toolbox\rvctools\startup_rvc.m')
         self.logger = log4matlab('log.log'); 
         self.logger.SetCommandWindowLevel(self.logger.DEBUG)

         
         self.UR3Object = UR3(uR3BaseTransform);
         self.logger.mlog = {self.logger.DEBUG,'LinearUR3Controller',"Linear UR3 Created With Base Transform:"};
         self.logger.mlog = {self.logger.DEBUG,'LinearUR3Controller',self.logger.MatrixToString(self.robotObject.model.base.T)}; 
        
        
      end
      
      % -------------------------------------------------------------------------- %
      %                               Runners                                      %
      % -------------------------------------------------------------------------- %
      
     
      
      % -------------------------------------------------------------------------- %
      %                               Getters                                      %
      % -------------------------------------------------------------------------- %
      
   
   end

   methods (Access=private)
       
      % -------------------------------------------------------------------------- %
      %                              Setters                                       %
      % -------------------------------------------------------------------------- %

      % -------------------------------------------------------------------------- %
      %                              Helpers                                       %
      % -------------------------------------------------------------------------- %
   end
end

