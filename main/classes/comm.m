classdef comm < handle
    properties(Access = private)
        handles
        
    end
    
    properties(Access = public)
        connected = 0;
        vir_con = 0;
        comport
        
        direction
        speed
        distanceL
        distanceR
        voltage
        beacon
        code
        f_c
        f_b
        rep        
        avg_dis        

        datav1
        n1 = 1;     
        datav2
        n2 = 1;
    end
   
    methods (Access = public) %connection functions
        function app = comm(parent)
            app.handles = parent;
            disp(app.handles,'starting com class');
        end
        
        function delete(app)
            close(app);
            disp(app.handles, 'closing com class');
        end
        
        function result = open(app,com_int)
            app.comport = num2str(com_int);
            if app.connected || app.vir_con
                   disp(app.handles, 'already connected');
                   result = 1;
            else
                if com_int < 0
                    %simulation code
                    %virtual connected
                    disp(app.handles, strcat('connected to virtual com',app.comport));
                    app.vir_con = 1;
                    result = 1;
                else
                    n = 0;
                    open = 0;
                    while ~open && n<10
                        open = EPOCommunications('open', strcat('\\.\COM',app.comport));
                        n = n+1;
                    end
                    if open
                        disp(app.handles, strcat('connected to com',app.comport));
                        app.connected = 1;
                        result = 1;
                    else
                       disp(app.handles, strcat('failed to open comm',app.comport)); 
                       result = 0;
                    end
                end
            end
        end
        
        function close(app)
             if app.connected
                n = 0;
                close = 0;
                while ~close && n<10
                    close = EPOCommunications('close');
                    n = n+1;
                end
                if ~close
                    disp(app.handles, strcat('failed to close com',app.comport))
                else
                    disp(app.handles, strcat('Com',app.comport,' closed'));
                    app.connected = 0;
                end
            elseif app.vir_con               
                %close simulation
                disp(app.handles, strcat('Virtual com',app.comport,' closed'));
                app.vir_con = 0;
            else
                 disp(app.handles, 'No open comports');
            end         
        end
        
        function result = send(app, data)
            if app.connected
                for k = 1:size(data,1)
                    for i = 1:3
                            EPOCommunications('transmit', data{k,:})                            
                    end
                end
            elseif app.vir_con
                for k = 1:size(data,1)
                    disp(app.handles, strcat({'Send to virtual car: '}, data(k,:) ));
                end
                result = 1;
            else
                disp(app.handles, 'Connect before sending data');
            end
        end
        
       function response = get_status(app, data)
            if app.connected
               status = 0;
               n = 0;
               switch data
                    case 'Distance'
                        while ~isa(status,'char') && n<5
                            try
                                status = EPOCommunications('transmit', 'Sd');
                            catch
                                %use try statement to prefent errors
                                n = n + 1;
                            end
                        end
                        
                        if isa(status,'char')
                            string = strsplit(status,{'R','L', '\n'});
                            app.distanceL = str2num(string{1,2});
                            app.distanceR= str2num(string{1,4});
                            response = 1;
                            save_avg_dis(app);
                        else
                            disp(app.handles,'No response from car');
                            response = 0;
                        end
                        
                    case 'Voltage'                        
                        while ~isa(status,'char') && n<5
                            try
                                status = EPOCommunications('transmit', 'Sv');
                            catch
                                %use try statement to prefent errors
                                n = n + 1;
                            end
                        end
                        
                        if isa(status,'char')
                            string = strsplit(status,{'T','V'});
                            app.voltage = string{1,3};
                            response = 1;
                        else
                            disp(app.handles,'No response from car');
                            response = 0;
                        end
                        
                    case 'All'
                        disp('controll to comm')
                        while ~isa(status,'char') && n<5
                            try
                                status = EPOCommunications('transmit', 'S');
                            catch
                                %use try statement to prefent errors
                                n = n + 1;
                            end
                        end
                        
                        if isa(status,'char')
                            string = strsplit(status);
                            app.direction = str2num(string{1,24});
                            app.speed = str2num(string{1,27});
                            app.distanceL = str2num(string{1,34});
                            app.distanceR = str2num(string{1,36});
                            app.voltage = str2num(string{1,39});
                            app.beacon = string{1,5};
                            app.code = strcat(string{1,8},string{1,9});
                            app.f_c = str2num(string{1,12});
                            app.f_b = str2num(string{1,15});
                            app.rep = str2num(string{1,18});
                            response = 1;
                            save_avg_dis(app);
                        else
                            disp(app.handles,'No response from car');
                            response = 0;
                        end
               end
               
               if app.handles.gui_available && app.handles.update_gui && response
                  update(app.handles.gui,'COMM');
               end
               
               if app.handles.data_acq(1)
                    app.n1 = app.n1 + 1;
                    app.datav1{app.n1,1} = toc(app.handles.global_tic);
                    app.datav1{app.n1,2} = app.direction;
                    app.datav1{app.n1,3} = app.speed;
                    app.datav1{app.n1,4} = app.distanceL;
                    app.datav1{app.n1,5} = app.distanceR;
                    app.datav1{app.n1,6} = data;
               end
               
            elseif app.vir_con               
                %get sim data
                            load('statusws.mat','status1')
                            string = strsplit(status1);
                            app.direction = str2num(string{1,24});
                            app.speed = str2num(string{1,27});
                            app.distanceL = str2num(string{1,34});
                            app.distanceR = str2num(string{1,36});
                            app.voltage = str2num(string{1,39});
                            app.beacon = string{1,5};
                            app.code = strcat(string{1,8},string{1,9});
                            app.f_c = str2num(string{1,12});
                            app.f_b = str2num(string{1,15});
                            app.rep = str2num(string{1,18});
                            response = 1;
                            save_avg_dis(app);
%                             delay_cor(app);
                            
                            response = 1;
                %temp random string
                if app.handles.gui_available && app.handles.update_gui
                    update(app.handles.gui,'COMM');
                end 
            else
                 disp(app.handles, 'Connect before requesting data');
                 response = 0;
            end
            
        end
    
    end
    
     methods (Access = public) %data processing function
         
         function save_avg_dis(app)
             if app.distanceL == 0
                 app.distanceL = 999;
             end
             if app.distanceR == 0
                 app.distanceR = 999;
             end
             if abs(app.distanceL - app.distanceR) < 40
                 app.avg_dis = (app.distanceL + app.distanceR)/200;
             else
                 app.avg_dis = min(app.distanceL, app.distanceR)/100;
             end
         end
         
         function reset_datav(app)
            app.n1 = 0;
            app.datav1 = num2cell(zeros(50,6));
            app.n2 = 0;
            app.datav2 = num2cell(zeros(50,2));
         end
         
     end
         
end