classdef kitt < matlab.apps.AppBase    
    properties(Access = public)
        handles
        
        Fmax
        
        sens_to_bumb = -.5;%meter => 10cm
        inner_radius = .57;
        outer_radius = .96;

        last_data_t
        
        contr_vec = [.4; 0.55; 0.65; 0.8; 0.85; 0.9;]
        x_count = [1 1];
        v_count = [1 1];
        
        ignore_sensors = 1;

        BR_Xfit
        BR_Tfit
        
        Afit
        Bfit
        Gfit
        Lfit
        a
        b
        g
        l
        
        map
    end
    
    properties (Access = private)
       turn_speed = [	0%151
                        0
                        0
                        0
                        0
                        0.502183291;%156
                        0.80090872;
                        1.027894371;
                        1.285608521;
                        1.44254214;
                        1.545402391;
                        1.681304611;
                        1.752381087;
                        1.806097801;
                        1.839613018];%165
                    
          V_drive
          
 
    end
    
    properties(Access = public) %midterm
         measurements
         datav
         n
         profile_loaded = 0;
         
         ctpc
         pctc
         sw_dis
         
         T
         Brake_P = 135;
    end
    
    methods(Access = public)
        
        function app = kitt(parent)
            
            disp(parent,'starting kitt class');           
            app.handles = parent;
            path = strcat(app.handles.main_path, app.handles.folders{4},'\kitt_save');
            try
                load(path, 'ctpc_s', 'pctc_s', 'sw_dis_s');
                app.ctpc = ctpc_s;
                app.pctc = pctc_s;
                app.sw_dis = sw_dis_s;
            catch
                app.ctpc = .1;
                app.pctc = .3;
                app.sw_dis = 3;
            end
            
        end
        
        function delete(app)
            ctpc_s = app.ctpc;
            pctc_s = app.pctc;
            sw_dis_s = app.sw_dis;
            path = strcat(app.handles.main_path, app.handles.folders{4},'\kitt_save');
            save(path, 'ctpc_s', 'pctc_s', 'sw_dis_s');
            disp(app.handles, 'closing kitt class');
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
       
        
        
        
        function runtime = wall_break(app, V_start, V_br, dis_to_wall)
            %Check for connection
            app.handles.global_tic = tic;
            if ~app.handles.comm.connected && ~app.handles.comm.vir_con
                disp(app.handles,'Connect to car first')
                runtime = 0;
                return;
            end
            
            %Check for profile
            if ~app.profile_loaded
               disp(app.handles,'No profile selected')
               runtime = 0;
               return;
            end
            
            %Collect data Y/N
            %can be copy'ed to any other gui, called using delete/save
            %function in handle class
            %use app.handles.comm.data_tic = tic to sync with comm class
            if app.handles.data_acq(2)
                app.handles.data_acq(1) = 1;
                reset_datav(app.handles);
            end
            
            %Get initial distance + wake up BT
            send(app.handles.comm,{'D150'});
            for k = 1:5
                response = get_status(app.handles.comm,'All');
                if ~response
                    disp(app.handles.gui,'Wall run cancelled');
                    runtime = toc(app.handles.global_tic);
                    return;                   
                end
                d(k) = app.handles.comm.avg_dis;
            end
            
            %Initialise variables
            dis_to_wall = dis_to_wall + app.sens_to_bumb;
            refresh = 0;            
            x0 = [0,mean(d)];
            v0 = 0;
            update_scalers(app, app.T(V_start-149), v0);
            
            if x0(2) > app.sw_dis
                app.ignore_sensors = 1;
            else
                app.ignore_sensors = 0;
            end
            
            %Confirmation
            answer = questdlg('Confirm wall break test','Confirm','Ok','Cancel','Cancel');
            if ~strcmp(answer,'Ok')
                runtime = 0;
                return;
            end 
            
            
            %t0 def
            app.handles.t0 = toc(app.handles.global_tic);
            send(app.handles.comm,{strcat('M',num2str(V_start))});
            %Disable Gui with progress bar
            if app.handles.gui_available
                progress = uiprogressdlg(app.handles.gui.mainfig,'Title','Driving','Indeterminate','on','Cancelable','on');
            end
            %Match with simulation time
            pause(app.pctc - toc(app.handles.global_tic)+app.handles.t0);
            comm = tic;
            
            
            while ~progress.CancelRequested
                
                %get data from kitt 
                response = get_status(app.handles.comm, 'Distance');
                app.last_data_t = toc(comm);
                comm = tic;
                
                if ~response
                   disp(app.handles.gui,'Wall run cancelled');
                   runtime = toc(app.handles.global_tic);
                   return;                   
                end  
                
               [x_exp, v_exp] = extrapolate(app, app.last_data_t);
               x_extr = x0(2)-x_exp;
               v_extr = v_exp + v0;
                
               if app.ignore_sensors &&  app.handles.comm.avg_dis > app.sw_dis
                   v0 = v_extr;
                   x0 = [x0(2), app.handles.comm.avg_dis];
               else
                app.ignore_sensors = 0;
               
                x0 = [x0(2), (app.handles.comm.avg_dis + x_extr)/2];
                %v0 = (((-diff(x0))/app.last_data_t) + v0+v_exp)/2;
                v0 = v_extr;
               end
               
               if app.handles.data_acq(1)
                    app.n = app.n+1;
                    app.datav(app.n,:) = [toc(app.handles.global_tic), x_extr, v_extr, app.handles.comm.avg_dis, (-diff(x0))/app.last_data_t ,x0(2), v0, app.ignore_sensors];
               end                
                
               update_scalers(app, app.T(V_start-149), v0);
                
                if ~app.ignore_sensors
                    idts = x0(2)- dis_to_wall;
                    for t = (app.ctpc + app.pctc):.01:(app.pctc + app.ctpc + app.last_data_t + .1)
                        [dx, dv] = extrapolate(app, t);
                        Xbr = app.BR_Xfit(app.T(151-V_br),v0+dv);
                        if (idts - dx - Xbr) <= 0
                            pause(t - app.ctpc - app.pctc - toc(comm));
                            sendtime = tic;
                            send(app.handles.comm,{strcat('M',num2str(V_br))});                        
                            pause(app.BR_Tfit(app.T(151-V_br),v0+dv)-toc(sendtime));
                            send(app.handles.comm,{'M150'});
                            runtime = toc(app.handles.global_tic)-app.handles.t0;
                            app.handles.data_acq(1) = 0;
                            close(progress)
                            return;
                        end
                    end                
                end
                
                if toc(app.handles.global_tic) - refresh > 2
                    send(app.handles.comm,{strcat('M',num2str(V_start))});
                    refresh = toc(app.handles.global_tic);
                end  
                   
            end
            
            runtime = toc(app.handles.global_tic)-app.handles.t0;
            send(app.handles.comm,{'M150'});       
            app.handles.data_acq(1) = 0;
            close(progress)
        end
        

    end
    
    methods(Access = private)
       
        function update_scalers(app, T, V)
            app.a = app.Afit(T,V);
            app.b = app.Bfit(T,V);
            app.g = app.Gfit(T,V);
            app.l = app.Lfit(T,V);
        end
        
        function [x, v] = extrapolate(app, t)
            x = (app.b*t) +(app.a*t^2);
            %tv = t + app.last_data_t/2; 
            v = (app.l*tv) +(app.g*tv^2);
        end
        

        
    end
    
    methods(Access = public)
        
        function out = load_profile(app,name)
            path = strcat(app.handles.main_path, app.handles.folders{1}, '\', name);
            try
                load(path, 'BR_X_cell_s', 'BR_T_cell_s', 'A_cell_s', 'B_cell_s', 'G_cell_s', 'L_cell_s', 'prop');
            catch
                disp(app.handles,'Profile does not exist');
                app.profile_loaded = 0;
                out = 0;
                return;
            end
            if ~multi_isa(app.handles,'sfit',BR_X_cell_s{5}, BR_T_cell_s{5}, A_cell_s{5}, B_cell_s{5}, G_cell_s{5}, L_cell_s{5});
               disp(app.handles,'Profile is not complete');
               app.profile_loaded = 0;
               out = 0;
               return;
            end
            app.Afit = A_cell_s{5};
            app.Bfit = B_cell_s{5};
            app.Gfit = G_cell_s{5};
            app.Lfit = L_cell_s{5};
            app.BR_Xfit = BR_X_cell_s{5};
            app.BR_Tfit = BR_T_cell_s{5};
            app.T = prop{6};            
            app.profile_loaded = 1;
            disp(app.handles, strcat(name, ' loaded'));
            out = 1;
        end
        
        function reset_datav(app)
            app.datav = zeros(50,8);
            app.n = 0;
        end
        
        function brake(app,V,Brake_P)
              send_brake = tic;
              send(app.handles.comm,{strcat('M',num2str(Brake_P))});                        
              pause(app.BR_Tfit(app.T(151-Brake_P),V)-toc(send_brake));
              send(app.handles.comm,{'M150'}); 
        end
        
%         function [moving_avg] = match_data(app, or, ex, count)
%             ind = min(length(app.contr_vec), count(2))
%             
%                     contr_vec
%         end
        
    end
    

    
end

