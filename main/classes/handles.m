classdef handles < handle
    properties (Access = public)
        gui
        kitt
        comm
        sim
        field
        
        main_path
        folders = [{'\data_files\profiles'}; %models(1)
                    {'\data_files\fields'} %fields with start/stop points (2)
                    {'\data_files\runs'} %data from runs with data acquisition enabled (3)
                    {'\data_files\close_saves'}]; %close save variables(4)
        
        global_tic;
        
        gui_available = 0;
        sim_available = 0;
        abort = 0;
        
        Debug = 0;
        update_gui = 1;
        data_acq = [0, 0];
        t0
        
        audio
    end
    
    methods (Access = public)
        function app  = handles(Debug, update_gui, data_acq, path, ~)
            app.Debug = Debug;
            app.update_gui = update_gui;
            app.main_path = path;
            
            app.data_acq = [0, data_acq];
            
            app.kitt = kitt(app);           
            app.comm = comm(app);
            app.field = field(app);
            
            reset_datav(app);
%           app.sim = sim(app);

  load('meting.mat','f_b', 'f_c', 'c_r', 'code');
 load('ref5','ref');
%  xref = micx5;


            app.audio.Fs = 48000;
            app.audio.f_b = f_b;
            app.audio.f_c = f_c; 
            app.audio.c_r = c_r;
            app.audio.code = code;
            app.audio.ref = ref;
            app.audio.pulse_rate = app.audio.f_b/app.audio.c_r;
            app.audio.peaks_measured = 8;
            app.audio.nsamples = (1/app.audio.pulse_rate)*(app.audio.peaks_measured + 1)*app.audio.Fs;
            clear x;
            
            

            if Debug; assignin('base','handles',app);end;
        end
        
        function delete(app)
            delete(app.comm);
            delete(app.kitt);
            delete(app.field);
            pause(2)
            if app.gui_available
            app.gui_available = 0;
            delete(app.gui);
            end
            disp('closing handle file')
        end
        
        function disp(app, string)
           temp = 1;
           if app.Debug
               disp(string);
               temp = 0;
           end
           if app.gui_available && app.update_gui
               disp(app.gui, string);
           elseif temp && app.update_gui
               disp(string);
           end            
        end
        
        function reset_datav(app)           
           reset_datav(app.kitt)
           reset_datav(app.comm)
        end
        
       function save_datav(app, name)
           data_kitt = app.kitt.datav{1:app.kitt.n,:};
           data_comm1 = app.comm.datav1{1:app.comm.n1,:};
           data_comm2 = app.comm.datav2{1:app.comm.n2,:};
           t0_s = app.t0;
           path = strcat(app.main_path, app.folders{3}, '\', name);
           save(path, 'data_kitt', 'data_comm1', 'data_comm2', 't0_s');
       end
        
       function [output] = multi_isa(app,type,varargin)
            if nargin <=2
               error('not enough input arguments')
               return;
            end
            output = true;
            for k = 1:nargin-2
                output = and(output,isa(varargin{k},type));
            end
            return
        end
        
    end    
end