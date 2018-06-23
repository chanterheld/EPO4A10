classdef field < matlab.apps.AppBase
    properties(Access = private)
        handles
        
    end
    
    properties(Access = public)
        field_sel = 0;
        
        Car
        Heading
        Mic
        Field
        Points
        Ratio
        
        last_measured
        
        map
        free_field
        
        cmap_im
        
    end
    
    methods (Access = public)
        function app = field(parent)
            app.handles = parent;
            app.map.active = 0;
            app.free_field.active = 0;
            load('colour_layout', 'cmap')
            app.cmap_im = (cmap);
            disp(app.handles,'starting field class');
        end
        
        function delete(app)
            if app.map.active
                close(app.map.fig)
            end
            
            if app.free_field.active
                close(app.free_field.fig)
            end
            
            disp(app.handles, 'closing field class');
        end
        
        function load(app,name)
            app.field_sel = 0;
            location = strcat('field',num2str(name));
            try
                    load(location, 'MIC_s', 'Heading_s', 'Points_s', 'Ratio_s', 'Field_s');
                    app.Car = Points_s(5,:);
                    app.last_measured = Points_s(5,:);
                    app.Heading = Heading_s;
                    app.Mic = MIC_s;
                    app.Points = Points_s;
                    app.Ratio = Ratio_s;                    
                    app.Field = Field_s;
                    disp(app.handles,strcat({'Field '},num2str(name),{' loaded'}  ));
            catch
                    disp(app.handles,'failed to load field');
                    return                    
            end              

                app.field_sel = 1;
        end
        
        function plot_fig(app)
                if ~app.map.active
                    app.map.fig = figure;
                    app.map.active = 1;
                    app.map.axes = subplot(1,1,1);
                    app.map.fig.CloseRequestFcn = createCallbackFcn(app, @plotCloseRequest, true);
                end
                axes(app.map.axes);
%                 imagesc(flipud(app.Field))
%                 set(gca,'YDir','normal')
                %imagesc(app.Field)
                imagesc(rot90(flipud(app.Field),-1))
                set(gca,'YDir','normal')
                
                colormap(flipud(gray));
                hold on
                labels = {'A','B','C','D','Start','Car'};
                plot([app.Points(:,1);app.Car(1)],[app.Points(:,2);app.Car(2)],'o')
                text([app.Points(:,1);app.Car(1)],[app.Points(:,2);app.Car(2)],labels, 'VerticalAlignment','bottom','HorizontalAlignment','right')
                hold off
                
        end
       
        function plot_free_field(app)
                if ~app.free_field.active
                    app.free_field.fig = figure;
                    app.free_field.active = 1;
                    app.free_field.axes = subplot(1,1,1);
                    app.free_field.fig.CloseRequestFcn = createCallbackFcn(app, @FFCloseRequest, true);
                end
                 [H, W] = size(app.free_field.map);                
                imagesc(rot90(flipud(app.free_field.map),-1),'Parent', app.free_field.axes)
                set(app.free_field.axes,'YDir','normal')
                set(app.free_field.axes,'XLim',[0 W])
                set(app.free_field.axes,'YLim',[0 H])
                colormap(app.free_field.axes,app.cmap_im);
                
                hold(app.free_field.axes,'on');
                labels = {'A','B','C','D','Start','Car'};
                plot(app.free_field.axes,[app.Points(:,1);app.Car(1)],[app.Points(:,2);app.Car(2)],'o')
                text(app.free_field.axes,[app.Points(:,1);app.Car(1)],[app.Points(:,2);app.Car(2)],labels, 'VerticalAlignment','bottom','HorizontalAlignment','right')
                hold(app.free_field.axes,'off');
                
       end
        
       function plotCloseRequest(app, ~)
            app.map.active = 0;
            delete(app.map.fig)
       end
        
       function FFCloseRequest(app, ~)
            app.free_field.active = 0;
            delete(app.free_field.fig)
        end 
        
    end
    
    methods(Access  = private)
        
    end
end

