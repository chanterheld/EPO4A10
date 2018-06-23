%%Todo  While loop variable/cancel options

function runtime = drive_final(app, order, collision_en, speed_str, speed_trn, driveT, brake_val)

audio = app.handles.audio; %audio variables => shorten variable name
app.handles.global_tic = tic; %enable global tic as refrence frame for measurements;

if ~app.handles.comm.connected && ~app.handles.comm.vir_con %Make sure car is connected in some way
    disp(app.handles,'Connect to car first')
    runtime = 0;
    return;
end

%Load Matrixes and fit objects responsible for predicting car performance
try
    load('drive_stop_fits', 'TTS_str', 'TTS_trn', 'BR_Tfit_str', 'BR_Tfit_trn', 'TimeVec', 'Fmax');
    TTS_trn = TTS_trn - .15;
catch
    disp(app.handles,'Fits not found')
    runtime = 0;
    return;
end

if ~app.handles.field.field_sel
    runtime = 0;
    disp(app.handles,'No field loaded')
    return;
end                  

expected_hdng = app.handles.field.Heading; %expected heading

ForceVec = linspace(0, Fmax, 16);           %DrivingValue to Force mapper

BrakeF = ForceVec(abs(150-brake_val)+1);   %Breaking force for this run

radia = [.57, .72, .96];                    %inner, speaker, outer radia
radia_sc = radia*app.handles.field.Ratio;   %both scaled and not

maxdev = .1;                                %max cut corner distance

just_turned = 0;                            %flag => last move was a turn
turn_dir = 'L';                             %flag => last turn was L/R
just_reversed = 0;                          %flag => last move was reversed

hit = 0;                                    %flag => used to abort planned move
hit_cnt = 0;                                %counter => rerout when multiple times drive backwards
rerout_flag = 0;                            %flag => rerout on next pass


cycle = 0;                                  %number of loops;
skip = 1;                                   %flag => skip position and heading fetch and update

last_path_point = app.handles.field.Car;    %used to calculate gone path to remove waypoints

rout_err_dev = [0 0;
                0 10;
                0 -10;
                10 0;
                -10 0];

%calculate max distance driven straight by integrating speed for driveT
intr_dis = 0;
br_dis = 0;
prevT = 0;
prevV = 0;
loopvar = 1;
cT = 0;
saved = 0;
while cT < driveT
    cV = TTS_str(speed_str-149,loopvar);
    cT = TimeVec(loopvar);
    intr_dis = intr_dis + (cT-prevT)*((cV + prevV)/2);
    br_t = BR_Tfit_str(BrakeF,cV);
    br_dis = (cV/2)*br_t;
    prevT = cT;
    prevV = cV;
    loopvar = loopvar+1;
    if cT >= driveT/2 && ~saved
       saved = 1;
       half_str_dis = intr_dis + br_dis;
    end
end
max_str_dis = intr_dis + br_dis;

%when no obstacles are place. fill field with known free space;
app.handles.field.free_field.map = app.handles.field.Field;
if ~collision_en
    [Hf, Wf] = size(app.handles.field.free_field.map);
    for m = 1:Hf
        for n = 1:Wf
            if app.handles.field.Field(m,n) == 0
                app.handles.field.free_field.map(m,n) = 2;
            end
        end
    end
end
app.handles.field.free_field.map(end,end) = 0;
app.handles.field.free_field.map(1,end) = 2;

bare_field = app.handles.field.Field; %walls are 1's, able to ignore anything outside field
rout_field = app.handles.field.Field;

if collision_en &&  app.handles.comm.vir_con    %If virtualizing with collision sensors enabled give
    obstacle_map =  app.handles.field.Field;    %option to add virtual opstacles to detect
    answer = questdlg('Add obstacle?','Yes','yes','No','no');
    if strcmp(answer,'yes')
        [obstacle_map] = place_obstacles(obstacle_map, 4, app.handles.field.Points, {'A','B','C','D','Start'});
    end
end

if app.handles.comm.vir_con  %If debug enable show field => what the car thinks is going on
    plot_fig(app.handles.field);
end

if app.handles.comm.connected %Integrated way to make the windows asio menu popup
    answer = questdlg('Turn on asio?','Confirm','Ok','Cancel','Cancel');
    if strcmp(answer,'Ok')
        pa_wavrecord(1,1,20*8000);
    end
end

for i = 1:size(order,1) %Makes a destination vector out of order vector
    destinations(i,:) = app.handles.field.Points(order(i),:);
end

[H,W] = size(app.handles.field.Field);
LinH = linspace(1,H,46);
LinW = linspace(1,W,46);

ob_mid_list = round([ones(46,1).*20,LinH';ones(46,1).*(W-20),LinH';LinW',ones(46,1).*20;LinW',ones(46,1).*(H-20)]);

[~, closest]=min(sum((ob_mid_list - app.handles.field.Car).^2,2));
first_point = ob_mid_list(closest,:) + extr_pos(angl([ob_mid_list(closest,:);app.handles.field.Car]),.5*app.handles.field.Ratio);

%Inital rout setup and rout for current position.
[Hn,Neighboors] = rout_set(H, W, destinations(1,:),3);
[waypoints,err] = rout(app.handles.field.Car,destinations(1,:), rout_field,Neighboors,Hn, radia(2), maxdev,first_point);
if err
    disp(app.handles,'routing failed')
    runtime = 0;
    return;
end



                         %list of obstacle mid points


send(app.handles.comm,{strcat('B',num2str(audio.f_b,'%d'));
                       strcat('F', num2str(audio.f_c,'%d'));
                       strcat('R', num2str(audio.c_r,'%d'));
                       strcat('C0x', audio.code)});

answer = questdlg('Confirm Drive','Confirm','Ok','Cancel','Cancel');
if ~strcmp(answer,'Ok')
    runtime = 0;
    return;
end

app.handles.t0 = toc(app.handles.global_tic); %start driving
send(app.handles.comm, {'A1'});

while ~app.handles.gui.Cancel.Value
    if ~skip
        if ~app.handles.comm.vir_con %%Record audio => Convert to distances => Convert to position
              [app.handles.field.Car,score] = get_car_position(audio,app.handles.field.Ratio,app.handles.field.Mic);
              if score > 700
                  app.handles.field.Car = final_point;
                  disp(app.handles, 'Car position 100% predicted');
              end
              app.handles.field.Car = round(app.handles.field.Car);
%             failed = 1;
%             while failed
%                 try
%                 [y_data] = record(audio.Fs,audio.nsamples);
%                 [D] = TDOA2(y_data,audio.ref,audio.pulse_rate);
%                 app.handles.field.Car = round(lst_sq_pos3D(D.*app.handles.field.Ratio, app.handles.field.Mic, 7.5*app.handles.field.Ratio, .3*app.handles.field.Ratio));
%                 
%                 if app.handles.field.Car(1) > W - 36 || app.handles.field.Car(1) < 36 || app.handles.field.Car(2) > H - 36 || app.handles.field.Car(2) < 36
%                     failed = 1;
%                     disp(app.handles,'Pos Failed')
%                 else
%                     failed = 0;
%                 end
%                 catch
%                     failed  = 1;
%                 end
%             end
        else %Virtual position using Ginput
            if ~app.handles.field.map.active
                plot_fig(app.handles.field);
            end
            axes(app.handles.field.map.axes)
            [x,y] = ginput(1);
            app.handles.field.Car = round([x,y]);
            plot_fig(app.handles.field);
        end
            
        if app.handles.Debug %Debug messages
            disp(app.handles,strcat({'Car located at: ['},{num2str(app.handles.field.Car(1))},{','},{num2str((2))},{'] / ['}, {num2str(app.handles.field.Car(1)/app.handles.field.Ratio)},{','},{num2str(app.handles.field.Car(2)/app.handles.field.Ratio)},{']'}))
            disp(app.handles,strcat(strcat({'Distance moved since last heading update:'},{num2str(dist(app.handles.field.Car, app.handles.field.last_measured))})))
        end
        
        if just_turned %Angle calculated based on centre of turn and current position
            switch turn_dir
                case 'L'
                    app.handles.field.Heading = angl([turn_mid;app.handles.field.Car]) + 90;
                case 'R'
                    app.handles.field.Heading = angl([turn_mid;app.handles.field.Car]) - 90;
            end
            if app.handles.field.Heading<-180 %-180<ang<180 angle boundry's
                app.handles.field.Heading = app.handles.field.Heading + 360;
            elseif app.handles.field.Heading > 180
                app.handles.field.Heading = app.handles.field.Heading - 360;
            end
            app.handles.field.last_measured = app.handles.field.Car; %Update last measured position
        elseif(dist(app.handles.field.Car, app.handles.field.last_measured) > .4*app.handles.field.Ratio)
            app.handles.field.Heading = angl([app.handles.field.last_measured;app.handles.field.Car]);
            app.handles.field.last_measured = app.handles.field.Car;
            if just_reversed %%Flip heading if the car reversed and that was logged
                if app.handles.field.Heading > 0
                    app.handles.field.Heading = app.handles.field.Heading - 180;
                else
                    app.handles.field.Heading = app.handles.field.Heading + 180;
                end
            end
        end
        
        if app.handles.Debug %Debug message
            disp(app.handles,strcat({'Measure heading Car: '}, {num2str(app.handles.field.Heading)}, {' degrees'} ))
        end
        
        %Avarage calcutated heading with expected heading
        exp_hdng_dif = app.handles.field.Heading-expected_hdng;
        if exp_hdng_dif<-180
            exp_hdng_dif = exp_hdng_dif + 360;
        elseif exp_hdng_dif > 180
            exp_hdng_dif = exp_hdng_dif - 360;
        end
        app.handles.field.Heading = app.handles.field.Heading - exp_hdng_dif/2;
        if app.handles.field.Heading<-180
            app.handles.field.Heading = app.handles.field.Heading + 360;
        elseif app.handles.field.Heading > 180
            app.handles.field.Heading = app.handles.field.Heading - 360;
        end
        
        if app.handles.Debug %Debug message
            disp(app.handles,strcat({'Distance to next wp: '},{num2str(dist(app.handles.field.Car,waypoints(1,1:2)))}))
        end
        
        %calculate last path postitions to delete waypoints
        if just_turned
            [turn_mid_path,Fangl_path] = mid_point(app.handles.field.Car,radia_sc(2),app.handles.field.Heading, turn_dir);
            Sangl_path = angl([turn_mid_path;last_path_point]);
            path_ang_dif = Sangl_path - Fangl_path;
            if path_ang_dif<-180
                path_ang_dif = path_ang_dif + 360;
            elseif path_ang_dif > 180
                path_ang_dif = path_ang_dif - 360;
            end
            if ((turn_dir == 'L')&&path_ang_dif < 0)||((turn_dir == 'R')&& path_ang_dif >= 0)
                    path_cir = drawcircle(turn_mid_path, radia_sc(2));
                    path = arc_sel([Sangl_path,Fangl_path],path_cir,turn_dir);
            else
                path = [app.handles.field.Car];
            end
            
        else
            path = plotLine(last_path_point, app.handles.field.Car);
        end
        for i = 0:min(5, size(path,1)-1)
            if path(end-i,:) == last_path_point
                path = flipud(path);
                break;
            end
        end
        last_path_point = app.handles.field.Car;
        
        %Close to Destination
        if dist(app.handles.field.Car,destinations(1,:))<waypoints(end,3)*app.handles.field.Ratio
            if size(destinations,1) == 1 %Final destination
                runtime = toc(app.handles.global_tic)-app.handles.t0;
                send(app.handles.comm, {'A0';'M150'});
                return
            else %Intermediate Destination: pause
                send(app.handles.comm, {'A0'});
                pausetic = tic;
                disp(app.handles, 'Wait 5 seconds at waypoint')
                destinations = destinations(2:end,:);
                [Hn,Neighboors] = rout_set(H, W, destinations(1,:),3);
                for i = 1:5
                    [waypoints,err] = rout(app.handles.field.Car+rout_err_dev(i,:),destinations(1,:), rout_field,Neighboors,Hn,radia(2), maxdev,first_point);
                    if ~err
                        break;
                    end
                end
                if err
                    disp(app.handles,'failed to rout');
                    runtime = toc(app.handles.global_tic)-app.handles.t0;
                    return
                end
                pause(10-toc(pausetic));
                send(app.handles.comm, {'A1'});
            end
            %Close to waypoint
        
        else
            for i = 1:size(path,1)
                if size(waypoints,1)<2
                    break;
                end
                if dist(path(i,:),waypoints(1,1:2))<waypoints(1,3)*app.handles.field.Ratio
                    waypoints = waypoints(2:end,:);
                end
            end
        end
        
        
    else %if skip
        disp(app.handles,'Skipped');
        skip = 0;
    end %if n/y skip
        
    
    if collision_en
        if ~app.handles.comm.vir_con %Get Distance from car
            [dis, err] = fetch_dis();
            if  err
                disp(app.handles, 'Failed receiving data from car')
%                 runtime = toc(app.handles.global_tic)-app.handles.t0;
%                 return;
            end
        else %Get distance from virtual space
            collision_line = plotLine(app.handles.field.Car, app.handles.field.Car + round(extr_pos(app.handles.field.Heading,5*app.handles.field.Ratio)));
            if collision_line(end,:) == app.handles.field.Car
                collision_line = flipud(collision_line);
            end
            for i = 1:size(collision_line,1)
                if obstacle_map(collision_line(i,1),collision_line(i,2))
                    break
                end
            end
            dis = dist(app.handles.field.Car, collision_line(i,:))/app.handles.field.Ratio;
        end
        
        ob_add = 0;
        if dis < 2 %If close then 2 meters
            obj_mid = app.handles.field.Car + round(extr_pos(app.handles.field.Heading,(dis+.15)*app.handles.field.Ratio)); %Estimate center 15cm behind measurement
            if bare_field(obj_mid(1), obj_mid(2)) == 0                                                                          %Not part of the wall
                if app.handles.field.Field(obj_mid(1), obj_mid(2)) == 1                                                         %Point measured in already know object
                    min_mp_dist = 10*app.handles.field.Ratio;
                    for i = 1:size(ob_mid_list,1)
                        min_mp_dist = min(min_mp_dist, dist(obj_mid,ob_mid_list(i,:)));
                    end
                    if min_mp_dist > .25*app.handles.field.Ratio                                                                %If the new point is further then 25cm from all other mid points it is added
                        [obj_points] = draw_disk(obj_mid,.35*app.handles.field.Ratio);                                           %Object modeled as disk with 40cm radius
                        [app.handles.field.Field] = place_points(app.handles.field.Field, obj_points);
                        [app.handles.field.free_field.map] = place_points(app.handles.field.free_field.map, obj_points);
                        [wobj_points] = draw_disk(obj_mid,.45*app.handles.field.Ratio);  
                        [rout_field] = place_points(rout_field, wobj_points);
                        ob_mid_list = [ob_mid_list;obj_mid];
                        rerout_flag = 1;                                                                                        %Forced rerout when object added
                        if app.handles.Debug
                            disp(app.handles,'obstacle mid point added')
                            ob_add = 1;
                        end
                    end
                else                                                                                                            %when centre is outside other object => ofcourse also added
                    [obj_points] = draw_disk(obj_mid,.35*app.handles.field.Ratio);
                    [app.handles.field.Field] = place_points(app.handles.field.Field, obj_points);
                    [app.handles.field.free_field.map] = place_points(app.handles.field.free_field.map, obj_points);
                    [wobj_points] = draw_disk(obj_mid,.45*app.handles.field.Ratio);  
                    [rout_field] = place_points(rout_field, wobj_points);
                    ob_mid_list = [ob_mid_list;obj_mid];
                    rerout_flag = 1;
                    if app.handles.Debug
                        disp(app.handles,'obstacle mid point added')
                        ob_add = 1;
                    end
                end
            end
            if ob_add
                for m = 1:size(ob_mid_list,1)-1
                    if dist(ob_mid_list(m,:),ob_mid_list(end,:)) < 1*app.handles.field.Ratio
                        [con_ob_points] = plotLine(ob_mid_list(m,:),ob_mid_list(end,:));
                        [app.handles.field.Field] = place_points(app.handles.field.Field, con_ob_points);
                        [app.handles.field.free_field.map] = place_points(app.handles.field.free_field.map, con_ob_points);
                        [rout_field] = place_points(rout_field, con_ob_points);
                    end
                    
                end
            end
            [poly_cor,poly_mid] = FoF_poly(app.handles.field.Car,app.handles.field.Heading,dis-.1,app.handles.field.Ratio); %Clear area from car to object;
        else
            free_dis = min(3, dis-.1);
            [poly_cor,poly_mid] = FoF_poly(app.handles.field.Car,app.handles.field.Heading,free_dis,app.handles.field.Ratio);%Clear area max 3 meters
        end
        app.handles.field.free_field.map = free_place(app.handles.field.free_field.map, polyg_points(poly_cor,poly_mid));     %Clear area placed on map
    end
    
    if hit
        disp(app.handles,'hitted')
        hit = 0;
        hit_cnt = hit_cnt + 1;
        if hit_cnt > 2
            rerout_flag = 1;
        end
    else
       hit_cnt = 0; 
    end
    
    if rout_field(app.handles.field.Car(1),app.handles.field.Car(2)) == 1
        [~, closest]=min(sum((ob_mid_list - app.handles.field.Car).^2,2));
        first_point = ob_mid_list(closest,:) + extr_pos(angl([ob_mid_list(closest,:);app.handles.field.Car]),.5*app.handles.field.Ratio);
        first_point = ob_mid_list(closest,:) + extr_pos(angl([ob_mid_list(closest,:);app.handles.field.Car]),.5*app.handles.field.Ratio);
    end
    
    if rerout_flag % In case anything set rerout flag => rerout and clear
        for i = 1:5
            [waypoints,err] = rout(app.handles.field.Car+rout_err_dev(i,:),destinations(1,:), rout_field,Neighboors,Hn,radia(2), maxdev,first_point);
            if ~err
                break;
            end
        end
        if err
            disp(app.handles,'failed to rout');
            runtime = toc(app.handles.global_tic)-app.handles.t0;
           return
        end
        rerout_flag = 0;
    end
    
    obj_found = 0;
    while ~obj_found && size(waypoints,1)>1
        collision_line = plotLine(app.handles.field.Car, waypoints(2,1:2));
        for i = 1:size(collision_line,1)
            if app.handles.field.Field(collision_line(i,1),collision_line(i,2))
                obj_found = 1;
                break
            end
        end
        if ~obj_found
            waypoints = waypoints(2:end,:);
        end
    end
    
    
    %plot field with obstacles, known free area, route and current heading.
    plot_free_field(app.handles.field)
    vec_com = extr_pos(app.handles.field.Heading,.4*app.handles.field.Ratio);
    hold(app.handles.field.free_field.axes,'on')
    quiver(app.handles.field.free_field.axes, app.handles.field.Car(1),app.handles.field.Car(2),vec_com(1),vec_com(2),'MaxHeadSize',1);
    plot(app.handles.field.free_field.axes, [app.handles.field.Car(1); waypoints(:,1)],[app.handles.field.Car(2); waypoints(:,2)]);
    hold(app.handles.field.free_field.axes,'off')

    dir_to_wp = angl([app.handles.field.Car;waypoints(1,1:2)]);
    if app.handles.Debug %Debug measages
        disp(app.handles,strcat({'Expected heading Car: '}, {num2str(expected_hdng)}, {' degrees'} ))
        disp(app.handles,strcat({'Used heading Car: '}, {num2str(app.handles.field.Heading)}, {' degrees'} ))
        disp(app.handles,strcat({'Heading to WP: '}, {num2str(dir_to_wp)}, {' degrees'} ))
    end
    
    angl_err = (app.handles.field.Heading - dir_to_wp);
    if angl_err<-180
        angl_err = angl_err + 360;
    elseif angl_err > 180
        angl_err = angl_err - 360;
    end
    
    wall_close = 0;
    collision_line = plotLine(app.handles.field.Car, app.handles.field.Car + round(extr_pos(app.handles.field.Heading,1*app.handles.field.Ratio)));
    if collision_line(end,:) == app.handles.field.Car
        collision_line = flipud(collision_line);
    end
    for i = 1:size(collision_line,1)
        if app.handles.field.Field(collision_line(i,1),collision_line(i,2))
            wall_close = 1;
            break
        end
    end
    
    force_turn = 0;
    if wall_close
        collision_line = plotLine(app.handles.field.Car, destinations(1,:));
        for i = 1:size(collision_line,1)
            if app.handles.field.Field(collision_line(i,1),collision_line(i,2))
                force_turn = 1;
                break
            end
        end
    end
    
    if app.handles.Debug %Debug measages
       if wall_close
           disp(app.handles,'wall_close');
       end
       if force_turn
           disp(app.handles,'force_turn');
       end
    end
    
    %Sensitivy angle(threshold for steering or not) depended on distance to
    %destination or distance to a wall;
    if wall_close||(dist(app.handles.field.Car, destinations(1,:)) < radia_sc(2))
        sens_ang = 10;
    else
        sens_ang = 20;
    end
    

    if force_turn||(abs(angl_err) > sens_ang) %make turn
        if angl_err > 0 %turn right
            if app.handles.Debug %Debug message
                disp(app.handles,'turn right selected')
            end

            %Find midpoint and starting angle turn
            [turn_mid,Sangl] = mid_point(app.handles.field.Car,radia_sc(2),app.handles.field.Heading, 'R');
            
            if ~hit
                wp_close = 0;
                if dist(turn_mid, waypoints(1,1:2)) < (radia_sc(2) )% + .1*app.handles.field.Ratio)
                    if size(waypoints,1)>1
                        dir_nleg = angl(waypoints(1:2,1:2));
                        nxt_trn = dir_to_wp - dir_nleg;
                        if nxt_trn <-180
                            nxt_trn = nxt_trn + 360;
                        elseif nxt_trn > 180
                            nxt_trn = nxt_trn - 360;
                        end
                        if nxt_trn > 0
                            wp_close = 1;
                            rerout_flag = 1;
                        end                    
                    end
                    
                    if ~wp_close
                        obj_found = 0;
                        [turn_midR,SanglR] = mid_point(app.handles.field.Car,radia_sc(2),app.handles.field.Heading, 'L');
                        reverse_point = round(turn_midR + extr_pos(SanglR-90,radia_sc(2)));
                        collision_line = plotLine(reverse_point, waypoints(1,1:2));
                        for i = 1:size(collision_line,1)
                            if sum(collision_line(i,:) < 1) ~= 0
                                obj_found = 1;
                                break
                            end
                            if app.handles.field.Field(collision_line(i,1),collision_line(i,2))
                                obj_found = 1;
                                break
                            end
                        end
                        if ~obj_found
                            not_recomm = 0;
                            arcR = arc_sel([SanglR, SanglR-90],drawcircle(turn_midR, radia_sc(2)),'R');
                            for i = 1:size(arcR,1)
                                if app.handles.field.free_field.map(arcR(i,1),arcR(i,2)) == 1
                                    obj_found = 1;
                                    break;
                                elseif app.handles.field.free_field.map(arcR(i,1),arcR(i,2)) == 0
                                    not_recomm = not_recomm + 1;
                                end
                            end
                            if ~obj_found && (not_recomm < size(arcR,1)/2)
                                hit = 1;
                            end
                        end
                    end
                end
                
            end
            
            if ~hit     
                cir_points = drawcircle(turn_mid, radia_sc(2));
                angl_dif_mat = zeros(size(cir_points,1),1);
                for i = 1:size(cir_points,1)
                    %For all points on circle compare heading car and target heading
                    car_dir = angl([turn_mid;cir_points(i,:)])-90;
                    if ~wp_close
                        dirwp = angl([cir_points(i,:);waypoints(1,1:2)]);
                        angl_dif_mat(i)  = car_dir - dirwp;
                    else
                        angl_dif_mat(i)  = car_dir - dir_nleg;
                    end
                    
                    if angl_dif_mat(i) <-180 %Correct angle
                        angl_dif_mat(i)  = angl_dif_mat(i)  + 360;
                    elseif angl_dif_mat(i)  > 180
                        angl_dif_mat(i)  = angl_dif_mat(i)  - 360;
                    end
                end
                %find point of minimum difference and determine angle to
                %that point from middel. Now use outer radius to find
                %possible blockades
                [~,dur_ind] = min(abs(angl_dif_mat));
                Fangl = angl([turn_mid; cir_points(dur_ind,:)]);
                
                ocir_points = drawcircle(turn_mid, radia_sc(3));
                [cir_arc] = arc_sel([Sangl,Fangl],ocir_points,'R');                
                for i = 1:size(cir_arc,1)
                    if app.handles.field.Field(cir_arc(i,1),cir_arc(i,2)) == 1
                        hit = 1;
                        break
                    end
                end      
            end %if ~hit
            
            if ~hit 
                %checks for obstacles in predicted line
                [line_points] = plotLine(cir_points(dur_ind,:), waypoints(1,1:2));                
                for i = 1:size(line_points,1)
                    if app.handles.field.Field(line_points(i,1),line_points(i,2))
                        rerout_flag = 1;
                        break
                    end
                end
                
                Dangl = Sangl - Fangl; %Total amount of angle difference
                if Dangl < 0
                    Dangl = Dangl + 360;
                end
                
                %Calculate drive/braking times to end up on the correct position
                intr_dis = 0;
                br_dis = 0;
                prevT = 0;
                prevV = 0;
                loopvar = 1;                
                while intr_dis + br_dis < 2*pi*radia(2)*(Dangl/360) 
                    cV = TTS_trn(speed_trn-149,loopvar);
                    cT = TimeVec(loopvar);
                    intr_dis = intr_dis + (cT-prevT)*((cV + prevV)/2);
                    br_t = BR_Tfit_trn(BrakeF,cV);
                    br_dis = (cV/2)*br_t;                    
                    prevT = cT;
                    prevV = cV;                    
                    if cT >= driveT
                        break;
                    end
                    loopvar = loopvar + 1;
                end

                if app.handles.Debug
                    disp(app.handles,'right turn cofirmed and plotted');
                    final_point = turn_mid + extr_pos(Sangl - (((intr_dis+ br_dis)*360)/(2*pi*radia(2))),radia_sc(2));
                    if app.handles.field.free_field.active
                        hold(app.handles.field.free_field.axes,'on');
                        plot(app.handles.field.free_field.axes,cir_arc(:,1),cir_arc(:,2));
                        plot(app.handles.field.free_field.axes,turn_mid(1),turn_mid(2),'*k')
                        plot(app.handles.field.free_field.axes,final_point(1),final_point(2),'xb');
                        hold(app.handles.field.free_field.axes,'off');
                    end
                    if app.handles.field.map.active
                        hold(app.handles.field.map.axes,'on');
                        plot(app.handles.field.map.axes,final_point(1),final_point(2),'xb');
                        hold(app.handles.field.map.axes,'off');
                    end
                end
                
                %Send driving commands
                calcT = tic;
                send(app.handles.comm,[{'D100'};{strcat('M',num2str(speed_trn))}]);
                pause(cT-toc(calcT))
                calcT = tic;
                send(app.handles.comm,{strcat('M',num2str(brake_val))});
                pause(br_t-toc(calcT))
                send(app.handles.comm,{'M150'});
                
                %Update expected heading and set driving mode flags
                expected_hdng =  app.handles.field.Heading-((intr_dis + br_dis)*360)/(2*pi*radia(2));
                if expected_hdng <-180
                    expected_hdng  = expected_hdng  + 360;
                elseif expected_hdng  > 180
                    expected_hdng  = expected_hdng  - 360;
                end                
                just_turned = 1;
                turn_dir = 'R';
                just_reversed = 0;            
                
            else %if hit => obstacle in the way of turn or waypoint inside turning radius
%                 rerout_flag = 1;
                %Left circle for reversed driving
                [turn_mid,Sangl] = mid_point(app.handles.field.Car,radia_sc(2),app.handles.field.Heading, 'L');
                
                cir_points = drawcircle(turn_mid, radia_sc(2));
                angl_dif_mat = zeros(size(cir_points,1),1);
                
                for i = 1:size(cir_points,1)
                    car_dir = angl([turn_mid;cir_points(i,:)])+90;
                    
                    dirwp = angl([cir_points(i,:);waypoints(1,1:2)]);
                    angl_dif_mat(i)  = car_dir - dirwp;
                    
                    
                    if angl_dif_mat(i) <-180
                        angl_dif_mat(i)  = angl_dif_mat(i)  + 360;
                    elseif angl_dif_mat(i)  > 180
                        angl_dif_mat(i)  = angl_dif_mat(i)  - 360;
                    end
                end
                
                [~,dur_ind] = min(abs(angl_dif_mat));
                Fangl = angl([turn_mid; cir_points(dur_ind,:)]);
                
                ocir_points = drawcircle(turn_mid, radia_sc(3));
                [cir_arc] = arc_sel([Sangl,Fangl],ocir_points,'R');
                
                turn_not_free = 0;
                not_recomm = 0;
                for i = 1:size(cir_arc,1)
                    if app.handles.field.free_field.map(cir_arc(i,1),cir_arc(i,2)) == 1
                        turn_not_free = 1;
                        break
                    elseif app.handles.field.free_field.map(cir_arc(i,1),cir_arc(i,2)) == 0
                        not_recomm = not_recomm + 1;
                    end
                end
                
                if (not_recomm < size(cir_points,1)/8)&& ~turn_not_free                    
                    if app.handles.Debug
                        disp(app.handles,'right turn aborted, turning back left');
                    end
                    
                    Dangl = Sangl - Fangl;
                    if Dangl < 0
                        Dangl = Dangl + 360;
                    end
                    
                    %Calculate driving duration by integrating speed
                    intr_dis = 0;
                    br_dis = 0;
                    prevT = 0;
                    prevV = 0;
                    loopvar = 1;                    
                    while intr_dis + br_dis < 2*pi*radia(2)*(Dangl/360)
                        cV = TTS_trn(speed_trn-149,loopvar);
                        cT = TimeVec(loopvar);
                        intr_dis = intr_dis + (cT-prevT)*((cV + prevV)/2);
                        br_t = BR_Tfit_trn(BrakeF,cV);
                        br_dis = (cV/2)*br_t;                        
                        prevT = cT;
                        prevV = cV;                        
                        if cT >= driveT
                            break
                        end
                        loopvar = loopvar+1;
                    end
                    
                    if app.handles.Debug && app.handles.field.free_field.active %Debug message for predicted endpoint
                        final_point = turn_mid + extr_pos(Sangl - (((intr_dis+ br_dis)*360)/(2*pi*radia(2))),radia_sc(2));
                        hold(app.handles.field.free_field.axes,'on');
                        plot(app.handles.field.free_field.axes,final_point(1),final_point(2),'xb');
                        plot(app.handles.field.free_field.axes,cir_arc(:,1),cir_arc(:,2));
                        hold(app.handles.field.free_field.axes,'off');
                        if app.handles.field.map.active
                            hold(app.handles.field.map.axes,'on');
                            plot(app.handles.field.map.axes,final_point(1),final_point(2),'xb');
                            hold(app.handles.field.map.axes,'off');
                        end
                    end
                    
                    send(app.handles.comm,{'D200';strcat('M',num2str(150-abs(speed_trn-150)))});
                    pause(cT)
                    send(app.handles.comm,{strcat('M',num2str(150+abs(brake_val-150)))});
                    pause(br_t)
                    send(app.handles.comm,{'M150'});
                    
                    %Update expected heading
                    expected_hdng =  app.handles.field.Heading-((intr_dis + br_dis)*360)/(2*pi*radia(2));
                    if expected_hdng <-180
                        expected_hdng  = expected_hdng  + 360;
                    elseif expected_hdng  > 180
                        expected_hdng  = expected_hdng  - 360;
                    end                    
                    just_turned = 1;
                    turn_dir = 'L';
                    just_reversed = 1;                    
                    
                else
                    %Drive back with duration driveT/2
                    calcT = tic;
                    send(app.handles.comm,{'D150';strcat('M',num2str(150-abs(150-speed_str)))});
                    [~, indT] = min(abs(TimeVec - driveT/2));
                    pause(driveT/2 - toc(calcT))
                    calcT = tic;
                    send(app.handles.comm,{strcat('M',num2str(150+abs(150-brake_val)))});
                    cV = TTS_str(speed_trn-149,indT);
                    br_t = BR_Tfit_str(BrakeF,cV);
                    pause(br_t - toc(calcT))
                    send(app.handles.comm,{'M150'});
                    
                    if app.handles.Debug
                        disp(app.handles,'right turn aborded, driving back')
                        final_point = app.handles.field.Car + extr_pos(app.handles.field.Heading+180,half_str_dis*app.handles.field.Ratio);
                        if app.handles.field.map.active
                            hold(app.handles.field.map.axes,'on');
                            plot(app.handles.field.map.axes,final_point(1),final_point(2),'xb');
                            hold(app.handles.field.map.axes,'off');
                        end
                        if app.handles.field.free_field.active
                            hold(app.handles.field.free_field.axes,'on');
                            plot(app.handles.field.free_field.axes,final_point(1),final_point(2),'xb');
                            hold(app.handles.field.free_field.axes,'off');
                        end
                    end
                    
                    expected_hdng = app.handles.field.Heading;
                    just_turned = 0;
                    just_reversed = 1;
                    %end temp code
                end   
            end %if ~hit/hit

        else%turn left
            if app.handles.Debug %Debug message
                disp(app.handles,'turn left selected')
            end
            %Find midpoint and starting angle turn
            [turn_mid,Sangl] = mid_point(app.handles.field.Car,radia_sc(2),app.handles.field.Heading, 'L');
            
            %If destination inside turning radius we need to reposition => hit = 1;
            if dist(turn_mid, destinations(1,:)) < (radia_sc(2) ) && (dist(app.handles.field.Car, destinations(1,:)) < .35*app.handles.field.Ratio)
                
                hit = 1;
            end
            
            if ~hit
                wp_close = 0;
                if dist(turn_mid, waypoints(1,1:2)) < (radia_sc(2) )% + .1*app.handles.field.Ratio)
                    if size(waypoints,1)>1
                        dir_nleg = angl(waypoints(1:2,1:2));
                        nxt_trn = dir_to_wp - dir_nleg;
                        if nxt_trn <-180
                            nxt_trn = nxt_trn + 360;
                        elseif nxt_trn > 180
                            nxt_trn = nxt_trn - 360;
                        end
                        if nxt_trn < 0
                            wp_close = 1;
                            rerout_flag = 1;
                        end                    
                    end
                    
                    if ~wp_close
                        obj_found = 0;
                        [turn_midL,SanglL] = mid_point(app.handles.field.Car,radia_sc(2),app.handles.field.Heading, 'R');
                        reverse_point = round(turn_midL + extr_pos(SanglL+90,radia_sc(2)));
                        collision_line = plotLine(reverse_point, waypoints(1,1:2));
                        for i = 1:size(collision_line,1)
                            if sum(collision_line(i,:) < 1) ~= 0
                                obj_found = 1;
                                break
                            end
                            if app.handles.field.Field(collision_line(i,1),collision_line(i,2))
                                obj_found = 1;
                                break
                            end
                        end
                        if ~obj_found
                            not_recomm = 0;
                            arcR = arc_sel([SanglL, SanglL+90],drawcircle(turn_midL, radia_sc(2)),'L');
                            for i = 1:size(arcR,1)
                                if app.handles.field.free_field.map(arcR(i,1),arcR(i,2)) == 1
                                    obj_found = 1;
                                    break;
                                elseif app.handles.field.free_field.map(arcR(i,1),arcR(i,2)) == 0
                                    not_recomm = not_recomm + 1;
                                end
                            end
                            if ~obj_found && (not_recomm < size(arcR,1)/2)
                                hit = 1;
                            end
                        end
                    end
                end                
            end
            
            if ~hit
                cir_points = drawcircle(turn_mid, radia_sc(2));
                angl_dif_mat = zeros(size(cir_points,1),1);
                for i = 1:size(cir_points,1)
                    %For all points on circle compare heading car and target heading
                    car_dir = angl([turn_mid;cir_points(i,:)])+90;
                    if ~wp_close
                        dirwp = angl([cir_points(i,:);waypoints(1,1:2)]);
                        angl_dif_mat(i)  = car_dir - dirwp;
                    else
                        angl_dif_mat(i)  = car_dir - dir_nleg;
                    end
                    
                    if angl_dif_mat(i) <-180 %Correct angle
                        angl_dif_mat(i)  = angl_dif_mat(i)  + 360;
                    elseif angl_dif_mat(i)  > 180
                        angl_dif_mat(i)  = angl_dif_mat(i)  - 360;
                    end
                end
                %find point of minimum difference and determine angle to
                %that point from middel. Now use outer radius to find
                %possible blockades
                [~,dur_ind] = min(abs(angl_dif_mat));
                Fangl = angl([turn_mid; cir_points(dur_ind,:)]);
                
                ocir_points = drawcircle(turn_mid, radia_sc(3));
                [cir_arc] = arc_sel([Sangl,Fangl],ocir_points,'L');
                for i = 1:size(cir_arc,1)
                    if app.handles.field.Field(cir_arc(i,1),cir_arc(i,2)) == 1
                        hit = 1;
                        break
                    end
                end
            end %if ~hit
            
            if ~hit
                
                %checks for obstacles in predicted line
                [line_points] = plotLine(cir_points(dur_ind,:), waypoints(1,1:2));                
                for i = 1:size(line_points,1)
                    if app.handles.field.Field(line_points(i,1),line_points(i,2))
                        rerout_flag = 1;
                        break
                    end
                end
                
                Dangl = Fangl - Sangl; %Total amount of angle difference
                if Dangl < 0
                    Dangl = Dangl + 360;
                end
                
                %Calculate drive/braking times to end up on the correct position
                intr_dis = 0;
                br_dis = 0;
                prevT = 0;
                prevV = 0;
                loopvar = 1;                
                while intr_dis + br_dis < 2*pi*radia(2)*(Dangl/360) 
                    cV = TTS_trn(speed_trn-149,loopvar);
                    cT = TimeVec(loopvar);
                    intr_dis = intr_dis + (cT-prevT)*((cV + prevV)/2);
                    br_t = BR_Tfit_trn(BrakeF,cV);
                    br_dis = (cV/2)*br_t;                    
                    prevT = cT;
                    prevV = cV;                    
                    if cT >= driveT
                        break;
                    end
                    loopvar = loopvar + 1;
                end
                
                if app.handles.Debug && app.handles.field.free_field.active %Debug message for predicted endpoint
                    
                    hold(app.handles.field.free_field.axes,'on');
                    
                    hold(app.handles.field.free_field.axes,'off');
                end
                if app.handles.Debug
                    disp(app.handles,'left turn cofirmed and plotted');
                    final_point = turn_mid + extr_pos(Sangl + (((intr_dis+ br_dis)*360)/(2*pi*radia(2))),radia_sc(2));
                    if app.handles.field.free_field.active                        
                        hold(app.handles.field.free_field.axes,'on');
                        plot(app.handles.field.free_field.axes,final_point(1),final_point(2),'xb');
                        plot(app.handles.field.free_field.axes,cir_arc(:,1),cir_arc(:,2));
                        plot(app.handles.field.free_field.axes,turn_mid(1),turn_mid(2),'*k')
                        hold(app.handles.field.free_field.axes,'off');
                    end
                    if app.handles.field.map.active                        
                        hold(app.handles.field.map.axes,'on');
                        plot(app.handles.field.map.axes,final_point(1),final_point(2),'xb');
                        hold(app.handles.field.map.axes,'off');
                    end
                end
                
                %Send driving commands
                calcT = tic;
                send(app.handles.comm,[{'D200'};{strcat('M',num2str(speed_trn))}]);
                pause(cT-toc(calcT))
                calcT = tic;
                send(app.handles.comm,{strcat('M',num2str(brake_val))});
                pause(br_t-toc(calcT))
                send(app.handles.comm,{'M150'});
                
                %Update expected heading and set driving mode flags
                expected_hdng =  app.handles.field.Heading + ((intr_dis + br_dis)*360)/(2*pi*radia(2));
                if expected_hdng <-180
                    expected_hdng  = expected_hdng  + 360;
                elseif expected_hdng  > 180
                    expected_hdng  = expected_hdng  - 360;
                end                
                just_turned = 1;
                turn_dir = 'L';
                just_reversed = 0;            
                
            else %if hit => obstacle in the way of turn or waypoint inside turning radius
%                 rerout_flag = 1;
                %Left circle for reversed driving
                [turn_mid,Sangl] = mid_point(app.handles.field.Car,radia_sc(2),app.handles.field.Heading, 'R');                
                cir_points = drawcircle(turn_mid, radia_sc(2));
                angl_dif_mat = zeros(size(cir_points,1),1);                
                for i = 1:size(cir_points,1)
                    car_dir = angl([turn_mid;cir_points(i,:)])-90;
                    dirwp = angl([cir_points(i,:);waypoints(1,1:2)]);
                    angl_dif_mat(i)  = car_dir - dirwp;
                    if angl_dif_mat(i) <-180
                        angl_dif_mat(i)  = angl_dif_mat(i)  + 360;
                    elseif angl_dif_mat(i)  > 180
                        angl_dif_mat(i)  = angl_dif_mat(i)  - 360;
                    end
                end
                
                [~,dur_ind] = min(abs(angl_dif_mat));
                Fangl = angl([turn_mid; cir_points(dur_ind,:)]);
                
                %ocir points maybe
                ocir_points = drawcircle(turn_mid, radia_sc(3));
                [cir_arc] = arc_sel([Sangl,Fangl],ocir_points,'L');
                
                turn_not_free = 0;
                not_recomm = 0;
                for i = 1:size(cir_arc,1)
                    if app.handles.field.free_field.map(cir_arc(i,1),cir_arc(i,2)) == 1
                        turn_not_free = 1;
                        break
                    elseif app.handles.field.free_field.map(cir_arc(i,1),cir_arc(i,2)) == 0
                        not_recomm = not_recomm + 1;
                    end
                end
                
                if (not_recomm < size(cir_points,1)/8)&& ~turn_not_free
                    if app.handles.Debug
                        disp(app.handles,'left turn aborted, turning back right');
                    end
                    
                    Dangl = Fangl - Sangl;
                    if Dangl < 0
                        Dangl = Dangl + 360;
                    end
                    
                    %Calculate driving duration by integrating speed
                    intr_dis = 0;
                    br_dis = 0;
                    prevT = 0;
                    prevV = 0;
                    loopvar = 1;
                    while intr_dis + br_dis < 2*pi*radia(2)*(Dangl/360)
                        cV = TTS_trn(speed_trn-149,loopvar);
                        cT = TimeVec(loopvar);
                        intr_dis = intr_dis + (cT-prevT)*((cV + prevV)/2);
                        br_t = BR_Tfit_trn(BrakeF,cV);
                        br_dis = (cV/2)*br_t;
                        prevT = cT;
                        prevV = cV;
                        if cT >= driveT
                            break
                        end
                        loopvar = loopvar+1;
                    end
                    
                    if app.handles.Debug && app.handles.field.free_field.active %Debug message for predicted endpoint
                        final_point = turn_mid + extr_pos(Sangl + (((intr_dis+ br_dis)*360)/(2*pi*radia(2))),radia_sc(2));
                        hold(app.handles.field.free_field.axes,'on');
                        plot(app.handles.field.free_field.axes,final_point(1),final_point(2),'xb');
                        plot(app.handles.field.free_field.axes,cir_arc(:,1),cir_arc(:,2));
                        hold(app.handles.field.free_field.axes,'off');
                        if app.handles.field.map.active
                            hold(app.handles.field.map.axes,'on');
                            plot(app.handles.field.map.axes,final_point(1),final_point(2),'xb');
                            hold(app.handles.field.map.axes,'off');
                        end
                    end
                    
                    send(app.handles.comm,{'D100';strcat('M',num2str(150-abs(speed_trn-150)))});
                    pause(cT)
                    send(app.handles.comm,{strcat('M',num2str(150+abs(brake_val-150)))});
                    pause(br_t)
                    send(app.handles.comm,{'M150'});
                    
                    %Update expected heading
                    expected_hdng =  app.handles.field.Heading+((intr_dis + br_dis)*360)/(2*pi*radia(2));
                    if expected_hdng <-180
                        expected_hdng  = expected_hdng  + 360;
                    elseif expected_hdng  > 180
                        expected_hdng  = expected_hdng  - 360;
                    end
                    just_turned = 1;
                    turn_dir = 'R';
                    just_reversed = 1;
                    
                else
                    %Drive back with duration driveT/2
                    calcT = tic;
                    send(app.handles.comm,{'D150';strcat('M',num2str(150-abs(150-speed_str)))});
                    [~, indT] = min(abs(TimeVec - driveT/2));
                    pause(driveT/2 - toc(calcT))
                    calcT = tic;
                    send(app.handles.comm,{strcat('M',num2str(150+abs(150-brake_val)))});
                    cV = TTS_str(speed_trn-149,indT);
                    br_t = BR_Tfit_str(BrakeF,cV);
                    pause(br_t - toc(calcT))
                    send(app.handles.comm,{'M150'});
                    
                    if app.handles.Debug
                        disp(app.handles,'Left turn aborded, driving back')
                        final_point = app.handles.field.Car + extr_pos(app.handles.field.Heading+180,half_str_dis*app.handles.field.Ratio);
                        if app.handles.field.map.active
                            hold(app.handles.field.map.axes,'on');
                            plot(app.handles.field.map.axes,final_point(1),final_point(2),'xb');
                            hold(app.handles.field.map.axes,'off');
                        end
                        if app.handles.field.free_field.active
                            hold(app.handles.field.free_field.axes,'on');
                            plot(app.handles.field.free_field.axes,final_point(1),final_point(2),'xb');
                            hold(app.handles.field.free_field.axes,'off');
                        end
                    end
                    
                    expected_hdng = app.handles.field.Heading;
                    just_turned = 0;
                    just_reversed = 1;
                    %end temp code
                end
            end %if ~hit/hit         
        end%if turn right else turn left
    else %go straight
        
        if app.handles.Debug
            disp(app.handles,'driving straight selected and executing')
        end
        
        dist_des = dist(app.handles.field.Car,destinations(1,:))/app.handles.field.Ratio;
        if dist_des < max_str_dis
            intr_dis = 0;
            br_dis = 0;
            prevT = 0;
            prevV = 0;
            loopvar = 1;
            while dist_des - intr_dis - br_dis > .1
                cV = TTS_str(speed_str-149,loopvar);
                cT = TimeVec(loopvar);
                intr_dis = intr_dis + (cT-prevT)*((cV + prevV)/2);
                br_t = BR_Tfit_str(BrakeF,cV);
                br_dis = (cV/2)*br_t;
                prevT = cT;
                prevV = cV;
                if cT >= driveT
                    break
                end
                loopvar = loopvar+1;
            end
            drivenDis = intr_dis + br_dis;
            cT = cT + .4;
        else
            cT = driveT;
            [~, indT] = min(abs(TimeVec - driveT));
            cV = TTS_str(speed_str-149,indT);
            br_t = BR_Tfit_str(BrakeF,cV);
            drivenDis = max_str_dis;
        end

        if app.handles.Debug && app.handles.field.free_field.active %Debug message for predicted endpoint
            final_point = app.handles.field.Car + extr_pos(app.handles.field.Heading,drivenDis*app.handles.field.Ratio);
            hold(app.handles.field.free_field.axes,'on');
            plot(app.handles.field.free_field.axes,final_point(1),final_point(2),'xb');
            hold(app.handles.field.free_field.axes,'off');
            if app.handles.field.map.active
                hold(app.handles.field.map.axes,'on');
                plot(app.handles.field.map.axes,final_point(1),final_point(2),'xb');
                hold(app.handles.field.map.axes,'off');
            end
        end
        
        calcT = tic;
        send(app.handles.comm,[{'D150'};{strcat('M',num2str(150+abs(150-speed_str)))}]);
        pause(cT - toc(calcT))
        calcT = tic;
        send(app.handles.comm,{strcat('M',num2str(150-abs(150-brake_val)))});
        pause(br_t - toc(calcT))
        send(app.handles.comm,{'M150'});
        
        expected_hdng = app.handles.field.Heading;
        just_turned = 0;
        just_reversed = 0;     

    end %%if turn else straight
    
    cycle = cycle + 1;
end
runtime = toc(app.handles.global_tic)-app.handles.t0;
send(app.handles.comm, {'A0';'M150'});
disp(app.handles,strcat({'This trip took '}, {num2str(cycle)}, {' computing cycles'}));

end