function kalman_start_sim;

main_data = get(gcf,'UserData');

main_data.sim_active = 1;

set(gcf,'UserData',main_data);


while (main_data.sim_active == 1)

    drawnow;
    main_data = get(gcf,'UserData');
    
    %**********************************************************************
    % Generate noise with appropriate power
    %**********************************************************************
    main_data.signals.n     = randn(1,1) * main_data.parameters.sigma_n;    
    main_data.signals.u     = randn(1,1) * main_data.parameters.sigma_u;
    
    %**********************************************************************
    % Measurement equation
    %**********************************************************************   
    main_data.signals.y_old  = main_data.signals.y;
    main_data.signals.y      = main_data.parameters.b * main_data.signals.x ...
                               + main_data.signals.n;
            
                           
    x_old_tmp = main_data.signals.x;                          
    %**********************************************************************
    % Kalman filter
    %**********************************************************************
    main_data.signals.kalman_gain_old = main_data.signals.kalman_gain;
    main_data.signals.star_P_old      = main_data.signals.star_P;
    if (main_data.axes_top.index < 1)
       main_data.signals.P           = main_data.signals.P_0;
       main_data.signals.star_P      = main_data.signals.P_0;
       main_data.signals.kalman_gain = main_data.signals.P * main_data.parameters.c ...
                                       / (main_data.parameters.c * main_data.signals.P ....
                                         * main_data.parameters.c + main_data.signals.N);
       main_data.signals.tilde_P     = main_data.signals.P - main_data.signals.kalman_gain ...
                                       * main_data.parameters.c * main_data.signals.P;
       main_data.signals.hat_x       = main_data.signals.kalman_gain * main_data.signals.y;
    else                
       main_data.signals.hat_x_old   = main_data.signals.hat_x; 
       main_data.signals.star_x      = main_data.parameters.A * main_data.signals.hat_x;
       main_data.signals.star_P      = main_data.parameters.A * main_data.signals.tilde_P * main_data.parameters.A ...
                                       + main_data.parameters.b *  main_data.signals.U * main_data.parameters.b;
       main_data.signals.kalman_gain = main_data.signals.star_P * main_data.parameters.c ...
                                       / ( main_data.parameters.c * main_data.signals.star_P ...
                                         * main_data.parameters.c + main_data.signals.N);
       main_data.signals.hat_x       = main_data.signals.star_x + main_data.signals.kalman_gain ...
                                       * (main_data.signals.y - main_data.parameters.c * ...
                                          main_data.signals.star_x );
       main_data.signals.tilde_P     = (1 - main_data.signals.kalman_gain * main_data.parameters.c) ...
                                       * main_data.signals.star_P;
    end;

    %**********************************************************************
    % Increment index
    %**********************************************************************
    main_data.axes_top.index    = main_data.axes_top.index + 1;
    main_data.axes_bottom.index = main_data.axes_bottom.index + 1;
    
    %**********************************************************************
    % Store data
    %**********************************************************************

    set(gcf,'UserData',main_data);
        
    %**********************************************************************
    % Update of the plots
    %**********************************************************************
    main_data = get(gcf,'UserData');    
    if (main_data.sim_active == 1)
      kalman_update_plot(1);  
      kalman_update_plot(2);
    end;
    
    %**********************************************************************
    % Store old x value
    %**********************************************************************
    main_data = get(gcf,'UserData'); 
    main_data.signals.x_old  = x_old_tmp;
    set(gcf,'UserData',main_data);    
    
    %**********************************************************************
    % Set condition for terminating the while loop
    %**********************************************************************
    if (main_data.auto_mode == 0)
        main_data.sim_active = 0;
    else
        pause(0.1);
    end;    
              
end;

