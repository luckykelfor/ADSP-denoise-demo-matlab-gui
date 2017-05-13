function kalman_plot_init(plot_nr);

main_data = get(gcf,'UserData');

sim_active_tmp       = main_data.sim_active;
main_data.sim_active = 0;
set(gcf,'UserData',main_data);

if (plot_nr == 1)
    tag_axes                                  = 'demo_kalman_axes_top';
    set(gcf,'CurrentAxes',findobj('Tag',tag_axes));
    plot_method                               = main_data.axes_top.plot_method;
    main_data.axes_top.plot_method_changed    = 0;
else
    tag_axes                                  = 'demo_kalman_axes_bottom';
    set(gcf,'CurrentAxes',findobj('Tag',tag_axes)); 
    plot_method                               = main_data.axes_bottom.plot_method;
    main_data.axes_bottom.plot_method_changed = 0;
end;

handle_1 = 0;
handle_2 = 0;

switch plot_method    
    case 'sig_time' 
        cla
        
        main_data.axes_top.plot_handle_1 = line([-1 -1],[0 0]);
        set(main_data.axes_top.plot_handle_1,'Color','b');
        set(main_data.axes_top.plot_handle_1,'LineWidth',1);
        set(main_data.axes_top.plot_handle_1,'EraseMode','none');
        
        main_data.axes_top.plot_handle_2 = line([-1 -1],[0 0]);
        set(main_data.axes_top.plot_handle_2,'Color','r');
        set(main_data.axes_top.plot_handle_2,'LineWidth',1);    
        set(main_data.axes_top.plot_handle_2,'EraseMode','none');
        
        main_data.axes_top.plot_handle_3 = line([-1 -1],[0 0]);
        set(main_data.axes_top.plot_handle_3,'Color','k');
        set(main_data.axes_top.plot_handle_3,'LineWidth',1);    
        set(main_data.axes_top.plot_handle_3,'EraseMode','none');

        legend('True value','Measured signal','Estimated value');          

        hold on;
        
        main_data.axes_top.plot_handle_1a = plot(-1,0,'bo');
        set(main_data.axes_top.plot_handle_1a,'LineWidth',2);
        set(main_data.axes_top.plot_handle_1a,'EraseMode','none');
        
        main_data.axes_top.plot_handle_2a = plot(-1,0,'ro');
        set(main_data.axes_top.plot_handle_2a,'LineWidth',2);
        set(main_data.axes_top.plot_handle_2a,'EraseMode','none');
        
        main_data.axes_top.plot_handle_3a = plot(-1,0,'ko');
        set(main_data.axes_top.plot_handle_3a,'LineWidth',2);
        set(main_data.axes_top.plot_handle_3a,'EraseMode','none');        
        
        axis([1 main_data.axes_top.index_max -abs(main_data.signals.x)-3*main_data.parameters.sigma_n-eps abs(main_data.signals.x)+3*main_data.parameters.sigma_n+eps]);
                      
        grid on;     
        
    case 'kalman_gain'     
        cla
        
        main_data.axes_bottom.plot_handle_1 = line([-1 -1],[0 0]);
        set(main_data.axes_bottom.plot_handle_1,'Color','b');
        set(main_data.axes_bottom.plot_handle_1,'LineWidth',1);
        set(main_data.axes_bottom.plot_handle_1,'EraseMode','none');
        
        hold on;
        
        main_data.axes_bottom.plot_handle_1a = plot(-1,0,'bo');
        set(main_data.axes_bottom.plot_handle_1a,'LineWidth',2);
        set(main_data.axes_bottom.plot_handle_1a,'EraseMode','none');        
        
        axis([1 main_data.axes_top.index_max -0.5 0.5]);
        
        legend('Kalman gain')        
        
        grid on;   
        
    case 'error_variance'     
        cla
        
        main_data.axes_bottom.plot_handle_1 = line([-1 -1],[0 0]);
        set(main_data.axes_bottom.plot_handle_1,'Color','b');
        set(main_data.axes_bottom.plot_handle_1,'LineWidth',1);
        set(main_data.axes_bottom.plot_handle_1,'EraseMode','none');
        
        hold on;
        
        main_data.axes_bottom.plot_handle_1a = plot(-1,0,'bo');
        set(main_data.axes_bottom.plot_handle_1a,'LineWidth',2);
        set(main_data.axes_bottom.plot_handle_1a,'EraseMode','none');          
        
        axis([1 main_data.axes_top.index_max 0 1]);
        
        legend('Error variance')        
        
        grid on;           
end;

drawnow;

set(gca,'Tag',tag_axes);

main_data.sim_active = sim_active_tmp;
set(gcf,'UserData',main_data);