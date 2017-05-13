function kalman_update_plot(nr);

main_data = get(gcf,'UserData');

%**************************************************************************
% Update plot window 1
%**************************************************************************
if (nr == 1)
    tag_axes = 'demo_kalman_axes_top';
    set(gcf,'CurrentAxes',findobj('Tag',tag_axes));
    plot_method    = main_data.axes_top.plot_method;
    plot_handle_1  = main_data.axes_top.plot_handle_1;
    plot_handle_2  = main_data.axes_top.plot_handle_2;
    plot_handle_3  = main_data.axes_top.plot_handle_3;
    plot_handle_1a = main_data.axes_top.plot_handle_1a;
    plot_handle_2a = main_data.axes_top.plot_handle_2a;
    plot_handle_3a = main_data.axes_top.plot_handle_3a;    
    switch plot_method    
        case 'sig_time'       
           if (main_data.axes_top.index < main_data.axes_top.index_max)
              set(plot_handle_1,'XData',[main_data.axes_top.index-1 main_data.axes_top.index], ...
                                'YData',[main_data.signals.x_old main_data.signals.x]);
              set(plot_handle_2,'XData',[main_data.axes_top.index-1 main_data.axes_top.index], ...
                                'YData',[main_data.signals.y_old main_data.signals.y]);
              set(plot_handle_3,'XData',[main_data.axes_top.index-1 main_data.axes_top.index], ...
                                'YData',[main_data.signals.hat_x_old main_data.signals.hat_x]);                         
              set(plot_handle_1a,'XData',main_data.axes_top.index, ...
                                 'YData',main_data.signals.x);                            
              set(plot_handle_2a,'XData',main_data.axes_top.index, ...
                                 'YData',main_data.signals.y);
              set(plot_handle_3a,'XData',main_data.axes_top.index, ...
                                 'YData',main_data.signals.hat_x);                             
           else
              main_data.axes_top.index = 1;
              set(gcf,'UserData',main_data);
              kalman_plot_init(1);
              main_data = get(gcf,'UserData');
              tag_axes = 'demo_kalman_axes_top';
              set(gcf,'CurrentAxes',findobj('Tag',tag_axes));
              plot_method    = main_data.axes_top.plot_method;
              plot_handle_1  = main_data.axes_top.plot_handle_1;
              plot_handle_2  = main_data.axes_top.plot_handle_2;
              plot_handle_3  = main_data.axes_top.plot_handle_3;   
              plot_handle_1a = main_data.axes_top.plot_handle_1a;
              plot_handle_2a = main_data.axes_top.plot_handle_2a;
              plot_handle_3a = main_data.axes_top.plot_handle_3a;               
              set(plot_handle_1,'XData',[main_data.axes_top.index-1 main_data.axes_top.index], ...
                                'YData',[main_data.signals.x_old main_data.signals.x]);
              set(plot_handle_2,'XData',[main_data.axes_top.index-1 main_data.axes_top.index], ...
                                'YData',[main_data.signals.y_old main_data.signals.y]);
              set(plot_handle_3,'XData',[main_data.axes_top.index-1 main_data.axes_top.index], ...
                                'YData',[main_data.signals.hat_x_old main_data.signals.hat_x]);                                        
              set(plot_handle_1a,'XData',main_data.axes_top.index, ...
                                 'YData',main_data.signals.x);                            
              set(plot_handle_2a,'XData',main_data.axes_top.index, ...
                                 'YData',main_data.signals.y);
              set(plot_handle_3a,'XData',main_data.axes_top.index, ...
                                 'YData',main_data.signals.hat_x);                                 
           end;
    end;    
end;

%**************************************************************************
% Update plot window 2
%**************************************************************************
if (nr == 2)
    tag_axes = 'demo_kalman_axes_bottom';
    set(gcf,'CurrentAxes',findobj('Tag',tag_axes));
    plot_method    = main_data.axes_bottom.plot_method;
    plot_handle_1  = main_data.axes_bottom.plot_handle_1;
    plot_handle_1a = main_data.axes_bottom.plot_handle_1a;
    switch plot_method    
        case 'kalman_gain'       
           if (main_data.axes_bottom.index < main_data.axes_top.index_max)
              set(plot_handle_1,'XData',[main_data.axes_bottom.index-1 main_data.axes_bottom.index], ...
                                'YData',[main_data.signals.kalman_gain_old main_data.signals.kalman_gain]);
              set(plot_handle_1a,'XData',main_data.axes_bottom.index, ...
                                 'YData',main_data.signals.kalman_gain);                            
           else
              main_data.axes_bottom.index = 1;
              set(gcf,'UserData',main_data);
              kalman_plot_init(2);
              main_data = get(gcf,'UserData');
              tag_axes = 'demo_kalman_axes_bottom';
              set(gcf,'CurrentAxes',findobj('Tag',tag_axes));
              plot_method    = main_data.axes_bottom.plot_method;
              plot_handle_1  = main_data.axes_bottom.plot_handle_1;
              plot_handle_1a = main_data.axes_bottom.plot_handle_1a;
              set(plot_handle_1,'XData',[main_data.axes_bottom.index-1 main_data.axes_bottom.index], ...
                                'YData',[main_data.signals.kalman_gain_old main_data.signals.kalman_gain]);
              set(plot_handle_1a,'XData',main_data.axes_bottom.index, ...
                                 'YData',main_data.signals.kalman_gain);                               
           end;
        case 'error_variance'       
           if (main_data.axes_bottom.index < main_data.axes_top.index_max)
              set(plot_handle_1,'XData',[main_data.axes_bottom.index-1 main_data.axes_bottom.index], ...
                                'YData',[main_data.signals.star_P_old main_data.signals.star_P]);
              set(plot_handle_1a,'XData',main_data.axes_bottom.index, ...
                                 'YData',main_data.signals.star_P);
                            
           else
              main_data.axes_bottom.index = 1;
              set(gcf,'UserData',main_data);
              kalman_plot_init(2);
              main_data = get(gcf,'UserData');
              tag_axes = 'demo_kalman_axes_bottom';
              set(gcf,'CurrentAxes',findobj('Tag',tag_axes));
              plot_method    = main_data.axes_bottom.plot_method;
              plot_handle_1  = main_data.axes_bottom.plot_handle_1;
              plot_handle_1a = main_data.axes_bottom.plot_handle_1a;              
              set(plot_handle_1,'XData',[main_data.axes_bottom.index-1 main_data.axes_bottom.index], ...
                                'YData',[main_data.signals.star_P_old main_data.signals.star_P]);
              set(plot_handle_1a,'XData',main_data.axes_bottom.index, ...
                                 'YData',main_data.signals.star_P);                            
           end;           
    end;    
end;