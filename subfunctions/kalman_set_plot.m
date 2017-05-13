function kalman_set_plot(val,axes_nr);

main_data = get(gcf,'UserData');

main_data.sim_active = 0;

if (axes_nr == 1)
    switch val
        case 1
            main_data.axes_top.plot_method = 'sig_time';
    end;
    set(gcf,'UserData',main_data);
    kalman_plot_init(1);
else
    switch val
        case 1
            main_data.axes_bottom.plot_method = 'kalman_gain';
        case 2
            main_data.axes_bottom.plot_method = 'error_variance';            
    end;
    set(gcf,'UserData',main_data);
    kalman_plot_init(2);    
end;

